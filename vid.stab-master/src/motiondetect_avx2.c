/*
 *  motiondetect_avx2.c
 *
 *  用于运动检测内循环的AVX2（32字节向量）内核。
 *
 *  此文件使用-mavx2编译，因此除非vs_cpu_flags()报告VS_CPU_AVX2，
 *  否则绝不能进入。它不包含在调度路径本身上运行的代码。
 *
 *  SPDX-License-Identifier: LGPL-2.1-or-later
 *
 *  This file is part of vid.stab video stabilization library
 *
 *  vid.stab is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published
 *  by the Free Software Foundation; either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  vid.stab is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with vid.stab; see the file COPYING.LESSER.  If not, see
 *  <https://www.gnu.org/licenses/>.
 *
 */

#include "motiondetect_opt.h"

#ifdef VS_HAVE_AVX2

#include <immintrin.h>

/* 在测试运行总和与阈值之前要累积的行数。
   1意味着"每一行"，这使得这些内核在每种情况下都返回与compareSubImg_thr / _sse2完全相同的值，
   包括提前退出——这是一个比"大于阈值"更容易测试的属性。
   较粗的节拍是合法的（调用者只将结果用作"error < minerror"，参见tests/test_simd_equivalence.c），
   并且每行节省一次水平归约，但在1080p下测得仅节省约1.5%的端到端性能，
   这不值得放弃精确相等保证。 */
#ifndef VS_AVX2_CHECK_ROWS
#define VS_AVX2_CHECK_ROWS 1
#endif

/* 累积的_mm256_sad_epu8产生的四个64位通道的水平求和。
   每个通道最多保存rows*32*255，所以64位不会溢出，
   并且对于任何真实的场大小，结果都能舒适地放入32位中。 */
/**
 * AVX2 SAD（绝对差和）的水平求和
 * @param v 256位累加器
 * @param tail 128位尾部累加器
 * @return 水平求和结果
 */
static inline unsigned int hsum_sad256(__m256i v, __m128i tail) {
  __m128i lo = _mm256_castsi256_si128(v);
  __m128i hi = _mm256_extracti128_si256(v, 1);
  __m128i s  = _mm_add_epi64(_mm_add_epi64(lo, hi), tail);
  s = _mm_add_epi64(s, _mm_unpackhi_epi64(s, s));
  return (unsigned int)_mm_cvtsi128_si32(s);
}

/**
 * 使用AVX2优化比较两个子图像（带阈值检查）
 * 这是运动检测的核心函数，使用SIMD指令加速计算
 * @param I1 第一幅图像指针
 * @param I2 第二幅图像指针
 * @param field 测量场指针
 * @param linesize1 第一幅图像的行大小
 * @param linesize2 第二幅图像的行大小
 * @param height 图像高度
 * @param bytesPerPixel 每像素字节数
 * @param d_x X方向位移
 * @param d_y Y方向位移
 * @param treshold 提前退出阈值
 * @return 绝对差和（SAD）
 */
unsigned int compareSubImg_thr_avx2(unsigned char* const I1, unsigned char* const I2,
                                    const Field* field,
                                    int linesize1, int linesize2, int height,
                                    int bytesPerPixel, int d_x, int d_y,
                                    unsigned int treshold) {
  int j;
  int s2 = field->size / 2;
  int rowBytes = field->size * bytesPerPixel;
  unsigned int sum = 0;
  unsigned char* p1;
  unsigned char* p2;
  /* field->size是16的倍数（vsMotionDetectInit），所以rowBytes也是：
     一行是整数个32字节块加上最多一个16字节尾部。 */
  int mainBytes = rowBytes & ~31;
  int hasTail   = rowBytes & 16;
  __m256i acc = _mm256_setzero_si256();
  __m128i accTail = _mm_setzero_si128();

  p1 = I1 + (field->x - s2) * bytesPerPixel + (field->y - s2) * linesize1;
  p2 = I2 + (field->x - s2 + d_x) * bytesPerPixel + (field->y - s2 + d_y) * linesize2;

  for (j = 0; j < field->size; j++) {
    int k;
    // 处理32字节对齐的主要部分
    for (k = 0; k < mainBytes; k += 32) {
      __m256i a = _mm256_loadu_si256((__m256i const*)(p1 + k));
      __m256i b = _mm256_loadu_si256((__m256i const*)(p2 + k));
      acc = _mm256_add_epi64(acc, _mm256_sad_epu8(a, b));
    }
    if (hasTail) {
      /* 16字节余数进入它自己的128位累加器，而不是被扩展到`acc`中。
         扩展在每行花费一个vpxor + vinserti128，而行经常是16（mod 32）字节——
         场大小是16的倍数但不是32的倍数，所以720p（80）和1080p（112）
         都每行走一次这个路径，其中开销分别落在行的三分之一和四分之一上。 */
      __m128i a = _mm_loadu_si128((__m128i const*)(p1 + mainBytes));
      __m128i b = _mm_loadu_si128((__m128i const*)(p2 + mainBytes));
      accTail = _mm_add_epi64(accTail, _mm_sad_epu8(a, b));
    }

    /* 提前退出：这个候选已经比目前为止的最佳匹配更差。
       契约（参见tests/test_simd_equivalence.c）只是值
       returned by an early exit is greater than the threshold, not that it
       equals the full sum, so checking every VS_AVX2_CHECK_ROWS rows instead
       of every row is allowed and the argmin the caller computes is
       unchanged. */
    if ((j & (VS_AVX2_CHECK_ROWS - 1)) == (VS_AVX2_CHECK_ROWS - 1)) {
      sum = hsum_sad256(acc, accTail);
      if (sum > treshold)
        return sum;
    }
    p1 += linesize1;
    p2 += linesize2;
  }

  return hsum_sad256(acc, accTail);
}

double contrastSubImg1_avx2(unsigned char* const I, const Field* field,
                            int linesize, int height) {
  int j;
  int s2 = field->size / 2;
  int mainBytes = field->size & ~31;
  int hasTail   = field->size & 16;
  unsigned char* p = I + (field->x - s2) + (field->y - s2) * linesize;
  __m256i vmin = _mm256_set1_epi8((char)0xFF);
  __m256i vmax = _mm256_setzero_si256();
  __m128i lo, hi;
  unsigned char mini, maxi;

  for (j = 0; j < field->size; j++) {
    int k;
    for (k = 0; k < mainBytes; k += 32) {
      __m256i v = _mm256_loadu_si256((__m256i const*)(p + k));
      vmin = _mm256_min_epu8(vmin, v);
      vmax = _mm256_max_epu8(vmax, v);
    }
    if (hasTail) {
      /* Broadcast the 16 byte remainder to both halves: duplicating values is
         harmless for min/max and avoids needing a neutral filler. */
      __m128i t = _mm_loadu_si128((__m128i const*)(p + mainBytes));
      __m256i v = _mm256_broadcastsi128_si256(t);
      vmin = _mm256_min_epu8(vmin, v);
      vmax = _mm256_max_epu8(vmax, v);
    }
    p += linesize;
  }

  /* fold 256 -> 128 -> scalar */
  lo = _mm256_castsi256_si128(vmin);
  hi = _mm256_extracti128_si256(vmin, 1);
  lo = _mm_min_epu8(lo, hi);
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 8));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 4));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 2));
  lo = _mm_min_epu8(lo, _mm_srli_si128(lo, 1));
  mini = (unsigned char)_mm_extract_epi16(lo, 0);

  lo = _mm256_castsi256_si128(vmax);
  hi = _mm256_extracti128_si256(vmax, 1);
  lo = _mm_max_epu8(lo, hi);
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 8));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 4));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 2));
  lo = _mm_max_epu8(lo, _mm_srli_si128(lo, 1));
  maxi = (unsigned char)_mm_extract_epi16(lo, 0);

  return (maxi - mini) / (maxi + mini + 0.1); // +0.1 to avoid division by 0
}

#endif /* VS_HAVE_AVX2 */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   tab-width:  2
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
