/*
 *  frameinfo.h
 *
 *  Copyright (C) Georg Martius - June 2007 - 2011
 *   georg dot martius at web dot de
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
#ifndef FRAMEINFO_H
#define FRAMEINFO_H

#include <stddef.h>
#include <stdlib.h>
#include <inttypes.h>
#include "vidstab_api.h"
/// 像素格式枚举
/// 定义了视频帧处理支持的各种颜色格式
typedef enum {PF_NONE = -1,
              PF_GRAY8,     ///< 灰度图像格式，8位每像素
              PF_YUV420P,   ///< 平面YUV 4:2:0格式，12位每像素，每2x2个Y样本对应1个Cr和1个Cb样本
              PF_YUV422P,   ///< 平面YUV 4:2:2格式，16位每像素，每2x1个Y样本对应1个Cr和1个Cb样本
              PF_YUV444P,   ///< 平面YUV 4:4:4格式，24位每像素，每1x1个Y样本对应1个Cr和1个Cb样本
              PF_YUV410P,   ///< 平面YUV 4:1:0格式，9位每像素，每4x4个Y样本对应1个Cr和1个Cb样本
              PF_YUV411P,   ///< 平面YUV 4:1:1格式，12位每像素，每4x1个Y样本对应1个Cr和1个Cb样本
              PF_YUV440P,   ///< 平面YUV 4:4:0格式，每1x2个Y样本对应1个Cr和1个Cb样本
              PF_YUVA420P,  ///< 带alpha通道的平面YUV 4:2:0格式，20位每像素
              PF_PACKED,    ///< 标志位：紧缩格式从此开始
              PF_RGB24,     ///< 紧缩RGB 8:8:8格式，24位每像素，RGBRGB...排列
              PF_BGR24,     ///< 紧缩BGR 8:8:8格式，24位每像素，BGRBGR...排列
              PF_RGBA,      ///< 紧缩RGBA 8:8:8:8格式，32位每像素，RGBARGBA...排列
              PF_NUMBER     ///< 像素格式总数，用于边界检查
} VSPixelFormat;

/** 视频帧信息结构体，用于视频防抖库
    仅支持平面图像格式
 */
typedef struct vsframeinfo {
  int width, height;           // 帧的宽度和高度
  int planes;                  // 平面数量（1个亮度平面，2-3个色度平面，4个alpha平面）
  int log2ChromaW;             // 色度平面宽度的子采样因子（以2为底的对数）
  int log2ChromaH;             // 色度平面高度的子采样因子（以2为底的对数）
  VSPixelFormat pFormat;       // 像素格式类型
  int bytesPerPixel;           // 每像素字节数（仅用于紧缩格式）
} VSFrameInfo;

/** 视频帧数据结构体，对应frameinfo
    包含图像数据和每行字节数信息
 */
typedef struct vsframe {
  uint8_t* data[4];    // 各平面的数据指针，对于紧缩格式所有数据都在plane 0中
  int linesize[4];     // 各平面每行的字节数（考虑内存对齐）
} VSFrame;

// 用于计算色度平面尺寸的宏（向上取整，保证计算正确）
#define CHROMA_SIZE(width,log2sub)  (-(-(width) >> (log2sub)))

/** 初始化指定格式的帧信息结构体

    尺寸必须与请求格式的色度子采样兼容，即宽度必须是1<<log2ChromaW的倍数，
    高度必须是1<<log2ChromaH的倍数。无子采样的格式（PF_GRAY8、PF_YUV444P和紧缩格式）
    接受任意正尺寸，因此奇数宽度/高度对它们是有效的。

    @return 成功返回非零值(1)，如果像素格式未知或尺寸无效则返回0。
            注意布尔约定：这*不是*VS_OK/VS_ERROR。
 */
VS_API int vsFrameInfoInit(VSFrameInfo* fi, int width, int height, VSPixelFormat pFormat);


/// 返回指定平面的水平子采样偏移量
VS_API int vsGetPlaneWidthSubS(const VSFrameInfo* fi, int plane);

/// 返回指定平面的垂直子采样偏移量
VS_API int vsGetPlaneHeightSubS(const VSFrameInfo* fi, int plane);

/// 零初始化帧结构体
VS_API void vsFrameNull(VSFrame* frame);

/// 如果帧为空（data[0]==0）则返回真
VS_API int vsFrameIsNull(const VSFrame* frame);

/// 比较两个帧是否相同（基于data[0]指针）
VS_API int vsFramesEqual(const VSFrame* frame1,const VSFrame* frame2);

/// 为帧分配内存
VS_API void vsFrameAllocate(VSFrame* frame, const VSFrameInfo* fi);


/// 从源帧复制指定平面到目标帧
VS_API void vsFrameCopyPlane(VSFrame* dest, const VSFrame* src,
                    const VSFrameInfo* fi, int plane);

/// 从源帧复制所有平面到目标帧
VS_API void vsFrameCopy(VSFrame* dest, const VSFrame* src, const VSFrameInfo* fi);

/** 填充数据指针，使其对应于保存在线性缓冲区中的图像。
    不执行复制操作。
    不要对其调用vsFrameFree()。
 */
VS_API void vsFrameFillFromBuffer(VSFrame* frame, uint8_t* img, const VSFrameInfo* fi);

/// 释放帧内存
VS_API void vsFrameFree(VSFrame* frame);

#endif  /* FRAMEINFO_H */

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 *   c-basic-offset: 2 t
 * End:
 *
 * vim: expandtab shiftwidth=2:
 */
