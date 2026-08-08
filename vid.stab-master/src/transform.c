/*
 *  transform.c
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

#include "transform.h"
#include "transform_internal.h"
#include "transformtype_operations.h"
#include "l1campathoptimization.h"

#include "transformfixedpoint.h"
#ifdef TESTING
#include "transformfloat.h"
#endif

#include <math.h>
#ifndef _MSC_VER
    #include <libgen.h>
#endif
#include <string.h>

// 插值类型名称数组
const char* interpol_type_names[5] = {"No (0)", "Linear (1)", "Bi-Linear (2)",
                                      "Bi-Cubic (3)"};

/**
 * 获取插值类型的名称
 * @param type 插值类型枚举值
 * @return 插值类型的名称字符串，未知类型返回"unknown"
 */
const char* getInterpolationTypeName(VSInterpolType type){
  if (type >= VS_Zero && type < VS_NBInterPolTypes)
    return interpol_type_names[(int) type];
  else
    return "unknown";
}

/**
 * 获取默认的变换配置
 * 注意：ffmpeg过滤器无法调用此函数
 * @param modName 模块名称（用于日志记录）
 * @return 默认的变换配置结构体
 */
VSTransformConfig vsTransformGetDefaultConfig(const char* modName){
  VSTransformConfig conf;
  /* 选项设置 */
  conf.maxShift           = -1;          // 无最大位移限制
  conf.maxAngle           = -1;          // 无最大角度限制
  conf.crop               = VSKeepBorder;// 保留边界
  conf.relative           = 1;           // 使用相对变换
  conf.invert             = 0;           // 不反转变换
  conf.smoothing          = 15;          // 平滑窗口大小
  conf.zoom               = 0;           // 无缩放
  conf.optZoom            = 1;           // 最优静态缩放
  conf.zoomSpeed          = 0.25;        // 缩放速度
  conf.interpolType       = VS_BiLinear; // 双线性插值
  conf.verbose            = 0;           // 不显示详细日志
  conf.modName            = modName;     // 模块名称
  conf.simpleMotionCalculation = 0;      // 不使用简单运动计算
  conf.storeTransforms    = 0;           // 不存储变换
  conf.smoothZoom         = 0;           // 不平滑缩放
  conf.camPathAlgo        = VSOptimalL1; // L1最优相机路径算法
  return conf;
}

/**
 * 获取当前的变换配置
 * @param conf 输出参数，用于存储配置
 * @param td 变换数据结构体指针
 */
void vsTransformGetConfig(VSTransformConfig* conf, const VSTransformData* td){
  if(td && conf)
    *conf = td->conf;
}

/**
 * 获取源帧的帧信息
 * @param td 变换数据结构体指针
 * @return 源帧信息结构体指针
 */
const VSFrameInfo* vsTransformGetSrcFrameInfo(const VSTransformData* td){
  return &td->fiSrc;
}

/**
 * 获取目标帧的帧信息
 * @param td 变换数据结构体指针
 * @return 目标帧信息结构体指针
 */
const VSFrameInfo* vsTransformGetDestFrameInfo(const VSTransformData* td){
  return &td->fiDest;
}


/**
 * 初始化变换数据结构体
 * 根据配置和帧信息设置变换处理所需的数据结构
 * @param td 变换数据结构体指针
 * @param conf 变换配置指针
 * @param fi_src 源帧信息指针
 * @param fi_dest 目标帧信息指针
 * @return 成功返回VS_OK，失败返回VS_ERROR
 */
int vsTransformDataInit(VSTransformData* td, const VSTransformConfig* conf,
                        const VSFrameInfo* fi_src, const VSFrameInfo* fi_dest){
  td->conf = *conf;

  td->fiSrc = *fi_src;
  td->fiDest = *fi_dest;

  vsFrameNull(&td->src);
  td->srcMalloced = 0;

  vsFrameNull(&td->destbuf);
  vsFrameNull(&td->dest);

  // 限制最大位移不超过帧尺寸的一半
  if (td->conf.maxShift > td->fiDest.width/2)
    td->conf.maxShift = td->fiDest.width/2;
  if (td->conf.maxShift > td->fiDest.height/2)
    td->conf.maxShift = td->fiDest.height/2;

  // 限制插值类型在有效范围内
  td->conf.interpolType = VS_MAX(VS_MIN(td->conf.interpolType,VS_BiCubic),VS_Zero);

  // 根据插值类型设置相应的插值函数
  switch(td->conf.interpolType){
   case VS_Zero:     td->interpolate = &interpolateZero; break;
   case VS_Linear:   td->interpolate = &interpolateLin; break;
   case VS_BiLinear: td->interpolate = &interpolateBiLin; break;
   case VS_BiCubic:  td->interpolate = &interpolateBiCub; break;
   default: td->interpolate = &interpolateBiLin;
  }
#ifdef TESTING
  // 测试模式下也设置浮点版本的插值函数
  switch(td->conf.interpolType){
   case VS_Zero:     td->_FLT(interpolate) = &_FLT(interpolateZero); break;
   case VS_Linear:   td->_FLT(interpolate) = &_FLT(interpolateLin); break;
   case VS_BiLinear: td->_FLT(interpolate) = &_FLT(interpolateBiLin); break;
   case VS_BiCubic:  td->_FLT(interpolate) = &_FLT(interpolateBiCub); break;
   default: td->_FLT(interpolate)          = &_FLT(interpolateBiLin);
  }

#endif
  return VS_OK;
}

/**
 * 清理变换数据结构体
 * 释放分配的内存资源
 * @param td 变换数据结构体指针
 */
void vsTransformDataCleanup(VSTransformData* td){
  // 释放内部分配的源帧缓冲区
  if (td->srcMalloced && !vsFrameIsNull(&td->src)) {
    vsFrameFree(&td->src);
  }
  // 如果保留边界模式，释放目标缓冲区
  if (td->conf.crop == VSKeepBorder && !vsFrameIsNull(&td->destbuf)) {
    vsFrameFree(&td->destbuf);
  }
}

/**
 * 准备变换操作
 * 设置源帧和目标帧，处理就地操作和边界保留情况
 * @param td 变换数据结构体指针
 * @param src 源帧指针
 * @param dest 目标帧指针
 * @return 成功返回VS_OK，失败返回VS_ERROR
 */
int vsTransformPrepare(VSTransformData* td, const VSFrame* src, VSFrame* dest){
  // 我们首先将帧复制到td->src，然后用变换后的版本覆盖目标
  td->dest = *dest;
  if(src==dest || td->srcMalloced){ // 就地操作：我们必须先复制源帧
    // 在复制之前我们必须拥有td->src。在这里测试vsFrameIsNull()是不够的：
    // 在前一帧走了下面的else分支后，td->src仍然别名那个调用者的缓冲区，
    // 所以复制会写入我们不拥有的内存（例如仍在使用的解码器参考帧）。参见#144。
    if(!td->srcMalloced) {
      vsFrameAllocate(&td->src,&td->fiSrc);
      td->srcMalloced = 1;
    }
    if (vsFrameIsNull(&td->src)) {
      vs_log_error(td->conf.modName, "vs_malloc failed\n");
      return VS_ERROR;
    }
    vsFrameCopy(&td->src, src, &td->fiSrc);
  }else{ // 否则不需要复制
    td->src=*src;
  }
  if (td->conf.crop == VSKeepBorder) {
    if(vsFrameIsNull(&td->destbuf)) {
      // 如果我们保留边界，我们需要第二个缓冲区来存储前一个稳定帧，所以我们使用destbuf
      vsFrameAllocate(&td->destbuf,&td->fiDest);
      if (vsFrameIsNull(&td->destbuf)) {
        vs_log_error(td->conf.modName, "vs_malloc failed\n");
        return VS_ERROR;
      }
      // 如果我们保留边界，将第一帧保存到背景缓冲区(destbuf)
      vsFrameCopy(&td->destbuf, src, &td->fiSrc); // 这里我们需要注意
    }
  }else{ // 否则我们直接在目标上操作
    td->destbuf = *dest;
  }
  return VS_OK;
}

/**
 * 执行实际的变换操作
 * 根据像素格式选择相应的变换函数
 * @param td 变换数据结构体指针
 * @param t 要应用的变换
 * @return 成功返回VS_OK，失败返回VS_ERROR
 */
int vsDoTransform(VSTransformData* td, VSTransform t){
  if (td->fiSrc.pFormat < PF_PACKED)
    return transformPlanar(td, t);  // 平面格式变换
  else
    return transformPacked(td, t);   // 紧缩格式变换
}


/**
 * 完成变换操作
 * 处理边界保留模式下的帧复制
 * @param td 变换数据结构体指针
 * @return 成功返回VS_OK，失败返回VS_ERROR
 */
int vsTransformFinish(VSTransformData* td){
  if(td->conf.crop == VSKeepBorder){
    // 我们必须将结果存储到视频缓冲区
    // 注意：destbuf存储稳定帧作为下一帧的默认值
    vsFrameCopy(&td->dest, &td->destbuf, &td->fiSrc);
  }
  return VS_OK;
}


/**
 * 获取下一个变换
 * 从变换序列中返回下一个变换，并增加内部计数器
 * @param td 变换数据结构体指针
 * @param trans 变换序列指针
 * @return 下一个变换，如果没有足够的变换则返回最后一个
 */
VSTransform vsGetNextTransform(const VSTransformData* td, VSTransformations* trans){
  if(trans->len <=0 ) return null_transform();
  if (trans->current >= trans->len) {
    trans->current = trans->len;
    if(!trans->warned_end)
      vs_log_warn(td->conf.modName, "not enough transforms found, use last transformation!\n");
    trans->warned_end = 1;
  }else{
    trans->current++;
  }
  return trans->ts[trans->current-1];
}

/**
 * 初始化变换序列结构体
 * @param trans 变换序列指针
 */
void vsTransformationsInit(VSTransformations* trans){
  trans->ts = 0;
  trans->len = 0;
  trans->current = 0;
  trans->warned_end = 0;
}

/**
 * 清理变换序列结构体
 * 释放变换数组内存
 * @param trans 变换序列指针
 */
void vsTransformationsCleanup(VSTransformations* trans){
  if (trans->ts) {
    vs_free(trans->ts);
    trans->ts = NULL;
  }
  trans->len=0;
}

/*
 *  这是消除视频中抖动的核心算法。
 *  我们有不同的实现，在这里进行选择。
 */
int cameraPathOptimization(VSTransformData* td, VSTransformations* trans){
  switch(td->conf.camPathAlgo){
   case VSAvg: return cameraPathAvg(td,trans);
   case VSOptimalL1:
#ifdef VS_HAVE_LPSOLVER
    /* L1优化通过组合相对变换来构建相机路径，所以绝对变换通过构造而不是失败使其超出范围。
       那是三脚架配置(relative=0:smoothing=0)，其中存储的变换已经是最终的每帧校正，
       高斯滤波器保持它们不变——这正是三脚架想要的。
       直接去那里，所以正确的设置保持安静。 */
    if(td->conf.relative){
      /* cameraPathOptimalL1除非成功否则保持trans不变，所以回退到高斯滤波器是安全的。
         它对于短于4帧的序列、没有缩放预算以及如果LP证明不可行时都会失败。 */
      if(cameraPathOptimalL1(td,trans)==VS_OK) return VS_OK;
      vs_log_msg(td->conf.modName,
                 "L1 camera path optimization unavailable, using gaussian filter");
    }
#endif // without an LP solver we always use the gaussian filter
    /* fall through */
   case VSGaussian: return cameraPathGaussian(td,trans);
  }
  return VS_ERROR;
}

/*
 *  我们对相机路径执行低通滤波。
 *  这支持缓慢的相机移动，但是以平滑的方式。
 *  这里我们使用高斯滤波器（高斯核）低通滤波器
 */
int cameraPathGaussian(VSTransformData* td, VSTransformations* trans){
  VSTransform* ts = trans->ts;
  if (trans->len < 1)
    return VS_ERROR;
  if (td->conf.verbose & VS_DEBUG) {
    vs_log_msg(td->conf.modName, "Preprocess transforms:");
  }

  /* 相对到绝对（积分变换） */
  if (td->conf.relative) {
    VSTransform t = ts[0];
    for (int i = 1; i < trans->len; i++) {
      ts[i] = add_transforms(&ts[i], &t);
      t = ts[i];
    }
  }

  if (td->conf.smoothing>0) {
    // 创建变换副本用于滤波
    VSTransform* ts2 = vs_malloc(sizeof(VSTransform) * trans->len);
    memcpy(ts2, ts, sizeof(VSTransform) * trans->len);
    int s = td->conf.smoothing * 2 + 1;
    VSArray kernel = vs_array_new(s);
    // 初始化高斯核
    int mu        = td->conf.smoothing;
    double sigma2 = sqr(mu/2.0);
    for(int i=0; i<=mu; i++){
      kernel.dat[i] = kernel.dat[s-i-1] = exp(-sqr(i-mu)/sigma2);
    }
    // vs_array_print(kernel, stdout);

    // 对每个变换进行高斯滤波
    for (int i = 0; i < trans->len; i++) {
      // 进行卷积运算：
      double weightsum=0;
      VSTransform avg = null_transform();
      for(int k=0; k<s; k++){
        int idx = i+k-mu;
        if(idx>=0 && idx<trans->len){
          if(unlikely(0 && ts2[idx].extra==1)){ // 处理场景切换或坏帧
            if(k<mu) { // 在我们帧的过去：忽略之前的一切
              avg=null_transform();
              weightsum=0;
              continue;
            }else{           //当前帧或在未来：停在这里
              if(k==mu)      //对于当前帧：完全忽略
                weightsum=0;
              break;
            }
          }
          weightsum+=kernel.dat[k];
          avg=add_transforms_(avg, mult_transform(&ts2[idx], kernel.dat[k]));
        }
      }
      if(weightsum>0){
        avg = mult_transform(&avg, 1.0/weightsum);

        // 高频必须被变换掉
        ts[i] = sub_transforms(&ts[i], &avg);
      }
      if (td->conf.verbose & VS_DEBUG) {
        vs_log_msg(td->conf.modName,
                   " avg: %5lf, %5lf, %5lf extra: %i weightsum %5lf",
                   avg.x, avg.y, avg.alpha, ts[i].extra, weightsum
                  );
      }
    }
    vs_array_free(kernel);
    vs_free(ts2);
  }
  return VS_OK;
}

/*
 *  We perform a low-pass filter in terms of transformations.
 *  This supports slow camera movement (low frequency), but in a smooth fasion.
 *  Here a simple average based filter
 */
int cameraPathAvg(VSTransformData* td, VSTransformations* trans){
  VSTransform* ts = trans->ts;

  if (trans->len < 1)
    return VS_ERROR;
  if (td->conf.verbose & VS_DEBUG) {
   vs_log_msg(td->conf.modName, "Preprocess transforms:");
  }
  if (td->conf.smoothing>0) {
    /* smoothing */
    VSTransform* ts2 = vs_malloc(sizeof(VSTransform) * trans->len);
    memcpy(ts2, ts, sizeof(VSTransform) * trans->len);

    /*  we will do a sliding average with minimal update
     *   \hat x_{n/2} = x_1+x_2 + .. + x_n
     *   \hat x_{n/2+1} = x_2+x_3 + .. + x_{n+1} = x_{n/2} - x_1 + x_{n+1}
     *   avg = \hat x / n
     */
    int s = td->conf.smoothing * 2 + 1;
    VSTransform null = null_transform();
    /* avg is the average over [-smoothing, smoothing] transforms
       around the current point */
    VSTransform avg;
    /* avg2 is a sliding average over the filtered signal! (only to past)
     *  with smoothing * 2 horizon to kill offsets */
    VSTransform avg2 = null_transform();
    double tau = 1.0/(2 * s);
    /* initialise sliding sum with hypothetic sum centered around
     * -1st element. We have two choices:
     * a) assume the camera is not moving at the beginning
     * b) assume that the camera moves and we use the first transforms
     */
    VSTransform s_sum = null;
    for (int i = 0; i < td->conf.smoothing; i++){
      s_sum = add_transforms(&s_sum, i < trans->len ? &ts2[i]:&null);
    }
    mult_transform(&s_sum, 2); // choice b (comment out for choice a)

    for (int i = 0; i < trans->len; i++) {
      VSTransform* old = ((i - td->conf.smoothing - 1) < 0)
        ? &null : &ts2[(i - td->conf.smoothing - 1)];
      VSTransform* new = ((i + td->conf.smoothing) >= trans->len)
        ? &null : &ts2[(i + td->conf.smoothing)];
      s_sum = sub_transforms(&s_sum, old);
      s_sum = add_transforms(&s_sum, new);

      avg = mult_transform(&s_sum, 1.0/s);

      /* lowpass filter:
       * meaning high frequency must be transformed away
       */
      ts[i] = sub_transforms(&ts2[i], &avg);
      /* kill accumulating offset in the filtered signal*/
      avg2 = add_transforms_(mult_transform(&avg2, 1 - tau),
                             mult_transform(&ts[i], tau));
      ts[i] = sub_transforms(&ts[i], &avg2);

      if (td->conf.verbose & VS_DEBUG) {
        vs_log_msg(td->conf.modName,
                   "s_sum: %5lf %5lf %5lf, ts: %5lf, %5lf, %5lf\n",
                   s_sum.x, s_sum.y, s_sum.alpha,
                   ts[i].x, ts[i].y, ts[i].alpha);
        vs_log_msg(td->conf.modName,
                   "  avg: %5lf, %5lf, %5lf avg2: %5lf, %5lf, %5lf",
                   avg.x, avg.y, avg.alpha,
                   avg2.x, avg2.y, avg2.alpha);
      }
    }
    vs_free(ts2);
  }
  /* relative to absolute */
  if (td->conf.relative) {
    VSTransform t = ts[0];
    for (int i = 1; i < trans->len; i++) {
      ts[i] = add_transforms(&ts[i], &t);
      t = ts[i];
    }
  }
  return VS_OK;
}


/**
 * vsPreprocessTransforms: camera path optimization, relative to absolute conversion,
 *  and cropping of too large transforms.
 *
 * Parameters:
 *            td: transform private data structure
 *         trans: list of transformations (changed)
 * Return value:
 *     1 for success and 0 for failure
 * Preconditions:
 *     None
 * Side effects:
 *     td->trans will be modified
 */
int vsPreprocessTransforms(VSTransformData* td, VSTransformations* trans)
{
  // works inplace on trans
  if(cameraPathOptimization(td, trans)!=VS_OK) return VS_ERROR;
  VSTransform* ts = trans->ts;
  /*  invert? */
  if (td->conf.invert) {
    for (int i = 0; i < trans->len; i++) {
      ts[i] = mult_transform(&ts[i], -1);
    }
  }

  /* crop at maximal shift */
  if (td->conf.maxShift != -1)
    for (int i = 0; i < trans->len; i++) {
      ts[i].x     = VS_CLAMP(ts[i].x, -td->conf.maxShift, td->conf.maxShift);
      ts[i].y     = VS_CLAMP(ts[i].y, -td->conf.maxShift, td->conf.maxShift);
    }
  if (td->conf.maxAngle != - 1.0)
    for (int i = 0; i < trans->len; i++)
      ts[i].alpha = VS_CLAMP(ts[i].alpha, -td->conf.maxAngle, td->conf.maxAngle);

  /* Calc optimal zoom (1)
   *  cheap algo is to only consider translations
   *  uses cleaned max and min to eliminate 99% of transforms
   */
  if (td->conf.optZoom == 1 && trans->len > 1){
    VSTransform min_t, max_t;
    cleanmaxmin_xy_transform(ts, trans->len, 1, &min_t, &max_t);  // 99% of all transformations
    // the zoom value only for x
    double zx = 2*VS_MAX(max_t.x,fabs(min_t.x))/td->fiSrc.width;
    // the zoom value only for y
    double zy = 2*VS_MAX(max_t.y,fabs(min_t.y))/td->fiSrc.height;
    td->conf.zoom += 100 * VS_MAX(zx,zy); // use maximum
    td->conf.zoom = VS_CLAMP(td->conf.zoom,-60,60);
    vs_log_info(td->conf.modName, "Final zoom: %lf\n", td->conf.zoom);
  }
  /* Calc optimal zoom (2)
   *  sliding average to zoom only as much as needed also using rotation angles
   *  the baseline zoom is the mean required zoom + global zoom
   *  in order to avoid too much zooming in and out
   */
  if (td->conf.optZoom == 2 && trans->len > 1){
    double* zooms=(double*)vs_zalloc(sizeof(double)*trans->len);
    int w = td->fiSrc.width;
    int h = td->fiSrc.height;
    double req;
    double meanzoom;
    for (int i = 0; i < trans->len; i++) {
      zooms[i] = transform_get_required_zoom(&ts[i], w, h);
    }

    double prezoom = 0.;
    double postzoom = 0.;
    if(td->conf.zoom>0.){
      prezoom = td->conf.zoom;
    } else if(td->conf.zoom < 0.){
      postzoom = td->conf.zoom;
    }

    meanzoom = mean(zooms, trans->len) + prezoom; // add global zoom
    // forward - propagation (to make the zooming smooth)
    req = meanzoom;
    for (int i = 0; i < trans->len; i++) {
      req = VS_MAX(req, zooms[i]);
      ts[i].zoom=VS_MAX(ts[i].zoom,req);
      req= VS_MAX(meanzoom, req - td->conf.zoomSpeed); // zoom-out each frame
    }
    // backward - propagation
    req = meanzoom;
    for (int i = trans->len-1; i >= 0; i--) {
      req = VS_MAX(req, zooms[i]);
      ts[i].zoom=VS_MAX(ts[i].zoom,req) + postzoom;
      req= VS_MAX(meanzoom, req - td->conf.zoomSpeed);
    }
    vs_free(zooms);
  }else if (td->conf.zoom != 0){ /* apply global zoom */
    for (int i = 0; i < trans->len; i++)
      ts[i].zoom += td->conf.zoom;
  }

  return VS_OK;
}


/**
 * vsLowPassTransforms: single step smoothing of transforms, using only the past.
 *  see also vsPreprocessTransforms. Here only relative transformations are
 *  considered (produced by motiondetection). Also cropping of too large transforms.
 *
 * Parameters:
 *            td: transform private data structure
 *           mem: memory for sliding average transformation
 *         trans: current transform (from previous to current frame)
 * Return value:
 *         new transformation for current frame
 * Preconditions:
 *     None
 */
VSTransform vsLowPassTransforms(VSTransformData* td, VSSlidingAvgTrans* mem,
                                const VSTransform* trans)
{

  if (!mem->initialized){
    // use the first transformation as the average camera movement
    mem->avg=*trans;
    mem->initialized=1;
    mem->zoomavg=0.0;
    mem->accum = null_transform();
    return mem->accum;
  }else{
    double s = 1.0/(td->conf.smoothing + 1);
    double tau = 1.0/(3.0 * (td->conf.smoothing + 1));
    if(td->conf.smoothing>0){
      // otherwise do the sliding window
      mem->avg = add_transforms_(mult_transform(&mem->avg, 1 - s),
                                 mult_transform(trans, s));
    }else{
      mem->avg = *trans;
    }

    /* lowpass filter:
     * meaning high frequency must be transformed away
     */
    VSTransform newtrans = sub_transforms(trans, &mem->avg);

    /* relative to absolute */
    if (td->conf.relative) {
      newtrans = add_transforms(&newtrans, &mem->accum);
      mem->accum = newtrans;
      if(td->conf.smoothing>0){
        // kill accumulating effects
        mem->accum = mult_transform(&mem->accum, 1.0 - tau);
      }
    }

    /* crop at maximal shift */
    if (td->conf.maxShift != -1){
      newtrans.x     = VS_CLAMP(newtrans.x, -td->conf.maxShift, td->conf.maxShift);
      newtrans.y     = VS_CLAMP(newtrans.y, -td->conf.maxShift, td->conf.maxShift);
    }
    if (td->conf.maxAngle != - 1.0)
      newtrans.alpha = VS_CLAMP(newtrans.alpha, -td->conf.maxAngle, td->conf.maxAngle);

    /* Calc sliding optimal zoom
     *  cheap algo is to only consider translations and to sliding avg
     */
    if (td->conf.optZoom != 0 && td->conf.smoothing > 0){
      // the zoom value only for x
      double zx = 2*newtrans.x/td->fiSrc.width;
      // the zoom value only for y
      double zy = 2*newtrans.y/td->fiSrc.height;
      double reqzoom = 100* VS_MAX(fabs(zx),fabs(zy)); // maximum is requried zoom
      mem->zoomavg = (mem->zoomavg*(1-s) + reqzoom*s);
      // since we only use past it is good to aniticipate
      //  and zoom a little in any case (so set td->zoom to 2 or so)
      newtrans.zoom = mem->zoomavg;
    }
    if (td->conf.zoom != 0){
      newtrans.zoom += td->conf.zoom;
    }
    return newtrans;
  }
}

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
