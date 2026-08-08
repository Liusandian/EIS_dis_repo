/*
 *  frameinfo.c
 *
 *  Copyright (C) Georg Martius - Feb - 2013
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

#include "frameinfo.h"
#include "vidstabdefines.h"
#include <assert.h>
#include <string.h>

/**
 * 初始化帧信息结构体
 * 根据指定的像素格式设置帧的参数，包括平面数量、色度子采样等
 */
int vsFrameInfoInit(VSFrameInfo* fi, int width, int height, VSPixelFormat pFormat){
  fi->pFormat=pFormat;
  fi->width = width;
  fi->height = height;
  fi->planes=3;                   // 默认3个平面（YUV）
  fi->log2ChromaW = 0;            // 默认无色度子采样
  fi->log2ChromaH = 0;
  fi->bytesPerPixel=1;            // 默认每像素1字节

  // 根据不同的像素格式设置相应参数
  switch(pFormat){
   case PF_GRAY8:                 // 灰度格式只有1个平面
    fi->planes=1;
    break;
   case PF_YUV420P:               // 4:2:0格式：色度水平和垂直都是2:1子采样
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 1;
    break;
   case PF_YUV422P:               // 4:2:2格式：色度水平2:1子采样，垂直无子采样
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 0;
    break;
   case PF_YUV444P:               // 4:4:4格式：无色度子采样
    break;
   case PF_YUV410P:               // 4:1:0格式：色度水平和垂直都是4:1子采样
    fi->log2ChromaW = 2;
    fi->log2ChromaH = 2;
    break;
   case PF_YUV411P:               // 4:1:1格式：色度水平4:1子采样，垂直无子采样
    fi->log2ChromaW = 2;
    fi->log2ChromaH = 0;
    break;
   case PF_YUV440P:               // 4:4:0格式：色度水平无子采样，垂直2:1子采样
    fi->log2ChromaW = 0;
    fi->log2ChromaH = 1;
    break;
   case PF_YUVA420P:              // 带alpha通道的4:2:0格式
    fi->log2ChromaW = 1;
    fi->log2ChromaH = 1;
    fi->planes = 4;               // 4个平面（Y、U、V、A）
    break;
    /* 对于紧缩格式，所有数据都在plane 0中（见VSFrame），所以只有1个平面。
       平面数不能为0：所有的平面循环都是`for(plane=0; plane<fi->planes; plane++)`，
       当planes==0时，不会分配任何内容，也不会复制任何内容。 */
   case PF_RGB24:                 // RGB紧缩格式：24位，3字节每像素
   case PF_BGR24:                 // BGR紧缩格式：24位，3字节每像素
    fi->bytesPerPixel=3;
    fi->planes = 1;
    break;
   case PF_RGBA:                  // RGBA紧缩格式：32位，4字节每像素
    fi->bytesPerPixel=4;
    fi->planes = 1;
    break;
   default:
    fi->pFormat=0;
    return 0;
  }

  /* 运行时验证（断言会在-DNDEBUG的发布版本中被编译掉，导致错误的尺寸
     进入色度平面算术运算并超出分配范围的末尾）。

     只有真正与格式色度子采样不兼容的尺寸才会被拒绝：
     vsFrameAllocate()使用截断位移(width >> log2ChromaW)来确定色度平面大小，
     而变换代码使用向上取整的CHROMA_SIZE()，所以只有当尺寸是子采样因子的倍数时，
     两者才一致。
     无子采样的格式（PF_GRAY8、PF_YUV444P、紧缩RGB格式）完全不限制尺寸，
     例如PF_YUV422P只限制宽度。 */
  if(width<=0 || height<=0){
    vs_log_error("vid.stab","invalid frame dimensions %ix%i: "
                 "width and height must be positive\n", width, height);
    fi->pFormat=0;
    return 0;
  }
  if(width % (1<<fi->log2ChromaW) != 0 || height % (1<<fi->log2ChromaH) != 0){
    vs_log_error("vid.stab","invalid frame dimensions %ix%i for pixel format %i: "
                 "width must be a multiple of %i and height a multiple of %i "
                 "for this chroma subsampling\n", width, height, (int)pFormat,
                 1<<fi->log2ChromaW, 1<<fi->log2ChromaH);
    fi->pFormat=0;
    return 0;
  }
  return 1;
}

/**
 * 获取指定平面的水平子采样偏移量
 * 色度平面（plane 1和2）有子采样，亮度平面（plane 0）无子采样
 */
int vsGetPlaneWidthSubS(const VSFrameInfo* fi, int plane){
  return plane == 1 || plane == 2 ? fi->log2ChromaW : 0;
}

/**
 * 获取指定平面的垂直子采样偏移量
 * 色度平面（plane 1和2）有子采样，亮度平面（plane 0）无子采样
 */
int vsGetPlaneHeightSubS(const VSFrameInfo* fi, int plane){
  return  plane == 1 || plane == 2 ? fi->log2ChromaH : 0;
}

/**
 * 检查帧是否为空
 * 空帧定义为frame指针为NULL或data[0]为NULL
 */
int vsFrameIsNull(const VSFrame* frame) {
  return frame==0 || frame->data[0]==0;
}


/**
 * 比较两个帧是否相同
 * 相同定义为两个帧指针相同或data[0]指针相同
 */
int vsFramesEqual(const VSFrame* frame1,const VSFrame* frame2){
  return frame1 && frame2 && (frame1==frame2 || frame1->data[0] == frame2->data[0]);
}

/**
 * 将帧结构体初始化为零
 * 清空所有数据指针和行大小
 */
void vsFrameNull(VSFrame* frame){
  memset(frame->data,0,sizeof(uint8_t*)*4);
  memset(frame->linesize,0,sizeof(int)*4);
}

/**
 * 为帧分配内存
 * 根据帧信息结构体中的格式和尺寸，为各个平面分配内存
 */
void vsFrameAllocate(VSFrame* frame, const VSFrameInfo* fi){
  vsFrameNull(frame);
  if(fi->pFormat<PF_PACKED){
    // 平面格式：为每个平面单独分配内存
    int i;
    assert(fi->planes > 0 && fi->planes <= 4);
    for (i=0; i< fi->planes; i++){
      int w = fi->width  >> vsGetPlaneWidthSubS(fi, i);  // 考虑色度子采样
      int h = fi->height >> vsGetPlaneHeightSubS(fi, i);
      frame->data[i] = vs_zalloc(w * h * sizeof(uint8_t));
      frame->linesize[i] = w;
      if(frame->data[i]==0)
        vs_log_error("vid.stab","out of memory: cannot allocated buffer");
    }
  }else{
    // 紧缩格式：只分配一个平面，包含所有颜色数据
    assert(fi->planes==1);
    int w = fi->width;
    int h = fi->height;
    frame->data[0] = vs_zalloc(w * h * sizeof(uint8_t)*fi->bytesPerPixel);
    frame->linesize[0] = w * fi->bytesPerPixel;
    if(frame->data[0]==0)
      vs_log_error("vid.stab","out of memory: cannot allocated buffer");
  }
}

/**
 * 复制指定平面从源帧到目标帧
 * 处理不同行大小的情况，逐行复制数据
 */
void vsFrameCopyPlane(VSFrame* dest, const VSFrame* src,
                    const VSFrameInfo* fi, int plane){
  assert(src->data[plane]);
  int h = fi->height >> vsGetPlaneHeightSubS(fi, plane);
  if(src->linesize[plane] == dest->linesize[plane])
    // 如果行大小相同，直接复制整个平面
    memcpy(dest->data[plane], src->data[plane], src->linesize[plane] *  h * sizeof(uint8_t));
  else {
    // 行大小不同，需要逐行复制
    uint8_t* d = dest->data[plane];
    const uint8_t* s = src->data[plane];
    // 每行实际图像数据的字节数（对于平面格式，bytesPerPixel为1）
    int w = (fi->width  >> vsGetPlaneWidthSubS(fi, plane)) * fi->bytesPerPixel;
    for (; h>0; h--) {
      memcpy(d,s,sizeof(uint8_t) * w);
      d += dest->linesize[plane];
      s += src ->linesize[plane];
    }
  }
}

/**
 * 复制整个帧从源到目标
 * 遍历所有平面并分别复制
 */
void vsFrameCopy(VSFrame* dest, const VSFrame* src, const VSFrameInfo* fi){
  int plane;
  assert(fi->planes > 0 && fi->planes <= 4);
  for (plane=0; plane< fi->planes; plane++){
    vsFrameCopyPlane(dest,src,fi,plane);
  }
}

/**
 * 从线性缓冲区填充帧数据指针
 * 不执行复制操作，只是设置指针指向缓冲区中的相应位置
 */
void vsFrameFillFromBuffer(VSFrame* frame, uint8_t* img, const VSFrameInfo* fi){
  assert(fi->planes > 0 && fi->planes <= 4);
  vsFrameNull(frame);
  long int offset = 0;
  int i;
  for (i=0; i< fi->planes; i++){
    int w = fi->width  >> vsGetPlaneWidthSubS(fi, i);
    int h = fi->height >> vsGetPlaneHeightSubS(fi, i);
    frame->data[i] = img + offset;
    frame->linesize[i] = w*fi->bytesPerPixel;
    offset += h * w*fi->bytesPerPixel;
  }
}

/**
 * 释放帧占用的内存
 * 释放所有平面的内存并将指针置零
 */
void vsFrameFree(VSFrame* frame){
  int plane;
  for (plane=0; plane< 4; plane++){
    if(frame->data[plane]) vs_free(frame->data[plane]);
    frame->data[plane]=0;
    frame->linesize[plane]=0;
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
