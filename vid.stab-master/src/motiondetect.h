/*
 *  motiondetect.h
 *
 *  Copyright (C) Georg Martius - February 2011
 *   georg dot martius at web dot de
 *  Copyright (C) Alexey Osipov - Jule 2011
 *   simba at lerlan dot ru
 *   speed optimizations (threshold, spiral, SSE, asm)
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

#ifndef MOTIONDETECT_H
#define MOTIONDETECT_H

#include <stddef.h>
#include <stdlib.h>

#include "transformtype.h"
#include "vidstabdefines.h"
#include "vsvector.h"
#include "frameinfo.h"
#include "vidstab_api.h"

#define ASCII_SERIALIZATION_MODE 1
#define BINARY_SERIALIZATION_MODE 2

/**
 * 运动检测配置结构体
 * 控制运动检测算法的各种参数
 */
typedef struct _vsmotiondetectconfig {
  /* 元参数，用于控制maxshift和fieldsize，取值范围1-15 */
  int         shakiness;        // 视频抖动程度：1(轻微)到15(严重抖动)
  int         accuracy;         // 检测精度：元参数，控制测量场数量，取值范围1-10
  int         stepSize;         // 场变换检测的步长大小
  int         algo;             // 已弃用
  int         virtualTripod;    // 虚拟三脚架模式
  /* 如果为1或2，则在帧中显示测量场和变换 */
  int         show;             // 0:不显示，1-2:显示场和变换
  /* 对比度低于此阈值的测量场将被丢弃 */
  double      contrastThreshold; // 对比度阈值，取值范围0-1
  const char* modName;          // 模块名称（用于日志记录）
  int         numThreads;       // 使用的线程数（0表示自动设置）
} VSMotionDetectConfig;

/** 运动检测场结构体
    定义用于检测帧间运动的测量场参数
 */
typedef struct _vsmotiondetectfields {
  /* 我们期望的连续帧间最大位移像素数 */
  int maxShift;                 // 最大位移（像素）
  int stepSize;                 // 检测步长
  int fieldNum;                 // 测量场数量
  int maxFields;                // 使用的最大场数（根据对比度选择）
  double contrastThreshold;     // 对比度阈值，低于此值的场将被丢弃
  int fieldSize;                // 场大小 = min(宽度, 高度)/10
  int fieldRows;                // 场的行数
  Field* fields;                // 测量场数组
  short useOffset;              // 如果为true，则使用偏移量
  VSTransform offset;           // 检测偏移量（例如从粗略扫描中获得）
} VSMotionDetectFields;

/** 运动检测数据结构体
    包含视频防抖运动检测部分的所有状态和数据
 */
typedef struct _vsmotiondetect {
  VSFrameInfo fi;               // 帧信息结构体

  VSMotionDetectConfig conf;    // 运动检测配置

  VSMotionDetectFields fieldscoarse;  // 粗略检测场
  VSMotionDetectFields fieldsfine;    // 精细检测场

  VSFrame curr;                 // 当前帧缓冲区的模糊版本
  VSFrame currorig;             // 当前帧缓冲区（原始版本，仅指针）
  VSFrame currtmp;              // 用于模糊处理的临时缓冲区
  VSFrame prev;                 // 上一帧的帧缓冲区（已复制）
  short hasSeenOneFrame;        // 如果有有效的上一帧则为true
  int initialized;              // 1表示已初始化，2表示已配置
  int serializationMode;        // 1表示ASCII模式，2表示二进制模式

  int frameNum;                 // 当前帧编号
} VSMotionDetect;

static const char vs_motiondetect_help[] = ""
    "Overview:\n"
    "    Generates a file with relative transform information\n"
    "     (translation, rotation) about subsequent frames."
    " See also transform.\n"
    "Options\n"
    "    'fileformat'  the type of file format used to write the transforms\n"
    "                  1: ascii (human readable) file format 2: binary (smaller) file format\n"
    "    'result'      path to the file used to write the transforms\n"
    "                  (def:inputfile.stab)\n"
    "    'shakiness'   how shaky is the video and how quick is the camera?\n"
    "                  1: little (fast) 10: very strong/quick (slow) (def: 5)\n"
    "    'accuracy'    accuracy of detection process (>=shakiness)\n"
    "                  1: low (fast) 15: high (slow) (def: 9)\n"
    "    'stepsize'    stepsize of search process, region around minimum \n"
    "                  is scanned with 1 pixel resolution (def: 6)\n"
    "    'mincontrast' below this contrast a field is discarded (0-1) (def: 0.3)\n"
    "    'tripod'      virtual tripod mode (if >0): motion is compared to a \n"
    "                  reference frame (frame # is the value) (def: 0)\n"
    "    'show'        0: draw nothing (def); 1,2: show fields and transforms\n"
    "                  in the resulting frames. Consider the 'preview' filter\n"
    "    'help'        print this help message\n";


/** 返回默认配置
 */
VS_API VSMotionDetectConfig vsMotionDetectGetDefaultConfig(const char* modName);

/** 初始化VSMotionDetect结构体并为帧等分配内存
 *  @return 成功返回VS_OK，否则返回VS_ERROR
 */
VS_API int vsMotionDetectInit(VSMotionDetect* md, const VSMotionDetectConfig* conf,
                       const VSFrameInfo* fi);

/**
 *  执行运动检测步骤
 *  只提供新的当前帧。上一帧存储在内部
 *  @param motions: 计算得到的局部运动（需要手动删除）
 * */
VS_API int vsMotionDetection(VSMotionDetect* md, LocalMotions* motions, VSFrame *frame);

/** 删除内部数据结构
 * 要再次使用VSMotionDetect，必须调用vsMotionDetectInit
 */
VS_API void vsMotionDetectionCleanup(VSMotionDetect* md);

/// 返回当前配置
VS_API void vsMotionDetectGetConfig(VSMotionDetectConfig* conf, const VSMotionDetect* md);

/// 返回帧信息
VS_API const VSFrameInfo* vsMotionDetectGetFrameInfo(const VSMotionDetect* md);

#endif  /* MOTIONDETECT_H */

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
