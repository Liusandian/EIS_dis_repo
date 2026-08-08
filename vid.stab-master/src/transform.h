/*
 *  transform.h
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
#ifndef __TRANSFORM_H
#define __TRANSFORM_H

#include <math.h>
#ifndef _MSC_VER
    #include <libgen.h>
#endif
#include <stdint.h>
#include "transformtype.h"
#include "frameinfo.h"
#include "vidstabdefines.h"
#include "vidstab_api.h"
#ifdef TESTING
#include "transformfloat.h"
#endif


/**
 * 变换序列结构体
 * 存储一系列帧的变换信息
 */
typedef struct _vstransformations {
    VSTransform* ts;        // 变换数组
    int current;            // 当前变换的索引
    int len;                // 变换数组的长度
    short warned_end;       // 是否已警告没有剩余变换
} VSTransformations;

/**
 * 滑动平均变换结构体
 * 用于实时平滑变换序列
 */
typedef struct _vsslidingavgtrans {
    VSTransform avg;        // 平均变换
    VSTransform accum;      // 用于相对到绝对转换的累加器
    double zoomavg;         // 平均缩放值
    short initialized;      // 是否已初始化
} VSSlidingAvgTrans;


/// 插值类型枚举
typedef enum { VS_Zero, VS_Linear, VS_BiLinear, VS_BiCubic, VS_NBInterPolTypes} VSInterpolType;

/// 返回插值类型的名称
VS_API const char* getInterpolationTypeName(VSInterpolType type);

/// 边界处理类型枚举
typedef enum { VSKeepBorder = 0, VSCropBorder } VSBorderType;
/// 相机路径算法枚举
typedef enum { VSOptimalL1 = 0, VSGaussian, VSAvg } VSCamPathAlgo;

/**
 * 插值函数指针类型：用于单通道图像数据的通用插值函数
 * 用于定点数/计算
 * 参数:
 *             rv: 目标像素（引用传递）
 *            x,y: 图像img中的源坐标。注意这是实值坐标（定点格式24.8），
 *                 这就是我们需要插值的原因
 *            img: 源图像
 *   width,height: 图像尺寸
 *            def: 坐标超出范围时的默认值
 * 返回值:  无
 */
typedef void (*vsInterpolateFun)(uint8_t *rv, int32_t x, int32_t y,
                                 const uint8_t *img, int linesize,
                                 int width, int height, uint8_t def);

/**
 * 变换配置结构体
 * 控制视频变换和防抖处理的各种参数
 */
typedef struct _VSTransformConfig {

    /* 是否将变换视为相对（相对于前一帧）变换或绝对变换 */
    int            relative;    // 1:相对变换，0:绝对变换
    /* 用于平滑变换的帧数（向前和向后） */
    int            smoothing;   // 平滑窗口大小
    VSBorderType   crop;        // 边界处理：1:黑色背景，0:保留上一帧的边界
    int            invert;      // 1:反转变换，0:不反转
    double         zoom;        // 缩放百分比：0->无缩放，10:放大10%
    int            optZoom;     // 2:最优自适应缩放，1:最优静态缩放，0:无优化
    double         zoomSpeed;   // 自适应缩放：每帧缩放百分比
    VSInterpolType interpolType; // 插值类型：0->无插值，1->线性，2->双线性，3->双三次
    int            maxShift;    // 最大位移像素数
    double         maxAngle;    // 最大旋转角度（弧度）
    const char*    modName;     // 模块名称（用于日志记录）
    int            verbose;     // 日志级别
    // 如果为1，则使用简单但快速的方法来确定全局运动
    int            simpleMotionCalculation;
    int            storeTransforms; // 将计算得到的变换存储到文件
    int            smoothZoom;   // 如果为1，缩放也会被平滑。通常不推荐。
    VSCamPathAlgo  camPathAlgo;  // 用于相机路径优化的算法
    /* L1最优相机路径（VSOptimalL1）没有自己的参数：
     * 它从zoom/optZoom读取缩放预算，从smoothing读取地平线，
     * 参见vsL1ConfigFromTransformConfig()。 */
} VSTransformConfig;

/**
 * 变换数据结构体
 * 包含视频变换处理所需的所有状态和缓冲区
 */
typedef struct _VSTransformData {
    VSFrameInfo fiSrc;          // 源帧信息
    VSFrameInfo fiDest;         // 目标帧信息

    VSFrame src;                // 当前帧缓冲区的副本
    VSFrame destbuf;            // 指向附加缓冲区或目标缓冲区（取决于crop设置）
    VSFrame dest;               // 指向目标缓冲区

    short srcMalloced;          // 如果源缓冲区是内部分配的则为1

    vsInterpolateFun interpolate; // 指向插值函数的指针
#ifdef TESTING
    _FLT(vsInterpolateFun) _FLT(interpolate); // 测试用的浮点版本插值函数
#endif

    /* 配置选项 */
    VSTransformConfig conf;     // 变换配置

    int initialized;            // 1表示已初始化，2表示已配置
} VSTransformData;


static const char vs_transform_help[] = ""
    "Overview\n"
    "    Reads a file with transform information for each frame\n"
    "     and applies them. See also filter stabilize.\n"
    "Options\n"
    "    'input'     path to the file used to read the transforms\n"
    "                (def: inputfile.trf)\n"
    "    'smoothing' number of frames*2 + 1 used for lowpass filtering \n"
    "                used for stabilizing (def: 10)\n"
    "    'maxshift'  maximal number of pixels to translate image\n"
    "                (def: -1 no limit)\n"
    "    'maxangle'  maximal angle in rad to rotate image (def: -1 no limit)\n"
    "    'crop'      0: keep border (def), 1: black background\n"
    "    'invert'    1: invert transforms(def: 0)\n"
    "    'relative'  consider transforms as 0: absolute, 1: relative (def)\n"
    "    'zoom'      percentage to zoom >0: zoom in, <0 zoom out (def: 0)\n"
    "    'optzoom'   0: nothing, 1: determine optimal static zoom (def)\n"
    "                i.e. no (or only little) border should be visible.\n"
    "                2: determine optimal adaptive zoom\n"
    "                Note that the value given at 'zoom' is added to the \n"
    "                here calculated one\n"
    "    'zoomspeed' for adaptive zoom: zoom per frame in percent \n"
    "    'interpol'  type of interpolation: 0: no interpolation, \n"
    "                1: linear (horizontal), 2: bi-linear (def), \n"
    "                3: bi-cubic\n"
    "    'sharpen'   amount of sharpening: 0: no sharpening (def: 0.8)\n"
    "                uses filter unsharp with 5x5 matrix\n"
    "    'tripod'    virtual tripod mode (=relative=0:smoothing=0)\n"
    "    'help'      print this help message\n";

/** 返回默认配置
 */
VS_API VSTransformConfig vsTransformGetDefaultConfig(const char* modName);

/** 使用配置初始化VSTransformData结构体并为帧等分配内存
 *  @return 成功返回VS_OK，否则返回VS_ERROR
 */
VS_API int vsTransformDataInit(VSTransformData* td, const VSTransformConfig* conf,
                        const VSFrameInfo* fi_src, const VSFrameInfo* fi_dest);


/** 删除内部数据结构
 * 要再次使用VSTransformData，必须调用vsTransformDataInit
 */
VS_API void vsTransformDataCleanup(VSTransformData* td);

/// 返回当前配置
VS_API void vsTransformGetConfig(VSTransformConfig* conf, const VSTransformData* td);

/// 返回源帧的帧信息
VS_API const VSFrameInfo* vsTransformGetSrcFrameInfo(const VSTransformData* td);
/// 返回目标帧的帧信息
VS_API const VSFrameInfo* vsTransformGetDestFrameInfo(const VSTransformData* td);


/// 初始化VSTransformations结构体
VS_API void vsTransformationsInit(VSTransformations* trans);
/// 删除VSTransformations内部内存
VS_API void vsTransformationsCleanup(VSTransformations* trans);

/// 返回下一个变换并增加内部计数器
VS_API VSTransform vsGetNextTransform(const VSTransformData* td, VSTransformations* trans);

/** 一次性预处理变换列表。这里计算防抖！
 */
VS_API int vsPreprocessTransforms(VSTransformData* td, VSTransformations* trans);

/**
 * vsLowPassTransforms: 单步平滑变换，仅使用过去的数据。
 *  另见vsPreprocessTransforms。 */
VS_API VSTransform vsLowPassTransforms(VSTransformData* td, VSSlidingAvgTrans* mem,
                            const VSTransform* trans);

/** 调用此函数为下一个变换做准备（transformPacked/transformPlanar）
    并提供源帧缓冲区和要写入的帧。这些可以是相同的指针，
    用于就地操作（直接在帧缓冲区上工作）
 */
VS_API int vsTransformPrepare(VSTransformData* td, const VSFrame* src, VSFrame* dest);

/// 执行实际的变换
VS_API int vsDoTransform(VSTransformData* td, VSTransform t);


/** 调用此函数完成帧的变换（transformPacked/transformPlanar）
 */
VS_API int vsTransformFinish(VSTransformData* td);


#endif

/*
 * Local variables:
 *   c-file-style: "stroustrup"
 *   c-file-offsets: ((case-label . *) (statement-case-intro . *))
 *   indent-tabs-mode: nil
 * End:
 *
 * vim: expandtab shiftwidth=4:
 */
