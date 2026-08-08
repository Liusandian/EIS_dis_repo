# VidStab 视频稳像库

[![C/C++ CI](https://github.com/georgmartius/vid.stab/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/georgmartius/vid.stab/actions/workflows/c-cpp.yml)

Vidstab 是一个视频稳像库，可以作为插件与 Ffmpeg 配合使用。

**为什么需要这个工具**

使用手持相机或安装在车辆上的相机拍摄的视频通常会出现不受欢迎的抖动和颤动。在冲浪、滑雪、骑行和步行时拍摄视频等活动特别容易出现相机抖动。Vidstab 针对这些视频内容，帮助创建更平滑、更稳定的视频。

**一些主要功能包括：**

* 快速检测连续帧之间的变换，例如在给定范围内的平移和旋转。
* 低通滤波平滑处理，可调节地平线。
* L1最优相机路径（Grundmann等人，CVPR 2011）：稳定路径由静态、线性和抛物线段组成，产生的结果看起来像是来自推拉镜头或摇臂，而不是低通滤波器。作为线性程序求解，内置求解器，因此不需要额外依赖。它由常用参数控制：`zoom`/`optzoom` 给出它可以花费的裁剪预算，`smoothing` 路径应保持刚性的地平线。
* 智能快速多测量场检测算法，具有对比度选择功能。
* 裁剪选项：保持空白（黑色）或从前一帧保持。
* 可选绘制测量场和检测到的变换以进行视觉分析。
* 可以放大以消除抖动边框（自动模式）。
* 结果图像进行插值处理（不同算法）。
* 虚拟三脚架模式，提供三脚架体验。

**注意：** 本说明文档介绍了如何将 vidstab 与 Ffmpeg 一起使用。欢迎通过 georg dot martius @ web dot de 提问。

## 系统要求
* 基于 Linux 的系统
* ffmpeg 源代码
* Cmake

L1最优相机路径使用内置内点求解器求解，不需要其他依赖。也可以使用 GLPK，通过 `cmake -DVIDSTAB_LPSOLVER=glpk`；测试套件会针对两者运行，并与 `docs/l1campath-reference.py` 中的参考实现进行交叉检查。

## 安装说明

要在 ffmpeg 中使用 vidstab 库，必须使用 `--enable-libvidstab` 选项配置 ffmpeg。

### 默认构建和安装：
##### 安装 vidstab 库：

```shell
cd path/to/vid.stab/dir/
cmake .
make
sudo make install
```

##### 安装 ffmpeg：

```shell
cd path/to/ffmpeg/dir/
./configure --enable-gpl --enable-libvidstab <其他配置选项>
make
sudo make install
```

**关于 `--enable-gpl`：** vidstab 本身是 LGPL-2.1-or-later 许可证，不需要它，但 ffmpeg 的 `configure` 仍然将 `--enable-libvidstab` 限制在 `--enable-gpl` 后面，因此在构建 ffmpeg 时仍然需要它。这是 ffmpeg 的打包决定，不是此库的许可证要求。

### 或者，可以以这种方式将 vidstab 安装到自定义目录：
##### 安装 vidstab 库：

```shell
cd path/to/vid.stab/dir/
cmake -DCMAKE_INSTALL_PREFIX:PATH=path/to/install_dir/
make
sudo make install
```

##### 安装 ffmpeg：

```shell
cd path/to/ffmpeg/dir/
PKG_CONFIG_PATH="path/to/install_dir/lib/pkgconfig" \
./configure --enable-gpl --enable-libvidstab <其他可选配置选项>
make
sudo make install
```

在首次运行 ffmpeg 之前，确保导出 `LD_LIBRARY_PATH` 指向 vidstab 库，例如：

```shell
export LD_LIBRARY_PATH=path/to/install_dir/lib:$LD_LIBRARY_PATH
```

## 使用说明

**目前与 ffmpeg 一起使用时，vidstab 库必须在双遍模式下运行。** 第一遍使用 **vidstabdetect** 滤镜，第二遍使用 **vidstabtransform** 滤镜。

*如果您需要单遍处理，ffmpeg 自带的 [deshake](http://www.ffmpeg.org/ffmpeg-filters.html#deshake) 滤镜可以做到，尽管 vidstab 双遍滤镜提供更优的结果。*

vidstabdetect 滤镜（在第一遍中）将生成一个包含连续帧之间相对平移和旋转变换信息的文件。然后 vidstabtransform 滤镜（在第二遍中）将读取此信息以补偿抖动运动并产生稳定的视频输出。

确保使用 ffmpeg 提供的 [unsharp](http://www.ffmpeg.org/ffmpeg-filters.html#unsharp-1) 滤镜以获得最佳效果（仅在第二遍中）。

注意：10位 4:2:2 视频必须下采样到 8位 4:2:0 以避免色度偏移或颜色溢出/涂抹等失真（请参见下面的 `format=yuv420p` 示例）。

*请参阅 [ffmpeg 滤镜列表](http://www.ffmpeg.org/ffmpeg-filters.html) 以了解有关 vidstabdetect、vidstabtransform 和 ffmpeg 提供的所有其他滤镜的更多信息。*

### 隔行视频

**在稳像之前进行去隔行处理。** Vidstab 没有场的概念：它将每个输入帧视为一个渐进图像。向其提供隔行材料意味着两个场——相隔半帧周期，因此显示不同的运动——被分析为单个图像，因此梳状伪影被测量为图像内容，并且检测到的运动是两个场的无意义平均值。然后变换将两个场一起移动，这不会消除抖动，并将梳状伪影涂抹到整个帧上。

在两遍中都在 `vidstabdetect` 之前放置去隔行器，例如使用 [yadif](http://www.ffmpeg.org/ffmpeg-filters.html#yadif-1)：

```shell
ffmpeg -i input.mkv -vf yadif,vidstabdetect -f null -
ffmpeg -i input.mkv -vf yadif,vidstabtransform,unsharp=5:5:0.8:3:3:0.4 out_stabilized.mp4
```

在两遍中使用相同的去隔行器设置，因为变换是在第一遍看到的帧像素中测量的。如果您去隔行到双倍速率（`yadif=1`，每场一帧），这没问题——只需在两遍中都这样做，以便帧数对齐。不支持稳像和重新隔行以保持隔行交付物。

### 变换文件

第一遍写入的 `.trf` 文件记录在 [docs/trf-format.md](docs/trf-format.md) 中——包括编码、存储值的含义以及如何提供您自己的相机路径。

### vidstab 滤镜的可用选项：

##### 第一遍（vidstabdetect 滤镜）：

<dl>
  <dt><b>result</b></dt>
  <dd>设置用于写入变换信息的文件路径。默认值为 <b>transforms.trf</b>。</dd>
  <dt><b>shakiness</b></dt>
  <dd>设置输入视频的抖动程度或相机的快速程度。它接受 1-10 范围内的整数，值为 1 表示轻微抖动，值为 10 表示强烈抖动。默认值为 5。</dd>
  <dt><b>accuracy</b></dt>
  <dd>设置检测过程的准确性。它必须是 1-15 范围内的值。值为 1 表示低准确性，值为 15 表示高准确性。默认值为 15。</dd>
  <dt><b>stepsize</b></dt>
  <dd>设置搜索过程的步长。最小值周围区域以 1 像素分辨率扫描。默认值为 6。</dd>
  <dt><b>mincontrast</b></dt>
  <dd>设置最小对比度。任何对比度低于此值的测量场都将被丢弃。必须是 0-1 范围内的浮点值。默认值为 0.3。</dd>
  <dt><b>tripod</b></dt>
  <dd>  设置三脚架模式的参考帧号。如果启用，帧的运动将与过滤流中的参考帧进行比较，参考帧由指定编号标识。目的是补偿或多或少静态场景中的所有运动，并使相机视图绝对静止。如果设置为 0，则禁用。帧从 1 开始计数。
  <br>参考帧之前的帧不会稳定：不对它们应用校正（它们仍然像片段的其余部分一样被放大/裁剪），并且稳定在参考帧处开始，视图会捕捉到参考姿势。
  <br>注意：如果在第一遍中使用此模式，则也应在第二遍中使用此模式。</dd>
  <dt><b>show</b></dt>
  <dd>在结果帧中显示场和变换以进行视觉分析。它接受 0-2 范围内的整数。默认值为 0，它禁用任何可视化。
  <br>在三脚架模式下，参考帧之前的帧不显示叠加，因为不测量它们的运动。</dd>
</dl>

##### 示例：
  使用默认值：
```shell
ffmpeg -i input.mp4 -vf vidstabdetect -f null -
```

  *` -f null - ` 确保不产生输出，因为这只是第一遍。这反过来会导致更快的速度。*

  分析强烈抖动的视频并将结果放在文件 `mytransforms.trf` 中：
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=shakiness=10:accuracy=15:result="mytransforms.trf" -f null -
```

  在结果视频中可视化内部变换的结果：
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=show=1 dummy_output.mp4
```

  分析高抖动视频：
```shell
ffmpeg -i input.mp4 -vf vidstabdetect=shakiness=10 dummy_output.mp4
```

  将 10位 4:2:2 文件下采样到 8位 4:2:0 以避免色度偏移和颜色溢出/涂抹等失真：
```shell
ffmpeg -i input.mp4 -vf format=yuv420p,vidstabdetect -f null -
```

##### 第二遍（vidstabtransform 滤镜）：
<dl>
  <dt><b>input</b></dt>
  <dd>设置用于读取变换的文件路径。默认值为 <b>transforms.trf</b>。</dd>
  <dt><b>smoothing</b></dt>
  <dd>设置用于低通滤波相机运动的帧数（值*2 + 1）。默认值为 10。<br>例如，数字 10 表示使用 21 帧（过去 10 帧和未来 10 帧）来平滑视频中的运动。较大的值会导致视频更平滑，但限制了相机的加速度（平移/倾斜运动）。0 是一个特殊情况，模拟静态相机。</dd>
  <dt><b>optalgo</b></dt>
  <dd>设置相机路径优化算法。接受的值有：
  <br><i><b>gauss：</b></i> 相机运动上的高斯核低通滤波器（默认）。
  <br><i><b>avg：</b></i> 变换上的平均。</dd>
  <dt><b>maxshift</b></dt>
  <dd>设置平移帧的最大像素数。默认值为 -1，表示：无限制。</dd>
  <dt><b>maxangle</b></dt>
  <dd>设置旋转帧的最大角度（以弧度为单位，度*PI/180）。默认值为 -1，表示：无限制。</dd>
  <dt><b>crop</b></dt>
  <dd>  指定如何处理由于运动补偿而可能缩进的空帧边框。可用值有：
  <br><i><b>keep</b></i>：保持前一帧的图像信息（默认）。
  <br><i><b>black</b></i>：将边框区域填充为黑色。</dd>
  <dt><b>invert</b></dt>
  <dd>如果设置为 1，则反转变换。默认值为 0。</dd>
  <dt><b>relative</b></dt>
  <dd>如果设置为 1，则将变换视为相对于前一帧，如果设置为 0，则视为绝对。默认值为 0。</dd>
  <dt><b>zoom</b></dt>
  <dd>设置放大的百分比。正值将导致放大效果，负值将导致缩小效果。默认值为 0（无缩放）。</dd>
  <dt><b>optzoom</b></dt>
  <dd>设置最佳缩放以避免空白边框。接受的值有：
  <br><i><b>0</b></i>：禁用。
  <br><i><b>1</b></i>：确定最佳静态缩放值（只有非常强烈的运动才会导致可见边框）（默认）。
  <br><i><b>2</b></i>：确定最佳自适应缩放值（不会有可见边框），请参见 <b>zoomspeed</b>。
  <br>请注意，此处计算的值将添加到 zoom 给定的值中。</dd>
  <dt><b>zoomspeed</b></dt>
  <dd>设置每帧最大缩放百分比（当 optzoom 设置为 2 时启用）。范围从 0 到 5，默认值为 0.25。</dd>
  <dt><b>interpol</b></dt>
  <dd>指定插值类型。可用值有：
  <br><i><b>no</b></i>：无插值。
  <br><i><b>linear</b></i>：仅水平线性。
  <br><i><b>bilinear</b></i>：双向线性（默认）。
  <br><i><b>bicubic</b></i>：双向立方（速度慢）。</dd>
  <dt><b>tripod</b></dt>
  <dd>如果设置为 1，则启用虚拟三脚架模式，这相当于 <b>relative=0:smoothing=0</b>。默认值为 0。
  <br>存储的变换按原样使用：不应用平滑或累积，它们只是被传递，包括参考帧之前帧的前导零变换。哪些帧不稳定是在第一遍中决定的。
  <br>注意：如果在第一遍中使用了此模式，则也应在第二遍中使用此模式。</dd>
  <dt><b>debug</b></dt>
  <dd>如果设置为 1，则增加日志详细程度。此外，检测到的全局运动将写入临时文件 <b>global_motions.trf</b>。默认值为 0。</dd>

</dl>

##### 示例：
  使用默认值：
```shell
ffmpeg -i input.mp4 -vf vidstabtransform,unsharp=5:5:0.8:3:3:0.4 out_stabilized.mp4
```
注意使用 ffmpeg 的 unsharp 滤镜，这总是被推荐的。

再多放大一点并从给定文件加载变换数据：
```shell
ffmpeg -i input.mp4 -vf vidstabtransform=zoom=5:input="mytransforms.trf" out_stabilized.mp4
```

进一步平滑视频：
```shell
ffmpeg -i input.mp4 -vf vidstabtransform=smoothing=30:input="mytransforms.trf" out_stabilized.mp4
```

将 10位 4:2:2 文件下采样到 8位 4:2:0 以避免色度偏移和颜色溢出/涂抹等失真：
```shell
ffmpeg -i input.mp4 -vf format=yuv420p,vidstabtransform out_stabilized.mp4
```

## 开发/贡献

Vidstab 是一个开源库——非常欢迎拉取请求。一些您可能喜欢帮助我们的事情：

 **我正在为这个项目寻找新的维护者/开发者！** 如果您感兴趣，请联系我（Georg）。

 * vidstab 达不到标准的特定视频片段。
 * 错误/修复。
 * 新功能和改进。
 * 文档。

## 许可证

**GNU Lesser General Public License，版本 2.1 或更高版本**
([COPYING.LESSER](./COPYING.LESSER))。v1.1.2 及之前的版本是 GPL；请参见 [RELICENSE.md](./RELICENSE.md)。
