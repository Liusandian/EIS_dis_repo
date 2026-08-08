"""
开源IMU数据集加载器
支持EuRoC、TUM VI等常用数据集
"""

import os
import cv2
import numpy as np
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass
import struct
import csv


@dataclass
class IMURawData:
    """IMU原始数据"""
    timestamp: float
    angular_velocity: np.ndarray  # [wx, wy, wz] rad/s
    linear_acceleration: np.ndarray  # [ax, ay, az] m/s²


@dataclass
class ImageData:
    """图像数据"""
    timestamp: float
    image_path: str
    image: Optional[np.ndarray] = None


class EuRoCLoader:
    """EuRoC MAV数据集加载器"""

    def __init__(self, dataset_path: str):
        """
        初始化EuRoC数据集加载器

        Args:
            dataset_path: EuRoC数据集根目录路径
        """
        self.dataset_path = dataset_path
        self.imu_data = []
        self.image_data = []

    def load(self, sequence_name: str = 'MH_01_easy'):
        """
        加载EuRoC数据集

        Args:
            sequence_name: 序列名称，如 'MH_01_easy', 'V1_01_easy'
        """
        sequence_path = os.path.join(self.dataset_path, sequence_name)
        if not os.path.exists(sequence_path):
            raise ValueError(f"Sequence not found: {sequence_path}")

        # 加载IMU数据
        self._load_imu_data(sequence_path)

        # 加载图像数据
        self._load_image_data(sequence_path)

        print(f"EuRoC数据集加载完成:")
        print(f"  - IMU数据点: {len(self.imu_data)}")
        print(f"  - 图像帧数: {len(self.image_data)}")

    def _load_imu_data(self, sequence_path: str):
        """加载IMU数据"""
        imu_csv_path = os.path.join(sequence_path, 'mav0', 'imu0', 'data.csv')

        if not os.path.exists(imu_csv_path):
            raise FileNotFoundError(f"IMU data not found: {imu_csv_path}")

        # 读取CSV文件
        with open(imu_csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过标题行

            for row in reader:
                timestamp = float(row[0]) / 1e9  # 转换为秒
                wx, wy, wz = float(row[1]), float(row[2]), float(row[3])
                ax, ay, az = float(row[4]), float(row[5]), float(row[6])

                imu_data = IMURawData(
                    timestamp=timestamp,
                    angular_velocity=np.array([wx, wy, wz]),
                    linear_acceleration=np.array([ax, ay, az])
                )
                self.imu_data.append(imu_data)

    def _load_image_data(self, sequence_path: str):
        """加载图像数据"""
        img_csv_path = os.path.join(sequence_path, 'mav0', 'cam0', 'data.csv')
        img_dir = os.path.join(sequence_path, 'mav0', 'cam0', 'data')

        if not os.path.exists(img_csv_path):
            raise FileNotFoundError(f"Image data not found: {img_csv_path}")

        with open(img_csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过标题行

            for row in reader:
                timestamp = float(row[0]) / 1e9  # 转换为秒
                image_name = row[1]
                image_path = os.path.join(img_dir, image_name)

                image_data = ImageData(
                    timestamp=timestamp,
                    image_path=image_path
                )
                self.image_data.append(image_data)

    def get_synchronized_data(self) -> List[Tuple[ImageData, IMURawData]]:
        """
        获取同步的图像和IMU数据

        Returns:
            同步的数据列表，每个元素为 (image_data, imu_data) 元组
        """
        synchronized = []

        for img_data in self.image_data:
            # 找到最近的IMU数据
            closest_imu = min(self.imu_data,
                            key=lambda imu: abs(imu.timestamp - img_data.timestamp))

            # 检查时间差是否在合理范围内
            if abs(closest_imu.timestamp - img_data.timestamp) < 0.01:  # 10ms以内
                synchronized.append((img_data, closest_imu))

        return synchronized

    def load_image(self, image_path: str) -> np.ndarray:
        """加载图像"""
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Failed to load image: {image_path}")
        return image

    def get_camera_parameters(self, sequence_name: str) -> Dict:
        """获取相机内参"""
        sequence_path = os.path.join(self.dataset_path, sequence_name)
        yaml_path = os.path.join(sequence_path, 'mav0', 'cam0', 'sensor.yaml')

        # 简化的参数解析（实际需要完整的YAML解析器）
        params = {
            'camera_model': 'pinhole',
            'focal_length_x': 458.654,
            'focal_length_y': 457.296,
            'principal_point_x': 367.215,
            'principal_point_y': 248.375,
            'distortion': [-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05]
        }

        return params


class TUMVILoader:
    """TUM VI数据集加载器"""

    def __init__(self, dataset_path: str):
        """
        初始化TUM VI数据集加载器

        Args:
            dataset_path: TUM VI数据集根目录路径
        """
        self.dataset_path = dataset_path
        self.imu_data = []
        self.image_data = []

    def load(self, sequence_name: str = 'room1'):
        """
        加载TUM VI数据集

        Args:
            sequence_name: 序列名称
        """
        sequence_path = os.path.join(self.dataset_path, sequence_name)
        if not os.path.exists(sequence_path):
            raise ValueError(f"Sequence not found: {sequence_name}")

        # 加载IMU数据
        self._load_imu_data(sequence_path)

        # 加载图像数据
        self._load_image_data(sequence_path)

        print(f"TUM VI数据集加载完成:")
        print(f"  - IMU数据点: {len(self.imu_data)}")
        print(f"  - 图像帧数: {len(self.image_data)}")

    def _load_imu_data(self, sequence_path: str):
        """加载IMU数据"""
        imu_csv_path = os.path.join(sequence_path, 'imu', 'imu.txt')

        if not os.path.exists(imu_csv_path):
            raise FileNotFoundError(f"IMU data not found: {imu_csv_path}")

        with open(imu_csv_path, 'r') as f:
            for line in f:
                if line.startswith('#'):
                    continue

                parts = line.strip().split(',')
                timestamp = float(parts[0])

                # TUM VI格式: timestamp,w_x,w_y,w_z,a_x,a_y,a_z
                wx, wy, wz = float(parts[1]), float(parts[2]), float(parts[3])
                ax, ay, az = float(parts[4]), float(parts[5]), float(parts[6])

                imu_data = IMURawData(
                    timestamp=timestamp,
                    angular_velocity=np.array([wx, wy, wz]),
                    linear_acceleration=np.array([ax, ay, az])
                )
                self.imu_data.append(imu_data)

    def _load_image_data(self, sequence_path: str):
        """加载图像数据"""
        img_csv_path = os.path.join(sequence_path, 'cam0', 'data.csv')

        if not os.path.exists(img_csv_path):
            # 尝试其他可能的格式
            img_dir = os.path.join(sequence_path, 'cam0')
            if os.path.exists(img_dir):
                self._load_images_from_directory(img_dir)
                return
            else:
                raise FileNotFoundError(f"Image data not found in {sequence_path}")

        with open(img_csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过标题行

            for row in reader:
                timestamp = float(row[0])
                image_name = row[1]
                image_path = os.path.join(sequence_path, 'cam0', image_name)

                image_data = ImageData(
                    timestamp=timestamp,
                    image_path=image_path
                )
                self.image_data.append(image_data)

    def _load_images_from_directory(self, img_dir: str):
        """从目录加载图像"""
        image_files = sorted([f for f in os.listdir(img_dir) if f.endswith('.png')])

        for img_file in image_files:
            # 从文件名提取时间戳
            try:
                timestamp = float(img_file.split('.')[0])
            except ValueError:
                continue

            image_path = os.path.join(img_dir, img_file)
            image_data = ImageData(
                timestamp=timestamp,
                image_path=image_path
            )
            self.image_data.append(image_data)

    def get_synchronized_data(self) -> List[Tuple[ImageData, IMURawData]]:
        """获取同步的图像和IMU数据"""
        synchronized = []

        for img_data in self.image_data:
            closest_imu = min(self.imu_data,
                            key=lambda imu: abs(imu.timestamp - img_data.timestamp))

            if abs(closest_imu.timestamp - img_data.timestamp) < 0.01:
                synchronized.append((img_data, closest_imu))

        return synchronized

    def load_image(self, image_path: str) -> np.ndarray:
        """加载图像"""
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Failed to load image: {image_path}")
        return image


class GenericIMULoader:
    """通用IMU数据加载器（支持自定义格式）"""

    def __init__(self):
        self.imu_data = []
        self.image_data = []

    def load_imu_from_csv(self, csv_path: str,
                         timestamp_col: int = 0,
                         gyro_cols: List[int] = [1, 2, 3],
                         accel_cols: List[int] = [4, 5, 6],
                         timestamp_scale: float = 1.0):
        """
        从CSV文件加载IMU数据

        Args:
            csv_path: CSV文件路径
            timestamp_col: 时间戳列索引
            gyro_cols: 陀螺仪数据列索引 [wx, wy, wz]
            accel_cols: 加速度计数据列索引 [ax, ay, az]
            timestamp_scale: 时间戳缩放因子
        """
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader)  # 跳过标题行

            for row in reader:
                # 跳过空行
                if not row or len(row) < 7:
                    continue

                timestamp = float(row[timestamp_col]) * timestamp_scale
                wx = float(row[gyro_cols[0]])
                wy = float(row[gyro_cols[1]])
                wz = float(row[gyro_cols[2]])
                ax = float(row[accel_cols[0]])
                ay = float(row[accel_cols[1]])
                az = float(row[accel_cols[2]])

                imu_data = IMURawData(
                    timestamp=timestamp,
                    angular_velocity=np.array([wx, wy, wz]),
                    linear_acceleration=np.array([ax, ay, az])
                )
                self.imu_data.append(imu_data)

    def load_images_from_directory(self, img_dir: str,
                                  pattern: str = '*.png'):
        """
        从目录加载图像

        Args:
            img_dir: 图像目录路径
            pattern: 文件匹配模式
        """
        import glob

        image_files = sorted(glob.glob(os.path.join(img_dir, pattern)))

        for i, img_path in enumerate(image_files):
            # 使用索引作为时间戳（如果没有准确的时间戳）
            image_data = ImageData(
                timestamp=i * 0.05,  # 假设20Hz
                image_path=img_path
            )
            self.image_data.append(image_data)

    def get_synchronized_data(self) -> List[Tuple[ImageData, IMURawData]]:
        """获取同步的图像和IMU数据"""
        synchronized = []

        for img_data in self.image_data:
            if not self.imu_data:
                break

            closest_imu = min(self.imu_data,
                            key=lambda imu: abs(imu.timestamp - img_data.timestamp))

            if abs(closest_imu.timestamp - img_data.timestamp) < 0.1:
                synchronized.append((img_data, closest_imu))

        return synchronized

    def load_image(self, image_path: str) -> np.ndarray:
        """加载图像"""
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Failed to load image: {image_path}")
        return image


def create_demo_data(output_dir: str = './demo_imu_data',
                    num_frames: int = 100,
                    image_size: Tuple[int, int] = (640, 480)):
    """
    创建演示用的IMU数据集

    Args:
        output_dir: 输出目录
        num_frames: 帧数
        image_size: 图像尺寸 (width, height)
    """
    os.makedirs(output_dir, exist_ok=True)
    os.makedirs(os.path.join(output_dir, 'images'), exist_ok=True)

    # 创建IMU数据文件
    imu_data = []
    with open(os.path.join(output_dir, 'imu_data.csv'), 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'wx', 'wy', 'wz', 'ax', 'ay', 'az'])

        for i in range(num_frames * 10):  # IMU频率200Hz，图像20Hz
            timestamp = i * 0.005  # 5ms间隔

            # 模拟抖动
            wx = 0.1 * np.sin(2 * np.pi * 0.5 * timestamp) + 0.02 * np.random.randn()
            wy = 0.08 * np.cos(2 * np.pi * 0.3 * timestamp) + 0.02 * np.random.randn()
            wz = 0.05 * np.sin(2 * np.pi * 0.7 * timestamp) + 0.01 * np.random.randn()

            # 模拟重力加速度
            ax = 0.01 * np.random.randn()
            ay = 0.01 * np.random.randn()
            az = 9.81 + 0.01 * np.random.randn()

            writer.writerow([f'{timestamp:.6f}', f'{wx:.6f}', f'{wy:.6f}',
                           f'{wz:.6f}', f'{ax:.6f}', f'{ay:.6f}', f'{az:.6f}'])

            imu_data.append(IMURawData(
                timestamp=timestamp,
                angular_velocity=np.array([wx, wy, wz]),
                linear_acceleration=np.array([ax, ay, az])
            ))

    # 创建图像文件
    width, height = image_size
    with open(os.path.join(output_dir, 'image_data.csv'), 'w') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'image_name'])

        for i in range(num_frames):
            timestamp = i * 0.05  # 50ms间隔 (20Hz)
            image_name = f'frame_{i:04d}.png'

            # 生成测试图像
            image = np.zeros((height, width), dtype=np.uint8)
            for y in range(height):
                for x in range(width):
                    image[y, x] = int(255 * (x + y) / (width + height))

            # 添加一些特征
            cv2.circle(image, (width//2, height//2), 50, 255, 2)
            cv2.rectangle(image, (100, 100), (200, 200), 200, 2)

            # 保存图像
            image_path = os.path.join(output_dir, 'images', image_name)
            cv2.imwrite(image_path, image)

            writer.writerow([f'{timestamp:.6f}', image_name])

    print(f"演示数据集已创建到: {output_dir}")
    print(f"  - IMU数据点: {len(imu_data)}")
    print(f"  - 图像帧数: {num_frames}")

    return output_dir


# 使用示例
if __name__ == "__main__":
    # 创建演示数据
    demo_dir = create_demo_data()

    # 使用通用加载器加载演示数据
    loader = GenericIMULoader()
    loader.load_imu_from_csv(os.path.join(demo_dir, 'imu_data.csv'))
    loader.load_images_from_directory(os.path.join(demo_dir, 'images'))

    # 获取同步数据
    synchronized = loader.get_synchronized_data()
    print(f"\n同步数据对数: {len(synchronized)}")

    # 测试加载图像
    if synchronized:
        first_img, first_imu = synchronized[0]
        image = loader.load_image(first_img.image_path)
        print(f"第一帧图像尺寸: {image.shape}")
        print(f"对应IMU时间戳: {first_imu.timestamp:.6f}")
        print(f"对应IMU角速度: {first_imu.angular_velocity}")