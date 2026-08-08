"""
IMU视频稳定器 - 从IMU原始四元数数据到warp映射的完整实现
包含：IMU数据处理、姿态转换、平滑滤波、Δq补偿计算、射线旋转和warp映射生成
"""

import numpy as np
import cv2
from typing import List, Tuple, Optional
from dataclasses import dataclass
from collections import deque


@dataclass
class IMUData:
    """IMU数据结构"""
    timestamp: float
    quaternion: np.ndarray  # 四元数 [w, x, y, z]
    angular_velocity: np.ndarray  # 角速度 [wx, wy, wz] (rad/s)


@dataclass
class CameraPose:
    """相机姿态数据结构"""
    quaternion: np.ndarray  # 四元数表示的旋转 [w, x, y, z]
    rotation_matrix: np.ndarray  # 旋转矩阵 3x3
    euler_angles: np.ndarray  # 欧拉角 [roll, pitch, yaw] (rad)


class QuaternionOperations:
    """四元数操作工具类"""

    @staticmethod
    def normalize(q: np.ndarray) -> np.ndarray:
        """归一化四元数"""
        norm = np.linalg.norm(q)
        if norm < 1e-6:
            return np.array([1.0, 0.0, 0.0, 0.0])
        return q / norm

    @staticmethod
    def conjugate(q: np.ndarray) -> np.ndarray:
        """四元数共轭"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    @staticmethod
    def multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    @staticmethod
    def to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵"""
        q = QuaternionOperations.normalize(q)
        w, x, y, z = q

        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])

        return R

    @staticmethod
    def to_euler_angles(q: np.ndarray) -> np.ndarray:
        """四元数转欧拉角 (roll, pitch, yaw)"""
        q = QuaternionOperations.normalize(q)
        w, x, y, z = q

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])

    @staticmethod
    def from_angular_velocity(omega: np.ndarray, dt: float, prev_q: Optional[np.ndarray] = None) -> np.ndarray:
        """从角速度积分得到四元数"""
        if prev_q is None:
            prev_q = np.array([1.0, 0.0, 0.0, 0.0])

        # 角速度对应的四元数导数
        omega_norm = np.linalg.norm(omega)
        if omega_norm < 1e-6:
            return prev_q

        # 使用Rodrigues公式进行四元数积分
        half_omega = 0.5 * omega
        omega_q = np.array([0, half_omega[0], half_omega[1], half_omega[2]])

        # 一阶欧拉积分
        dq = QuaternionOperations.multiply(omega_q, prev_q)
        new_q = prev_q + dq * dt

        return QuaternionOperations.normalize(new_q)


class IMUProcessor:
    """IMU数据处理器"""

    def __init__(self, imu_to_camera_rotation: Optional[np.ndarray] = None):
        """
        初始化IMU处理器

        Args:
            imu_to_camera_rotation: IMU坐标系到相机坐标系的旋转矩阵
        """
        if imu_to_camera_rotation is None:
            # 默认的IMU到相机坐标系的转换（根据实际设备调整）
            self.imu_to_camera_rotation = np.eye(3)
        else:
            self.imu_to_camera_rotation = imu_to_camera_rotation

        self.prev_quaternion = np.array([1.0, 0.0, 0.0, 0.0])
        self.imu_poses = []
        self.camera_poses = []

    def process_imu_data(self, imu_data: IMUData) -> CameraPose:
        """
        处理单帧IMU数据，得到相机姿态

        Args:
            imu_data: 输入的IMU数据

        Returns:
            CameraPose: 相机姿态
        """
        # 1. 从角速度积分得到IMU姿态
        imu_quat = QuaternionOperations.from_angular_velocity(
            imu_data.angular_velocity,
            0.01,  # 假设10ms采样间隔
            self.prev_quaternion
        )

        # 如果输入已经有四元数数据，使用输入的四元数
        if np.linalg.norm(imu_data.quaternion) > 0:
            imu_quat = QuaternionOperations.normalize(imu_data.quaternion)

        self.prev_quaternion = imu_quat

        # 2. 转换IMU坐标系到相机坐标系
        imu_rotation_matrix = QuaternionOperations.to_rotation_matrix(imu_quat)
        camera_rotation_matrix = self.imu_to_camera_rotation @ imu_rotation_matrix

        # 从旋转矩阵重建四元数
        camera_quat = self.rotation_matrix_to_quaternion(camera_rotation_matrix)

        # 3. 计算欧拉角
        euler_angles = QuaternionOperations.to_euler_angles(camera_quat)

        # 创建相机姿态对象
        camera_pose = CameraPose(
            quaternion=camera_quat,
            rotation_matrix=camera_rotation_matrix,
            euler_angles=euler_angles
        )

        # 存储数据
        self.imu_poses.append(imu_quat)
        self.camera_poses.append(camera_pose)

        return camera_pose

    def rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """旋转矩阵转四元数"""
        trace = np.trace(R)

        if trace > 0:
            S = np.sqrt(trace + 1.0) * 2
            w = 0.25 * S
            x = (R[2, 1] - R[1, 2]) / S
            y = (R[0, 2] - R[2, 0]) / S
            z = (R[1, 0] - R[0, 1]) / S
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / S
            x = 0.25 * S
            y = (R[0, 1] + R[1, 0]) / S
            z = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / S
            x = (R[0, 1] + R[1, 0]) / S
            y = 0.25 * S
            z = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / S
            x = (R[0, 2] + R[2, 0]) / S
            y = (R[1, 2] + R[2, 1]) / S
            z = 0.25 * S

        return QuaternionOperations.normalize(np.array([w, x, y, z]))


class PoseSmoother:
    """姿态平滑滤波器"""

    def __init__(self, window_size: int = 5, smoothing_type: str = 'moving_average'):
        """
        初始化平滑滤波器

        Args:
            window_size: 平滑窗口大小
            smoothing_type: 平滑类型 ('moving_average', 'gaussian', 'exponential')
        """
        self.window_size = window_size
        self.smoothing_type = smoothing_type
        self.quaternion_buffer = deque(maxlen=window_size)

        # 高斯滤波器权重
        if smoothing_type == 'gaussian':
            x = np.linspace(-2, 2, window_size)
            self.gaussian_weights = np.exp(-0.5 * x**2)
            self.gaussian_weights /= np.sum(self.gaussian_weights)
        elif smoothing_type == 'exponential':
            self.alpha = 0.3  # 指数平滑系数
            self.prev_quaternion = None

    def smooth(self, current_quaternion: np.ndarray) -> np.ndarray:
        """
        对当前四元数进行平滑滤波

        Args:
            current_quaternion: 当前帧的四元数

        Returns:
            平滑后的四元数
        """
        current_quaternion = QuaternionOperations.normalize(current_quaternion)

        if self.smoothing_type == 'moving_average':
            return self.moving_average_smooth(current_quaternion)
        elif self.smoothing_type == 'gaussian':
            return self.gaussian_smooth(current_quaternion)
        elif self.smoothing_type == 'exponential':
            return self.exponential_smooth(current_quaternion)
        else:
            return current_quaternion

    def moving_average_smooth(self, current_quaternion: np.ndarray) -> np.ndarray:
        """移动平均平滑"""
        self.quaternion_buffer.append(current_quaternion)

        if len(self.quaternion_buffer) < 2:
            return current_quaternion

        # 四元数球面线性插值平均
        result = self.quaternion_buffer[0]
        for q in list(self.quaternion_buffer)[1:]:
            result = self.slerp(result, q, 0.5)

        return result

    def gaussian_smooth(self, current_quaternion: np.ndarray) -> np.ndarray:
        """高斯平滑"""
        self.quaternion_buffer.append(current_quaternion)

        if len(self.quaternion_buffer) < 2:
            return current_quaternion

        # 使用高斯权重进行加权平均
        quaternions = list(self.quaternion_buffer)
        weights = self.gaussian_weights[-len(quaternions):]
        weights = weights / np.sum(weights)  # 归一化

        # 加权球面插值
        result = quaternions[0]
        for q, w in zip(quaternions[1:], weights[1:]):
            result = self.slerp(result, q, w)

        return result

    def exponential_smooth(self, current_quaternion: np.ndarray) -> np.ndarray:
        """指数平滑"""
        if self.prev_quaternion is None:
            self.prev_quaternion = current_quaternion
            return current_quaternion

        # 指数加权移动平均
        smoothed = self.slerp(self.prev_quaternion, current_quaternion, self.alpha)
        self.prev_quaternion = smoothed

        return smoothed

    @staticmethod
    def slerp(q1: np.ndarray, q2: np.ndarray, t: float) -> np.ndarray:
        """球面线性插值"""
        q1 = QuaternionOperations.normalize(q1)
        q2 = QuaternionOperations.normalize(q2)

        # 计算点积
        dot = np.dot(q1, q2)

        # 如果点积为负，反转一个四元数以确保最短路径
        if dot < 0:
            q2 = -q2
            dot = -dot

        # 如果四元数非常接近，使用线性插值
        if dot > 0.9995:
            result = q1 + t * (q2 - q1)
            return QuaternionOperations.normalize(result)

        # 计算球面插值
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)

        theta = theta_0 * t
        sin_theta = np.sin(theta)

        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0

        return s0 * q1 + s1 * q2


class CompensationCalculator:
    """补偿Δq计算器"""

    def calculate_delta_q(self, current_pose: CameraPose, target_pose: CameraPose) -> np.ndarray:
        """
        计算当前姿态到目标姿态的补偿四元数

        Args:
            current_pose: 当前相机姿态
            target_pose: 目标相机姿态（平滑后的姿态）

        Returns:
            补偿四元数Δq
        """
        # Δq = q_target * q_current^(-1)
        current_q_conj = QuaternionOperations.conjugate(current_pose.quaternion)
        delta_q = QuaternionOperations.multiply(target_pose.quaternion, current_q_conj)

        return QuaternionOperations.normalize(delta_q)

    def calculate_compensation_transform(self, delta_q: np.ndarray, camera_matrix: np.ndarray) -> np.ndarray:
        """
        将补偿四元数转换为图像变换矩阵

        Args:
            delta_q: 补偿四元数
            camera_matrix: 相机内参矩阵

        Returns:
            3x3变换矩阵
        """
        # 将四元数转换为旋转矩阵
        rotation_matrix = QuaternionOperations.to_rotation_matrix(delta_q)

        # 转换为图像坐标系的变换（2D）
        # 假设相机主要绕光心旋转，这里简化为2D旋转变换
        theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

        # 构建仿射变换矩阵
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)

        transform_matrix = np.array([
            [cos_theta, -sin_theta, 0],
            [sin_theta, cos_theta, 0],
            [0, 0, 1]
        ])

        return transform_matrix


class WarpMapGenerator:
    """Warp映射生成器"""

    def __init__(self, width: int, height: int, camera_matrix: Optional[np.ndarray] = None):
        """
        初始化Warp映射生成器

        Args:
            width: 图像宽度
            height: 图像高度
            camera_matrix: 相机内参矩阵，如果为None则使用默认值
        """
        self.width = width
        self.height = height

        if camera_matrix is None:
            # 默认相机内参
            fx = fy = max(width, height) * 0.8
            cx, cy = width / 2, height / 2
            self.camera_matrix = np.array([
                [fx, 0, cx],
                [0, fy, cy],
                [0, 0, 1]
            ])
        else:
            self.camera_matrix = camera_matrix

        self.inv_camera_matrix = np.linalg.inv(self.camera_matrix)

    def generate_ray_rotations(self, delta_q: np.ndarray) -> np.ndarray:
        """
        为每个像素生成射线旋转变换

        Args:
            delta_q: 补偿四元数

        Returns:
            变换后的像素坐标网格
        """
        # 1. 生成像素坐标网格
        x_coords, y_coords = np.meshgrid(
            np.arange(self.width),
            np.arange(self.height)
        )

        # 2. 将像素坐标转换为归一化相机坐标
        # [u, v, 1]^T = K^-1 * [x, y, 1]^T
        homogeneous_coords = np.stack([
            x_coords.flatten(),
            y_coords.flatten(),
            np.ones(x_coords.size)
        ], axis=1)

        normalized_coords = homogeneous_coords @ self.inv_camera_matrix.T

        # 3. 应用旋转变换（射线旋转）
        rotation_matrix = QuaternionOperations.to_rotation_matrix(delta_q)
        rotated_coords = normalized_coords @ rotation_matrix.T

        # 4. 转换回像素坐标
        transformed_coords = rotated_coords @ self.camera_matrix.T

        # 5. 提取x, y坐标并重塑为图像形状
        map_x = (transformed_coords[:, 0] / transformed_coords[:, 2]).reshape(self.height, self.width).astype(np.float32)
        map_y = (transformed_coords[:, 1] / transformed_coords[:, 2]).reshape(self.height, self.width).astype(np.float32)

        return map_x, map_y

    def generate_warp_map(self, delta_q: np.ndarray, interpolation: str = 'linear') -> Tuple[np.ndarray, np.ndarray]:
        """
        生成完整的warp映射

        Args:
            delta_q: 补偿四元数
            interpolation: 插值类型 ('linear', 'cubic')

        Returns:
            (map_x, map_y): 重映射坐标
        """
        map_x, map_y = self.generate_ray_rotations(delta_q)

        return map_x, map_y

    def apply_warp(self, image: np.ndarray, delta_q: np.ndarray,
                   interpolation: str = 'linear', border_mode: int = cv2.BORDER_REFLECT) -> np.ndarray:
        """
        应用warp变换到图像

        Args:
            image: 输入图像
            delta_q: 补偿四元数
            interpolation: 插值方法
            border_mode: 边界处理模式

        Returns:
            变换后的图像
        """
        map_x, map_y = self.generate_warp_map(delta_q, interpolation)

        if interpolation == 'linear':
            interp_method = cv2.INTER_LINEAR
        elif interpolation == 'cubic':
            interp_method = cv2.INTER_CUBIC
        else:
            interp_method = cv2.INTER_LINEAR

        warped_image = cv2.remap(image, map_x, map_y, interp_method, borderMode=border_mode)

        return warped_image


class IMUVideoStabilizer:
    """IMU视频稳定器主类"""

    def __init__(self, width: int, height: int,
                 smoothing_window: int = 5,
                 smoothing_type: str = 'moving_average',
                 camera_matrix: Optional[np.ndarray] = None,
                 imu_to_camera_rotation: Optional[np.ndarray] = None):
        """
        初始化IMU视频稳定器

        Args:
            width: 图像宽度
            height: 图像高度
            smoothing_window: 平滑窗口大小
            smoothing_type: 平滑类型
            camera_matrix: 相机内参矩阵
            imu_to_camera_rotation: IMU到相机坐标系的旋转
        """
        self.width = width
        self.height = height

        # 初始化各个模块
        self.imu_processor = IMUProcessor(imu_to_camera_rotation)
        self.pose_smoother = PoseSmoother(smoothing_window, smoothing_type)
        self.compensation_calculator = CompensationCalculator()
        self.warp_generator = WarpMapGenerator(width, height, camera_matrix)

        # 存储处理结果
        self.processed_frames = []

    def process_frame(self, image: np.ndarray, imu_data: IMUData) -> np.ndarray:
        """
        处理单帧图像和IMU数据

        Args:
            image: 输入图像
            imu_data: 对应的IMU数据

        Returns:
            稳定后的图像
        """
        # 1. 处理IMU数据，得到相机姿态
        current_pose = self.imu_processor.process_imu_data(imu_data)

        # 2. 对姿态进行平滑滤波
        smoothed_quaternion = self.pose_smoother.smooth(current_pose.quaternion)
        smoothed_pose = CameraPose(
            quaternion=smoothed_quaternion,
            rotation_matrix=QuaternionOperations.to_rotation_matrix(smoothed_quaternion),
            euler_angles=QuaternionOperations.to_euler_angles(smoothed_quaternion)
        )

        # 3. 计算补偿Δq
        delta_q = self.compensation_calculator.calculate_delta_q(current_pose, smoothed_pose)

        # 4. 生成并应用warp映射
        stabilized_image = self.warp_generator.apply_warp(image, delta_q)

        # 存储处理结果
        frame_result = {
            'original_image': image.copy(),
            'stabilized_image': stabilized_image,
            'current_pose': current_pose,
            'smoothed_pose': smoothed_pose,
            'delta_q': delta_q,
            'imu_data': imu_data
        }
        self.processed_frames.append(frame_result)

        return stabilized_image

    def get_warp_map(self, imu_data: IMUData) -> Tuple[np.ndarray, np.ndarray]:
        """
        获取当前帧的warp映射（用于可视化）

        Args:
            imu_data: IMU数据

        Returns:
            (map_x, map_y): 重映射坐标
        """
        # 处理IMU数据
        current_pose = self.imu_processor.process_imu_data(imu_data)
        smoothed_quaternion = self.pose_smoother.smooth(current_pose.quaternion)
        smoothed_pose = CameraPose(
            quaternion=smoothed_quaternion,
            rotation_matrix=QuaternionOperations.to_rotation_matrix(smoothed_quaternion),
            euler_angles=QuaternionOperations.to_euler_angles(smoothed_quaternion)
        )

        # 计算补偿和warp映射
        delta_q = self.compensation_calculator.calculate_delta_q(current_pose, smoothed_pose)
        map_x, map_y = self.warp_generator.generate_warp_map(delta_q)

        return map_x, map_y


def visualize_mesh(image: np.ndarray, map_x: np.ndarray, map_y: np.ndarray,
                  grid_spacing: int = 20, line_color: Tuple[int, int, int] = (0, 255, 0),
                  line_thickness: int = 1) -> np.ndarray:
    """
    在图像上可视化warp mesh网格

    Args:
        image: 输入图像
        map_x: X方向重映射坐标
        map_y: Y方向重映射坐标
        grid_spacing: 网格间距
        line_color: 线条颜色 (B, G, R)
        line_thickness: 线条粗细

    Returns:
        带有mesh可视化的图像
    """
    vis_image = image.copy()
    height, width = image.shape[:2]

    # 生成网格点
    for y in range(0, height, grid_spacing):
        for x in range(0, width, grid_spacing):
            if y < map_y.shape[0] and x < map_x.shape[1]:
                # 获取变换后的坐标
                new_x = int(map_x[y, x])
                new_y = int(map_y[y, x])

                # 绘制从原始点到变换点的线
                if 0 <= new_x < width and 0 <= new_y < height:
                    cv2.line(vis_image, (x, y), (new_x, new_y), line_color, line_thickness)
                    cv2.circle(vis_image, (new_x, new_y), 2, line_color, -1)

    return vis_image


def generate_sample_imu_data(num_frames: int = 100, dt: float = 0.01) -> List[IMUData]:
    """
    生成示例IMU数据（用于测试）

    Args:
        num_frames: 帧数
        dt: 时间间隔

    Returns:
        IMU数据列表
    """
    imu_data_list = []

    # 模拟相机抖动：添加一些正弦波动的角速度
    for i in range(num_frames):
        timestamp = i * dt

        # 模拟抖动角速度（rad/s）
        wx = 0.1 * np.sin(2 * np.pi * 0.5 * timestamp) + 0.02 * np.random.randn()
        wy = 0.08 * np.cos(2 * np.pi * 0.3 * timestamp) + 0.02 * np.random.randn()
        wz = 0.05 * np.sin(2 * np.pi * 0.7 * timestamp) + 0.01 * np.random.randn()

        angular_velocity = np.array([wx, wy, wz])

        # 初始化四元数为单位四元数（实际应用中从传感器获取）
        quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        imu_data = IMUData(
            timestamp=timestamp,
            quaternion=quaternion,
            angular_velocity=angular_velocity
        )

        imu_data_list.append(imu_data)

    return imu_data_list


def main():
    """主函数 - 演示完整流程"""
    print("IMU视频稳定器 - 从IMU数据到warp映射演示")
    print("=" * 50)

    # 参数设置
    width, height = 640, 480
    num_frames = 100

    print(f"图像尺寸: {width}x{height}")
    print(f"处理帧数: {num_frames}")

    # 初始化稳定器
    stabilizer = IMUVideoStabilizer(
        width=width,
        height=height,
        smoothing_window=7,
        smoothing_type='moving_average'
    )

    # 生成示例数据
    print("生成示例IMU数据...")
    imu_data_list = generate_sample_imu_data(num_frames)

    # 创建示例图像（渐变背景）
    print("创建示例图像...")
    sample_image = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            sample_image[y, x] = [
                int(255 * x / width),      # R
                int(255 * y / height),     # G
                int(255 * (x + y) / (width + height))  # B
            ]

    # 添加一些特征点以便观察稳定效果
    cv2.circle(sample_image, (width//2, height//2), 50, (255, 255, 255), 2)
    cv2.rectangle(sample_image, (100, 100), (200, 200), (0, 255, 0), 2)

    # 处理每一帧
    print("开始处理帧...")
    for i, imu_data in enumerate(imu_data_list):
        # 处理帧
        stabilized_image = stabilizer.process_frame(sample_image, imu_data)

        # 每10帧显示一次进度
        if (i + 1) % 10 == 0:
            print(f"已处理 {i + 1}/{num_frames} 帧")

            # 获取warp映射用于可视化
            map_x, map_y = stabilizer.get_warp_map(imu_data)

            # 可视化mesh
            mesh_vis = visualize_mesh(stabilized_image, map_x, map_y)

            # 显示结果
            cv2.imshow('Stabilized Image with Mesh', mesh_vis)

            # 按ESC退出，其他键继续
            if cv2.waitKey(100) & 0xFF == 27:
                break

    cv2.destroyAllWindows()
    print("处理完成!")

    # 输出统计信息
    print(f"\n统计信息:")
    print(f"总处理帧数: {len(stabilizer.processed_frames)}")

    if len(stabilizer.processed_frames) > 0:
        # 计算平均补偿量
        delta_q_norms = [np.linalg.norm(frame['delta_q']) for frame in stabilizer.processed_frames]
        print(f"平均补偿量: {np.mean(delta_q_norms):.6f}")
        print(f"最大补偿量: {np.max(delta_q_norms):.6f}")
        print(f"最小补偿量: {np.min(delta_q_norms):.6f}")

        # 姿态变化统计
        euler_changes = []
        for i in range(1, len(stabilizer.processed_frames)):
            curr_euler = stabilizer.processed_frames[i]['current_pose'].euler_angles
            prev_euler = stabilizer.processed_frames[i-1]['current_pose'].euler_angles
            euler_changes.append(np.abs(curr_euler - prev_euler))

        if euler_changes:
            euler_changes = np.array(euler_changes)
            print(f"\n姿态变化统计 (rad):")
            print(f"Roll 变化 - 平均: {np.mean(euler_changes[:, 0]):.6f}, 最大: {np.max(euler_changes[:, 0]):.6f}")
            print(f"Pitch 变化 - 平均: {np.mean(euler_changes[:, 1]):.6f}, 最大: {np.max(euler_changes[:, 1]):.6f}")
            print(f"Yaw 变化 - 平均: {np.mean(euler_changes[:, 2]):.6f}, 最大: {np.max(euler_changes[:, 2]):.6f}")


if __name__ == "__main__":
    main()