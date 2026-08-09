"""
生成随机合成的IMU四元数数据
模拟真实的相机运动和抖动
"""

import numpy as np
import os
import json
from typing import List, Tuple


class SyntheticIMUGenerator:
    
    """合成IMU数据生成器"""

    def __init__(self, duration: float = 10.0, imu_frequency: float = 200.0, camera_frequency: float = 20.0):
        """
        初始化合成数据生成器

        Args:
            duration: 数据持续时间（秒）
            imu_frequency: IMU采样频率（Hz）
            camera_frequency: 相机采样频率（Hz）
        """
        self.duration = duration
        self.imu_frequency = imu_frequency
        self.camera_frequency = camera_frequency

        self.imu_timestamps = np.arange(0, duration, 1.0 / imu_frequency)
        self.camera_timestamps = np.arange(0, duration, 1.0 / camera_frequency)

        self.imu_data = []
        self.camera_data = []

    def generate_camera_motion(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成相机主运动轨迹（模拟有意运动）

        Returns:
            (roll, pitch, yaw) 随时间变化的角度序列
        """
        t = self.imu_timestamps

        # 模拟相机的有意运动：平滑的正弦波运动
        # Roll: 缓慢的左右摇摆
        roll = 0.3 * np.sin(2 * np.pi * 0.2 * t) + 0.1 * np.sin(2 * np.pi * 0.5 * t)

        # Pitch: 缓慢的上下俯仰
        pitch = 0.2 * np.sin(2 * np.pi * 0.15 * t + np.pi/4) + 0.05 * np.cos(2 * np.pi * 0.8 * t)

        # Yaw: 缓慢的左右转动
        yaw = 0.4 * np.sin(2 * np.pi * 0.1 * t) + 0.1 * np.sin(2 * np.pi * 0.3 * t + np.pi/2)

        return roll, pitch, yaw

    def generate_camera_shake(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        生成相机抖动（模拟手抖）

        Returns:
            (roll, pitch, yaw) 随机抖动角度序列
        """
        n_samples = len(self.imu_timestamps)

        # 生成有色噪声（模拟真实的手抖特性）
        # 使用多个频率的随机正弦波叠加

        roll_shake = np.zeros(n_samples)
        pitch_shake = np.zeros(n_samples)
        yaw_shake = np.zeros(n_samples)

        # 叠加不同频率的随机分量
        frequencies = [1.0, 2.0, 3.0, 5.0, 8.0, 12.0]

        for freq in frequencies:
            amplitude = 0.02 / freq  # 高频振幅较小
            phase_roll = np.random.rand() * 2 * np.pi
            phase_pitch = np.random.rand() * 2 * np.pi
            phase_yaw = np.random.rand() * 2 * np.pi

            t = self.imu_timestamps
            roll_shake += amplitude * np.sin(2 * np.pi * freq * t + phase_roll)
            pitch_shake += amplitude * np.sin(2 * np.pi * freq * t + phase_pitch)
            yaw_shake += amplitude * np.sin(2 * np.pi * freq * t + phase_yaw)

        # 添加高频随机噪声
        roll_shake += 0.005 * np.random.randn(n_samples)
        pitch_shake += 0.005 * np.random.randn(n_samples)
        yaw_shake += 0.005 * np.random.randn(n_samples)

        return roll_shake, pitch_shake, yaw_shake

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        """
        欧拉角转四元数

        Args:
            roll, pitch, yaw: 欧拉角（弧度）

        Returns:
            四元数 [w, x, y, z]
        """
        # 计算半角
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        # 计算四元数分量
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        quaternion = np.array([w, x, y, z])

        # 归一化
        norm = np.linalg.norm(quaternion)
        if norm > 0:
            quaternion = quaternion / norm

        return quaternion

    def quaternion_to_angular_velocity(self, q_current: np.ndarray, q_next: np.ndarray, dt: float) -> np.ndarray:
        """
        从两个四元数计算角速度

        Args:
            q_current: 当前四元数
            q_next: 下一个四元数
            dt: 时间间隔

        Returns:
            角速度向量 [wx, wy, wz] (rad/s)
        """
        # 计算四元数差值
        q_diff = self.quaternion_multiply(q_next, self.quaternion_conjugate(q_current))

        # 小角度近似: q ≈ [1, 0.5*ω*dt]
        # ω ≈ 2 * [q.x, q.y, q.z] / dt
        omega = 2.0 * q_diff[1:] / dt

        return omega

    def quaternion_multiply(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """四元数乘法"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        w = w1*w2 - x1*x2 - y1*y2 - z1*z2
        x = w1*x2 + x1*w2 + y1*z2 - z1*y2
        y = w1*y2 - x1*z2 + y1*w2 + z1*x2
        z = w1*z2 + x1*y2 - y1*x2 + z1*w2

        return np.array([w, x, y, z])

    def quaternion_conjugate(self, q: np.ndarray) -> np.ndarray:
        """四元数共轭"""
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def generate(self, shake_intensity: float = 1.0) -> Tuple[List[dict], List[dict]]:
        """
        生成完整的合成IMU数据

        Args:
            shake_intensity: 抖动强度系数

        Returns:
            (imu_data, camera_data): IMU数据和相机数据列表
        """
        print("开始生成合成IMU数据...")

        # 生成主运动和抖动
        roll_motion, pitch_motion, yaw_motion = self.generate_camera_motion()
        roll_shake, pitch_shake, yaw_shake = self.generate_camera_shake()

        # 组合运动（主运动 + 抖动）
        roll_total = roll_motion + shake_intensity * roll_shake
        pitch_total = pitch_motion + shake_intensity * pitch_shake
        yaw_total = yaw_motion + shake_intensity * yaw_shake

        # 生成四元数序列
        quaternions = []
        for i in range(len(self.imu_timestamps)):
            q = self.euler_to_quaternion(roll_total[i], pitch_total[i], yaw_total[i])
            quaternions.append(q)

        # 生成IMU数据
        dt = 1.0 / self.imu_frequency
        for i in range(len(self.imu_timestamps)):
            timestamp = self.imu_timestamps[i]

            # 计算角速度
            if i < len(quaternions) - 1:
                omega = self.quaternion_to_angular_velocity(quaternions[i], quaternions[i+1], dt)
            else:
                omega = np.array([0.0, 0.0, 0.0])  # 最后一个点

            # 模拟加速度（包含重力）
            # 简化模型：主要包含重力分量和运动加速度
            gravity = np.array([0.0, 0.0, 9.81])  # 重力加速度

            # 根据旋转姿态变换重力到IMU坐标系
            R = self.quaternion_to_rotation_matrix(quaternions[i])
            gravity_imu = R.T @ gravity  # 转换到IMU坐标系

            # 添加一些运动加速度噪声
            acceleration_noise = 0.1 * np.random.randn(3)
            acceleration = gravity_imu + acceleration_noise

            imu_entry = {
                'timestamp': timestamp,
                'quaternion': quaternions[i].tolist(),
                'angular_velocity': omega.tolist(),
                'linear_acceleration': acceleration.tolist()
            }
            self.imu_data.append(imu_entry)

        # 生成相机数据（在相机采样时刻）
        for cam_timestamp in self.camera_timestamps:
            # 找到最近的IMU数据
            closest_idx = np.argmin(np.abs(self.imu_timestamps - cam_timestamp))
            closest_imu = self.imu_data[closest_idx]

            camera_entry = {
                'timestamp': cam_timestamp,
                'quaternion': closest_imu['quaternion'],
                'frame_number': len(self.camera_data)
            }
            self.camera_data.append(camera_entry)

        print(f"生成完成:")
        print(f"  - IMU数据点: {len(self.imu_data)}")
        print(f"  - 相机帧数: {len(self.camera_data)}")
        print(f"  - 持续时间: {self.duration:.2f}秒")
        print(f"  - 抖动强度: {shake_intensity:.2f}")

        return self.imu_data, self.camera_data

    def quaternion_to_rotation_matrix(self, q: np.ndarray) -> np.ndarray:
        """四元数转旋转矩阵"""
        q = q / np.linalg.norm(q)  # 归一化
        w, x, y, z = q

        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])

        return R

    def save_to_file(self, output_dir: str = './synthetic_imu_data'):
        """
        保存生成的数据到文件

        Args:
            output_dir: 输出目录
        """
        os.makedirs(output_dir, exist_ok=True)

        # 保存IMU数据
        imu_file = os.path.join(output_dir, 'imu_data.json')
        with open(imu_file, 'w') as f:
            json.dump({
                'frequency': self.imu_frequency,
                'duration': self.duration,
                'data': self.imu_data
            }, f, indent=2)

        # 保存相机数据
        camera_file = os.path.join(output_dir, 'camera_data.json')
        with open(camera_file, 'w') as f:
            json.dump({
                'frequency': self.camera_frequency,
                'duration': self.duration,
                'data': self.camera_data
            }, f, indent=2)

        # 保存为CSV格式（便于其他工具使用）
        self._save_to_csv(output_dir)

        print(f"\n数据已保存到: {output_dir}")
        print(f"  - {imu_file}")
        print(f"  - {camera_file}")

    def _save_to_csv(self, output_dir: str):
        """保存为CSV格式"""
        import csv

        # IMU数据CSV
        imu_csv = os.path.join(output_dir, 'imu_data.csv')
        with open(imu_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'qw', 'qx', 'qy', 'qz',
                           'wx', 'wy', 'wz', 'ax', 'ay', 'az'])

            for entry in self.imu_data:
                row = [entry['timestamp']] + entry['quaternion'] + \
                      entry['angular_velocity'] + entry['linear_acceleration']
                writer.writerow(row)

        # 相机数据CSV
        camera_csv = os.path.join(output_dir, 'camera_data.csv')
        with open(camera_csv, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['timestamp', 'frame_number', 'qw', 'qx', 'qy', 'qz'])

            for entry in self.camera_data:
                row = [entry['timestamp'], entry['frame_number']] + entry['quaternion']
                writer.writerow(row)

    def visualize_data(self, output_dir: str = './synthetic_imu_data'):
        """
        可视化生成的数据（使用OpenCV）

        Args:
            output_dir: 输出目录
        """
        try:
            import cv2
        except ImportError:
            print("OpenCV未安装，跳过可视化")
            return

        os.makedirs(output_dir, exist_ok=True)

        # 提取数据
        timestamps = [entry['timestamp'] for entry in self.imu_data]
        quaternions = np.array([entry['quaternion'] for entry in self.imu_data])
        angular_velocities = np.array([entry['angular_velocity'] for entry in self.imu_data])

        # 计算欧拉角
        euler_angles = []
        for q in quaternions:
            euler = self.quaternion_to_euler(q)
            euler_angles.append(euler)
        euler_angles = np.array(euler_angles)

        # 创建可视化图像
        img_height = 600
        img_width = 1200
        visualization = np.ones((img_height, img_width, 3), dtype=np.uint8) * 255

        # 绘制欧拉角
        self._plot_data(visualization, timestamps, np.degrees(euler_angles[:, 0]),
                       (0, 0, img_width//3, 200), "Roll (degrees)", (255, 0, 0))
        self._plot_data(visualization, timestamps, np.degrees(euler_angles[:, 1]),
                       (img_width//3, 0, 2*img_width//3, 200), "Pitch (degrees)", (0, 255, 0))
        self._plot_data(visualization, timestamps, np.degrees(euler_angles[:, 2]),
                       (2*img_width//3, 0, img_width, 200), "Yaw (degrees)", (0, 0, 255))

        # 绘制角速度
        self._plot_data(visualization, timestamps, np.degrees(angular_velocities[:, 0]),
                       (0, 200, img_width//3, 400), "Angular Velocity X (deg/s)", (255, 0, 0))
        self._plot_data(visualization, timestamps, np.degrees(angular_velocities[:, 1]),
                       (img_width//3, 200, 2*img_width//3, 400), "Angular Velocity Y (deg/s)", (0, 255, 0))
        self._plot_data(visualization, timestamps, np.degrees(angular_velocities[:, 2]),
                       (2*img_width//3, 200, img_width, 400), "Angular Velocity Z (deg/s)", (0, 0, 255))

        # 绘制四元数分量
        self._plot_data(visualization, timestamps, quaternions[:, 0],
                       (0, 400, img_width//4, 600), "Quaternion W", (255, 0, 0))
        self._plot_data(visualization, timestamps, quaternions[:, 1],
                       (img_width//4, 400, img_width//2, 600), "Quaternion X", (0, 255, 0))
        self._plot_data(visualization, timestamps, quaternions[:, 2],
                       (img_width//2, 400, 3*img_width//4, 600), "Quaternion Y", (0, 0, 255))
        self._plot_data(visualization, timestamps, quaternions[:, 3],
                       (3*img_width//4, 400, img_width, 600), "Quaternion Z", (255, 255, 0))

        # 保存可视化图像
        plot_file = os.path.join(output_dir, 'imu_data_visualization.png')
        cv2.imwrite(plot_file, visualization)
        print(f"可视化图表已保存: {plot_file}")

    def _plot_data(self, image, x_data, y_data, region, title, color, offset=0):
        """在图像的指定区域绘制数据"""
        try:
            import cv2
        except ImportError:
            print("OpenCV未安装，跳过绘图")
            return

        x1, y1, x2, y2 = region
        width = x2 - x1
        height = y2 - y1

        # 绘制标题
        cv2.putText(image, title, (x1 + 10, y1 + 20 + offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

        # 归一化数据到绘图区域
        if len(x_data) > 0 and len(y_data) > 0:
            x_min, x_max = min(x_data), max(x_data)
            y_min, y_max = min(y_data), max(y_data)

            if x_max - x_min > 0 and y_max - y_min > 0:
                points = []
                for i in range(len(x_data)):
                    px = int(x1 + (x_data[i] - x_min) / (x_max - x_min) * (width - 20) + 10)
                    py = int(y2 - (y_data[i] - y_min) / (y_max - y_min) * (height - 40) - 10)
                    points.append((px, py))

                # 绘制数据线
                for i in range(len(points) - 1):
                    try:
                        cv2.line(image, points[i], points[i+1], color, 1)
                    except:
                        pass  # 如果cv2不可用，跳过绘图

    def quaternion_to_euler(self, q: np.ndarray) -> np.ndarray:
        """四元数转欧拉角"""
        q = q / np.linalg.norm(q)
        w, x, y, z = q

        # Roll
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])


def main():
    """主函数"""
    print("=" * 60)
    print("合成IMU数据生成器")
    print("=" * 60)

    # 参数设置
    duration = 10.0           # 持续时间（秒）
    imu_freq = 200.0          # IMU频率（Hz）
    camera_freq = 20.0        # 相机频率（Hz）
    shake_intensity = 1.5     # 抖动强度

    print(f"\n生成参数:")
    print(f"  持续时间: {duration}秒")
    print(f"  IMU频率: {imu_freq}Hz")
    print(f"  相机频率: {camera_freq}Hz")
    print(f"  抖动强度: {shake_intensity}")

    # 创建生成器
    generator = SyntheticIMUGenerator(duration, imu_freq, camera_freq)

    # 生成数据
    imu_data, camera_data = generator.generate(shake_intensity)

    # 保存数据
    generator.save_to_file('./synthetic_imu_data')

    # 可视化数据
    generator.visualize_data('./synthetic_imu_data')

    # 输出一些统计信息
    print(f"\n统计信息:")
    quaternions = np.array([entry['quaternion'] for entry in imu_data])
    angular_velocities = np.array([entry['angular_velocity'] for entry in imu_data])

    print(f"  四元数模长: {np.linalg.norm(quaternions, axis=1).mean():.6f} (应接近1.0)")
    print(f"  角速度RMS: {np.sqrt(np.mean(angular_velocities**2))}")
    print(f"  最大角速度: {np.max(np.abs(angular_velocities)):.4f} rad/s")

    print("\n" + "=" * 60)
    print("数据生成完成！")
    print("=" * 60)


if __name__ == "__main__":
    main()