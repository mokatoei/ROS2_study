# Imu消息官方定义
sensor_msgs/msg/Imu 消息用于承载惯性测量单元（IMU）传感器的数据。IMU 通常包含加速度计 (Accelerometer) 和陀螺仪 (Gyroscope)，有些还包含磁力计 (Magnetometer)。这个消息通过标准化的格式，提供了这些传感器测量的原始数据以及融合后的姿态估计。
通过 ros2 interface show sensor_msgs/msg/Imu 命令可以查看其定义：
``` bash
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.

std_msgs/Header header
        builtin_interfaces/Time stamp
                int32 sec
                uint32 nanosec
        string frame_id

geometry_msgs/Quaternion orientation # 姿态
        float64 x 0
        float64 y 0 
        float64 z 0
        float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity # 角速度
        float64 x
        float64 y
        float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration # 线速度
        float64 x
        float64 y
        float64 z
float64[9] linear_acceleration_covariance # Row major x, y z
```

## std_msgs/Header header
- 作用： 这是所有 ROS 2 时间戳数据类型的标准元数据头。它包含了图像被捕获的时间和所属的坐标系信息。
- 内部字段及用途：
    - builtin_interfaces/Time stamp: 时间戳。表示 IMU 数据被采集的精确时间。基于此可以对不同传感器的数据（如 IMU 和里程计）进行同步，以及可以在时间线上回放数据。
        - int32 sec：
            表示秒（seconds）部分。这是一个带符号的 32 位整数，所以可以表示从某个参考点（通常是 Unix Epoch，即 1970 年 1 月 1 日 00:00:00 UTC，或者系统启动时间）开始的秒数。它能表示正负的秒数，所以也可以处理过去的某个时间点。

        - uint32 nanosec：
            表示纳秒（nanoseconds）部分。这是一个无符号的 32 位整数。
            它的值范围是 [0, 1,000,000,000)，即从 0 到 999,999,999。
            这个字段用于表示秒以下的更精确的时间分辨率。它永远不会达到 10 亿纳秒，因为 10 亿纳秒就等于 1 秒，届时 sec 字段会加 1，nanosec 字段会归零。

    - string frame_id: 坐标系 ID。表示 IMU 传感器所在的坐标系（例如，"imu_link"、"base_link"）。这对于在机器人模型中进行坐标变换（TF）和传感器融合是必需的。详细介绍可看机器人坐标简介。

## geometry_msgs/Quaternion orientation
- 作用： 
    表示 IMU 估计的姿态（Orientation / Attitude）。姿态通常是通过融合加速度计、陀螺仪和磁力计（如果存在）的数据得到的。

- 类型： 
    geometry_msgs/Quaternion。这是一个四元数，由四个 float64 值 x, y, z, w 组成。

- 用途： 
    四元数是一种表示旋转的数学方式，相比欧拉角（roll, pitch, yaw）没有万向节锁（Gimbal Lock）问题，更适合进行姿态估计和旋转运算。它描述了传感器坐标系相对于某个参考坐标系（通常是全局坐标系或初始校准时的姿态）的旋转。

- 单位： 
    无单位。

## float64[9] orientation_covariance
- 作用： 
    表示 orientation 姿态估计的协方差矩阵。协方差矩阵用于量化姿态估计的不确定性或误差。

- 类型： 
    一个包含 9 个 float64 元素的数组，代表一个 3x3 的矩阵（按行优先存储）。

- 用途：
    对于状态估计算法（如扩展卡尔曼滤波器 EKF、无迹卡尔曼滤波器 UKF），协方差信息至关重要。它告诉算法姿态估计的“可靠性”有多高，以及不同轴向上的误差是如何相互关联的。
    如果协方差未知，应将所有 9 个元素设置为 0。
    如果某个轴向的协方差“无穷大”（即测量不可靠），可以将其对角线元素设置为 -1。

## geometry_msgs/Vector3 angular_velocity
- 作用： 
    表示 IMU 测量的角速度，即传感器绕其自身 X、Y、Z 轴的旋转速率。这通常由陀螺仪直接测量。

- 类型： 
    geometry_msgs/Vector3。由三个 float64 值 x, y, z 组成。

- 用途： 
    用于估计物体的旋转动态。

- 单位： 
    弧度/秒 (radians/second 或 rad/s)。不是度/秒！


## float64[9] angular_velocity_covariance
- 作用：
    表示 angular_velocity 角速度测量的协方差矩阵，量化角速度测量的不确定性。

- 类型： 
    同 orientation_covariance，9 个 float64 元素的数组。

- 用途： 
    提供角速度测量的误差信息，用于状态估计算法。

## geometry_msgs/Vector3 linear_acceleration
- 作用： 
    表示 IMU 测量的线加速度。这通常由加速度计直接测量，包含了物体自身的运动加速度和重力加速度。

- 类型： 
    geometry_msgs/Vector3。由三个 float64 值 x, y, z 组成。

- 用途： 
    用于估计物体的线性运动动态，并可以从中分离出重力分量来帮助姿态估计。

- 单位： 
    米/秒² (meters/second² 或 m/s²)。不是 G 值！ 如果你的 IMU 输出是 G 值，驱动程序必须将其转换为 m/s²（1G ≈ 9.80665 m/s²）。

## float64[9] linear_acceleration_covariance
- 作用： 
    表示 linear_acceleration 线加速度测量的协方差矩阵，量化线加速度测量的不确定性。

- 类型： 
    同 orientation_covariance 和 angular_velocity_covariance，9 个 float64 元素的数组。

- 用途： 
    提供线加速度测量的误差信息，用于状态估计算法。
