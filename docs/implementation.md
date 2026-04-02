# bp001 仿真实现说明

这份文档保留工程实现信息，不再重复初学者导览和 LQR 基础概念。

如果你还没建立整体地图，先看：

[`docs/beginner_guide.md`](./beginner_guide.md)

如果你想先理解 `A/B/Q/R/K` 和 `u = -Kx`，先看：

[`docs/minimal_lqr_demo.md`](./minimal_lqr_demo.md)

## 1. 项目目标

当前版本的目标很明确：

- 基于 ROS 2 Jazzy 和 Gazebo Sim 跑通完整闭环
- 提供一个可读、可复现的两轮平衡车 LQR 示例
- 在稳定平衡基础上支持保守的 `/cmd_vel` 速度接口

它是一个 HelloWorld 级工程样例，不以高速和高动态性能为目标。

## 2. 包结构

项目由 3 个包组成。

### `bp001_description`

机器人本体描述：

- 车体和轮子的 URDF/Xacro
- 网格资源
- Gazebo 资源导出

关键文件：

- [`bp001_description.xacro`](../src/bp001_description/urdf/bp001_description.xacro)

### `bp001_sim`

仿真集成：

- 世界文件
- Gazebo 插件扩展
- 启动 Gazebo、生成机器人、配置桥接

关键文件：

- [`empty.sdf`](../src/bp001_sim/worlds/empty.sdf)
- [`bp001_sim.xacro`](../src/bp001_sim/urdf/bp001_sim.xacro)
- [`sim.launch.py`](../src/bp001_sim/launch/sim.launch.py)

### `bp001_control`

控制逻辑：

- LQR 平衡控制器
- 默认参数
- 控制器启动入口

关键文件：

- [`balance_controller.py`](../src/bp001_control/bp001_control/balance_controller.py)
- [`balance_controller.yaml`](../src/bp001_control/config/balance_controller.yaml)

## 3. 启动链路

完整链路如下：

1. 运行 `ros2 launch bp001_sim sim.launch.py`
2. `gz sim` 启动世界
3. `robot_state_publisher` 发布 `robot_description`
4. `ros_gz_sim create` 在 Gazebo 中生成 `bp001`
5. `ros_gz_bridge` 建立 ROS 和 Gazebo 话题桥接
6. `bp001_control` 启动并进入控制循环

运行后，控制器主要读取：

- `/imu`
- `/joint_states`
- `/cmd_vel`

并发布：

- `/model/bp001/joint/joint_lw/cmd_vel`
- `/model/bp001/joint/joint_rw/cmd_vel`

## 4. 机器人与 Gazebo 配置

机器人主体包含：

- `base_link`
- `link_lw`
- `link_rw`
- `joint_lw`
- `joint_rw`

Gazebo 扩展中主要补充了：

- IMU 传感器
- 左右轮摩擦参数
- `JointStatePublisher`
- 左右轮 `JointController`

当前轮关节控制工作在速度目标模式，目的是让整个样例更稳、更容易理解。

## 5. 控制器结构

控制器的核心状态是：

`x = [wheel_pos_error, wheel_vel, pitch_error, pitch_rate]`

控制律是：

`u = -Kx`

其中：

- `wheel_pos_error`
  抑制位置漂移
- `wheel_vel`
  反映当前移动状态
- `pitch_error`
  反映姿态偏差
- `pitch_rate`
  反映姿态变化速度

这个 `u` 是平衡相关的轮速命令，之后还会叠加速度和转向相关处理。

## 6. 离散 LQR 模型

控制器内部使用工程化离散近似模型，而不是完整刚体解析推导。

模型参数主要包括：

- `lqr_g_over_l`
- `lqr_theta_damping`
- `lqr_vel_damping`
- `lqr_input_gain`
- `lqr_motor_gain`

代价函数参数主要包括：

- `q_pos`
- `q_vel`
- `q_pitch`
- `q_pitch_rate`
- `r_input`

调参直觉可以粗略记成：

- `q_pitch`、`q_pitch_rate` 大一些，更重视姿态稳定
- `q_pos`、`q_vel` 大一些，更重视抑制位置和速度漂移
- `r_input` 大一些，输出会更保守

## 7. `/cmd_vel` 接口是怎么叠上去的

速度接口并没有替代 LQR。

当前思路是：

1. `linear.x` 先生成轮速参考
2. 速度误差再生成小范围 `pitch_target`
3. LQR 根据完整状态继续稳定系统
4. `angular.z` 生成左右轮差速偏置

相关参数主要在：

- [`balance_controller.yaml`](../src/bp001_control/config/balance_controller.yaml)

重点包括：

- `wheel_radius`
- `wheel_track`
- `cmd_vel_timeout`
- `cmd_vel_linear_limit`
- `cmd_vel_angular_limit`
- `velocity_pitch_kp`
- `velocity_pitch_ki`
- `max_pitch_target`
- `wheel_speed_feedforward_gain`

## 8. 滤波与输出保护

为了让离散控制在仿真里更稳，控制器还做了两类工程保护。

状态滤波：

- `pitch_alpha`
- `pitch_rate_alpha`
- `wheel_vel_alpha`

输出保护：

- `command_alpha`
- `max_cmd_step`
- `max_wheel_speed`

这些不是控制理论核心，但对可用性很重要。

## 9. 复现步骤

### 环境

建议环境：

- Ubuntu 24.04
- ROS 2 Jazzy
- `ros_gz_sim`
- `ros_gz_bridge`
- `xacro`
- `numpy`
- `scipy`

### 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 启动

```bash
ros2 launch bp001_sim sim.launch.py
```

### 基础验证

启动后先检查：

- `/imu`
- `/joint_states`
- 左右轮 `cmd_vel` 话题是否存在

再用小速度命令验证：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.08}, angular: {z: 0.0}}" -r 10
```

建议先使用 `0.05 ~ 0.10 m/s`。

## 10. 常见问题

- 车辆闪烁、抽搐、像瞬移：
  通常是旧的 `gz sim` 或旧控制器实例残留。
- 发了大速度命令后不稳定：
  默认参数不是给高速工况准备的。
- 不发命令时还能轻微修正：
  这是平衡控制正常工作的一部分。

## 11. 收尾

这份文档回答的是“工程上它是怎么搭起来的”；控制基础和最小例子不要再到这里重复看。
