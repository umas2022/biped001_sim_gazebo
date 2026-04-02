# 初学者指南

这份文档只做一件事：

帮助第一次接触这个项目的人，先建立整体地图，再知道应该先看什么、后看什么。

如果只记一句话：

Gazebo 负责仿真物理世界，ROS 2 控制器负责读状态、算命令，LQR 负责把平衡这件事统一算出来。

## 1. 项目最小心智模型

这个项目可以粗略理解成三层：

1. 仿真层
   Gazebo Sim 提供世界、刚体动力学、接触和传感器。
2. 桥接层
   `ros_gz_bridge` 把 Gazebo 数据转成 ROS 2 话题，也把 ROS 2 命令发回 Gazebo。
3. 控制层
   `bp001_control` 读取 `/imu`、`/joint_states`、`/cmd_vel`，输出左右轮命令。

## 2. 整体数据流

```mermaid
flowchart LR
    A[Gazebo World] --> B[Robot bp001]
    B --> C[IMU / Joint State]
    C --> D[ros_gz_bridge]
    D --> E[/imu]
    D --> F[/joint_states]
    G[/cmd_vel] --> H[balance_controller]
    E --> H
    F --> H
    H --> I[/model/bp001/joint/joint_lw/cmd_vel]
    H --> J[/model/bp001/joint/joint_rw/cmd_vel]
    I --> D
    J --> D
    D --> B
```

核心闭环就是：

- Gazebo 产生状态
- 控制器读取状态并计算输出
- 输出再回到 Gazebo 驱动机器人

## 3. `/cmd_vel` 是怎么接入平衡控制的

`/cmd_vel` 不是直接绕开 LQR 去“硬推轮子”。

当前控制结构更接近：

1. `linear.x` 先变成目标轮速
2. 速度外环把速度误差变成一个小的 `pitch_target`
3. LQR 对完整状态 `x` 做反馈
4. `angular.z` 再叠加成左右轮差速偏置

所以这个项目本质上仍然是：

“LQR 负责平衡，`/cmd_vel` 负责给平衡器一个温和的运动目标。”

## 4. 先知道这些术语就够了

- `pitch`
  车身俯仰角，可以粗略理解为前后倾了多少。
- `pitch_rate`
  俯仰角速度。
- `wheel_vel`
  左右轮平均角速度。
- `wheel_pos_error`
  当前轮位置相对参考位置的偏差。
- `pitch_target`
  为了跟踪速度而生成的小目标倾角。
- `u = -Kx`
  LQR 的状态反馈控制律。

## 5. 推荐阅读顺序

第一次看项目，建议按这个顺序：

1. 先看 [`readme.md`](../readme.md)
2. 再看 [`docs/minimal_lqr_demo.md`](./minimal_lqr_demo.md)
3. 再看 [`docs/implementation.md`](./implementation.md)
4. 最后再读控制器源码 [`balance_controller.py`](../src/bp001_control/bp001_control/balance_controller.py)

如果你更关心启动链路而不是控制原理，也可以先看 `implementation` 再回来看最小 LQR 文档。

## 6. 看源码时先抓主线

最值得先看的文件只有四个：

1. [`sim.launch.py`](../src/bp001_sim/launch/sim.launch.py)
   看启动了哪些节点。
2. [`bp001_sim.xacro`](../src/bp001_sim/urdf/bp001_sim.xacro)
   看 Gazebo 插件、IMU 和关节控制怎么接。
3. [`balance_controller.yaml`](../src/bp001_control/config/balance_controller.yaml)
   看控制器参数。
4. [`balance_controller.py`](../src/bp001_control/bp001_control/balance_controller.py)
   看状态构造、LQR 求解和控制循环。

## 7. 常见误解

- 发了 `/cmd_vel` 不代表一定达到那个速度。
  实际表现还受姿态、限幅、模型误差和当前稳定范围影响。
- 匀速运动不等于绝对零倾角。
  两轮平衡车在运动时通常会带一个很小的姿态偏差。
- 默认参数不是高速参数。
  这个版本定位是入门示例，当前建议先用 `0.05 ~ 0.10 m/s`。

## 8. 一句话收尾

先把系统闭环和阅读顺序看清，再进入实现细节；不要一开始就陷进所有参数和所有节点里。
