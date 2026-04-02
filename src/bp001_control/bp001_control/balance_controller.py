import math

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from scipy.linalg import solve_discrete_are
from sensor_msgs.msg import Imu
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


def quaternion_to_pitch(x: float, y: float, z: float, w: float) -> float:
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        return math.copysign(math.pi / 2.0, sinp)
    return math.asin(sinp)


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def solve_discrete_lqr_gain(
    dt: float,
    g_over_l: float,
    theta_damping: float,
    vel_damping: float,
    input_gain: float,
    motor_gain: float,
    q_pos: float,
    q_vel: float,
    q_pitch: float,
    q_pitch_rate: float,
    r_input: float,
) -> np.ndarray:
    a = np.array([
        [1.0, dt, 0.0, 0.0],
        [0.0, 1.0 - vel_damping * dt, 0.0, motor_gain * dt],
        [0.0, 0.0, 1.0, dt],
        [0.0, 0.0, g_over_l * dt, 1.0 - theta_damping * dt],
    ])
    b = np.array([
        [0.0],
        [motor_gain * dt],
        [0.0],
        [-input_gain * dt],
    ])
    q = np.diag([q_pos, q_vel, q_pitch, q_pitch_rate])
    r = np.array([[r_input]])
    p = solve_discrete_are(a, b, q, r)
    k = np.linalg.inv(b.T @ p @ b + r) @ (b.T @ p @ a)
    return k[0]


class BalanceController(Node):
    def __init__(self) -> None:
        super().__init__('balance_controller')

        self.declare_parameter('dt', 0.01)
        self.declare_parameter('lqr_g_over_l', 22.0)
        self.declare_parameter('lqr_theta_damping', 2.8)
        self.declare_parameter('lqr_vel_damping', 3.0)
        self.declare_parameter('lqr_input_gain', 28.0)
        self.declare_parameter('lqr_motor_gain', 10.0)
        self.declare_parameter('q_pos', 2.0)
        self.declare_parameter('q_vel', 0.8)
        self.declare_parameter('q_pitch', 80.0)
        self.declare_parameter('q_pitch_rate', 8.0)
        self.declare_parameter('r_input', 1.4)
        self.declare_parameter('max_wheel_speed', 1.2)
        self.declare_parameter('command_alpha', 0.06)
        self.declare_parameter('max_cmd_step', 0.08)
        self.declare_parameter('pitch_deadband', 0.03)
        self.declare_parameter('pitch_ref', 0.0)
        self.declare_parameter('fall_pitch_limit', 0.8)
        self.declare_parameter('pitch_alpha', 0.25)
        self.declare_parameter('pitch_rate_alpha', 0.20)
        self.declare_parameter('wheel_vel_alpha', 0.15)
        self.declare_parameter('wheel_radius', 0.1)
        self.declare_parameter('wheel_track', 0.34)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('cmd_vel_linear_limit', 0.25)
        self.declare_parameter('cmd_vel_angular_limit', 1.2)
        self.declare_parameter('cmd_vel_alpha', 0.1)
        self.declare_parameter('wheel_pos_ref_alpha', 0.08)
        self.declare_parameter('idle_wheel_pos_ref_alpha', 0.03)
        self.declare_parameter('cmd_vel_active_threshold', 0.01)
        self.declare_parameter('velocity_pitch_kp', 0.12)
        self.declare_parameter('velocity_pitch_ki', 0.35)
        self.declare_parameter('velocity_integral_limit', 0.08)
        self.declare_parameter('max_pitch_target', 0.10)
        self.declare_parameter('wheel_speed_feedforward_gain', 0.35)
        self.declare_parameter('moving_position_hold_scale', 0.08)
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('left_cmd_topic', '/model/bp001/joint/joint_lw/cmd_vel')
        self.declare_parameter('right_cmd_topic', '/model/bp001/joint/joint_rw/cmd_vel')

        self.dt = float(self.get_parameter('dt').value)
        self.lqr_gain = solve_discrete_lqr_gain(
            dt=self.dt,
            g_over_l=float(self.get_parameter('lqr_g_over_l').value),
            theta_damping=float(self.get_parameter('lqr_theta_damping').value),
            vel_damping=float(self.get_parameter('lqr_vel_damping').value),
            input_gain=float(self.get_parameter('lqr_input_gain').value),
            motor_gain=float(self.get_parameter('lqr_motor_gain').value),
            q_pos=float(self.get_parameter('q_pos').value),
            q_vel=float(self.get_parameter('q_vel').value),
            q_pitch=float(self.get_parameter('q_pitch').value),
            q_pitch_rate=float(self.get_parameter('q_pitch_rate').value),
            r_input=float(self.get_parameter('r_input').value),
        )
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed').value)
        self.command_alpha = float(self.get_parameter('command_alpha').value)
        self.max_cmd_step = float(self.get_parameter('max_cmd_step').value)
        self.pitch_deadband = float(self.get_parameter('pitch_deadband').value)
        self.pitch_ref = float(self.get_parameter('pitch_ref').value)
        self.fall_pitch_limit = float(self.get_parameter('fall_pitch_limit').value)
        self.pitch_alpha = float(self.get_parameter('pitch_alpha').value)
        self.pitch_rate_alpha = float(self.get_parameter('pitch_rate_alpha').value)
        self.wheel_vel_alpha = float(self.get_parameter('wheel_vel_alpha').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_track = float(self.get_parameter('wheel_track').value)
        self.cmd_vel_timeout = float(self.get_parameter('cmd_vel_timeout').value)
        self.cmd_vel_linear_limit = float(self.get_parameter('cmd_vel_linear_limit').value)
        self.cmd_vel_angular_limit = float(self.get_parameter('cmd_vel_angular_limit').value)
        self.cmd_vel_alpha = float(self.get_parameter('cmd_vel_alpha').value)
        self.wheel_pos_ref_alpha = float(self.get_parameter('wheel_pos_ref_alpha').value)
        self.idle_wheel_pos_ref_alpha = float(
            self.get_parameter('idle_wheel_pos_ref_alpha').value
        )
        self.cmd_vel_active_threshold = float(self.get_parameter('cmd_vel_active_threshold').value)
        self.velocity_pitch_kp = float(self.get_parameter('velocity_pitch_kp').value)
        self.velocity_pitch_ki = float(self.get_parameter('velocity_pitch_ki').value)
        self.velocity_integral_limit = float(
            self.get_parameter('velocity_integral_limit').value
        )
        self.max_pitch_target = float(self.get_parameter('max_pitch_target').value)
        self.wheel_speed_feedforward_gain = float(
            self.get_parameter('wheel_speed_feedforward_gain').value
        )
        self.moving_position_hold_scale = float(
            self.get_parameter('moving_position_hold_scale').value
        )

        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        imu_topic = str(self.get_parameter('imu_topic').value)
        joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        left_cmd_topic = str(self.get_parameter('left_cmd_topic').value)
        right_cmd_topic = str(self.get_parameter('right_cmd_topic').value)

        self.pitch = 0.0
        self.pitch_rate = 0.0
        self.wheel_pos = 0.0
        self.wheel_vel = 0.0
        self.filtered_pitch = 0.0
        self.filtered_pitch_rate = 0.0
        self.filtered_wheel_vel = 0.0

        self.wheel_pos_ref = None
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.last_left_raw_pos = None
        self.last_right_raw_pos = None
        self.commanded_linear = 0.0
        self.commanded_angular = 0.0
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.last_cmd_vel_time = None
        self.velocity_error_integral = 0.0

        self.have_imu = False
        self.have_joint_state = False
        self.last_command = 0.0

        self.create_subscription(Twist, cmd_vel_topic, self.cmd_vel_cb, 20)
        self.create_subscription(Imu, imu_topic, self.imu_cb, 50)
        self.create_subscription(JointState, joint_states_topic, self.joint_states_cb, 50)
        self.pub_l = self.create_publisher(Float64, left_cmd_topic, 20)
        self.pub_r = self.create_publisher(Float64, right_cmd_topic, 20)

        self.last_debug_time = self.get_clock().now()
        self.create_timer(0.01, self.control_loop)
        self.get_logger().info(
            'LQR balance controller started with state feedback from '
            f'{imu_topic}, {joint_states_topic}, cmd_vel={cmd_vel_topic}; '
            f'K={np.array2string(self.lqr_gain, precision=3)}'
        )

    def cmd_vel_cb(self, msg: Twist) -> None:
        self.target_linear = clamp(
            msg.linear.x,
            -self.cmd_vel_linear_limit,
            self.cmd_vel_linear_limit,
        )
        self.target_angular = clamp(
            msg.angular.z,
            -self.cmd_vel_angular_limit,
            self.cmd_vel_angular_limit,
        )
        self.last_cmd_vel_time = self.get_clock().now()

    def imu_cb(self, msg: Imu) -> None:
        self.have_imu = True
        raw_pitch = quaternion_to_pitch(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        raw_pitch_rate = msg.angular_velocity.y
        self.pitch = raw_pitch
        self.pitch_rate = raw_pitch_rate
        self.filtered_pitch = (
            (1.0 - self.pitch_alpha) * self.filtered_pitch
            + self.pitch_alpha * raw_pitch
        )
        self.filtered_pitch_rate = (
            (1.0 - self.pitch_rate_alpha) * self.filtered_pitch_rate
            + self.pitch_rate_alpha * raw_pitch_rate
        )

    def joint_states_cb(self, msg: JointState) -> None:
        indices = {}
        for i, name in enumerate(msg.name):
            indices[name] = i

        if 'joint_lw' not in indices or 'joint_rw' not in indices:
            return

        li = indices['joint_lw']
        ri = indices['joint_rw']
        left_raw_pos = msg.position[li]
        right_raw_pos = msg.position[ri]

        if self.last_left_raw_pos is None or self.last_right_raw_pos is None:
            self.left_wheel_pos = left_raw_pos
            self.right_wheel_pos = right_raw_pos
        else:
            self.left_wheel_pos += wrap_to_pi(left_raw_pos - self.last_left_raw_pos)
            self.right_wheel_pos += wrap_to_pi(right_raw_pos - self.last_right_raw_pos)

        self.last_left_raw_pos = left_raw_pos
        self.last_right_raw_pos = right_raw_pos

        self.have_joint_state = True
        self.wheel_pos = 0.5 * (self.left_wheel_pos + self.right_wheel_pos)
        self.wheel_vel = 0.5 * (msg.velocity[li] + msg.velocity[ri])
        self.filtered_wheel_vel = (
            (1.0 - self.wheel_vel_alpha) * self.filtered_wheel_vel
            + self.wheel_vel_alpha * self.wheel_vel
        )
        if self.wheel_pos_ref is None:
            self.wheel_pos_ref = self.wheel_pos
            self.get_logger().info(
                'Initial references set: '
                f'pitch_ref={self.pitch_ref:.4f} rad, wheel_pos_ref={self.wheel_pos_ref:.4f} rad'
            )

    def publish_wheel_speeds(self, left_wheel_speed: float, right_wheel_speed: float) -> None:
        left_cmd = Float64()
        left_cmd.data = left_wheel_speed
        right_cmd = Float64()
        right_cmd.data = right_wheel_speed
        self.pub_l.publish(left_cmd)
        self.pub_r.publish(right_cmd)

    def control_loop(self) -> None:
        if not self.have_imu or self.wheel_pos_ref is None:
            self.last_command = 0.0
            self.publish_wheel_speeds(0.0, 0.0)
            return

        pitch_error = self.filtered_pitch - self.pitch_ref
        if abs(pitch_error) > self.fall_pitch_limit:
            self.last_command = 0.0
            self.commanded_linear = 0.0
            self.commanded_angular = 0.0
            self.velocity_error_integral = 0.0
            self.publish_wheel_speeds(0.0, 0.0)
            return

        now = self.get_clock().now()
        if self.last_cmd_vel_time is not None:
            cmd_age = (now - self.last_cmd_vel_time).nanoseconds / 1e9
            if cmd_age > self.cmd_vel_timeout:
                self.target_linear = 0.0
                self.target_angular = 0.0

        self.commanded_linear = (
            (1.0 - self.cmd_vel_alpha) * self.commanded_linear
            + self.cmd_vel_alpha * self.target_linear
        )
        self.commanded_angular = (
            (1.0 - self.cmd_vel_alpha) * self.commanded_angular
            + self.cmd_vel_alpha * self.target_angular
        )

        wheel_vel_ref = -self.commanded_linear / self.wheel_radius
        moving_cmd = abs(self.commanded_linear) > self.cmd_vel_active_threshold
        if moving_cmd:
            self.wheel_pos_ref = (
                (1.0 - self.wheel_pos_ref_alpha) * self.wheel_pos_ref
                + self.wheel_pos_ref_alpha * self.wheel_pos
            )
        else:
            self.wheel_pos_ref = (
                (1.0 - self.idle_wheel_pos_ref_alpha) * self.wheel_pos_ref
                + self.idle_wheel_pos_ref_alpha * self.wheel_pos
            )
        velocity_error = wheel_vel_ref - self.filtered_wheel_vel
        if moving_cmd:
            self.velocity_error_integral = clamp(
                self.velocity_error_integral + velocity_error * self.dt,
                -self.velocity_integral_limit,
                self.velocity_integral_limit,
            )
        else:
            self.velocity_error_integral *= 0.9

        pitch_target = clamp(
            -(
                self.velocity_pitch_kp * velocity_error
                + self.velocity_pitch_ki * self.velocity_error_integral
            ),
            -self.max_pitch_target,
            self.max_pitch_target,
        )
        pitch_error = self.filtered_pitch - (self.pitch_ref + pitch_target)
        if abs(pitch_error) < self.pitch_deadband:
            pitch_error = 0.0
        wheel_pos_error = self.wheel_pos - self.wheel_pos_ref
        if moving_cmd:
            wheel_pos_error *= self.moving_position_hold_scale
        state = np.array([
            wheel_pos_error,
            self.filtered_wheel_vel,
            pitch_error,
            self.filtered_pitch_rate,
        ])
        target_balance_speed = -float(self.lqr_gain @ state)
        target_balance_speed = clamp(
            target_balance_speed,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )
        target_wheel_speed = clamp(
            target_balance_speed + self.wheel_speed_feedforward_gain * wheel_vel_ref,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )
        filtered_wheel_speed = (
            (1.0 - self.command_alpha) * self.last_command
            + self.command_alpha * target_wheel_speed
        )
        delta = clamp(
            filtered_wheel_speed - self.last_command,
            -self.max_cmd_step,
            self.max_cmd_step,
        )
        balance_speed = self.last_command + delta
        self.last_command = balance_speed

        turn_bias = -0.5 * self.commanded_angular * self.wheel_track / self.wheel_radius
        left_wheel_speed = clamp(
            balance_speed - turn_bias,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )
        right_wheel_speed = clamp(
            balance_speed + turn_bias,
            -self.max_wheel_speed,
            self.max_wheel_speed,
        )
        self.publish_wheel_speeds(left_wheel_speed, right_wheel_speed)

        if (now - self.last_debug_time).nanoseconds > 1_000_000_000:
            self.last_debug_time = now
            self.get_logger().info(
                f'pitch_err={pitch_error:.3f}, pitch_rate={self.filtered_pitch_rate:.3f}, '
                f'wheel_pos_err={wheel_pos_error:.3f}, wheel_vel={self.filtered_wheel_vel:.3f}, '
                f'cmd_v={self.commanded_linear:.3f}, cmd_w={self.commanded_angular:.3f}, '
                f'wheel_vel_ref={wheel_vel_ref:.3f}, pitch_target={pitch_target:.3f}, '
                f'left={left_wheel_speed:.3f}, right={right_wheel_speed:.3f}'
            )


def main() -> None:
    rclpy.init()
    node = BalanceController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
