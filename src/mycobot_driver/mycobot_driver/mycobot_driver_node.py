#!/usr/bin/env python3
"""
mycobot_driver_node — bridges MoveIt2 (WSL2, ROS 2 Jazzy) to the real myCobot
280 Pi, replacing the ros2_control hardware interface for the real robot.

Talks to mycobot_tcp_bridge.py on the robot's Raspberry Pi over a simple ASCII
TCP protocol (GET_ANGLES / SET_ANGLES / PING) — no pymycobot or DDS on this side.

Two jobs:
  1. Publishes /joint_states for the 6 arm joints.
       - idle:      the robot's real angles (GET_ANGLES; last-good held on ERR)
       - executing: the interpolated commanded setpoint (open-loop), so MoveIt's
                    execution monitor sees the expected motion (get_angles fails
                    while the arm is moving, so we must not feed it back live).
  2. Serves the FollowJointTrajectory action MoveIt calls
     (/arm_controller/follow_joint_trajectory): streams the trajectory points to
     the robot via SET_ANGLES, paced by time_from_start, then succeeds.

Mirrors the philosophy of Elephant's official slider_control.py / listen_real.py.
"""
import math
import socket
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from builtin_interfaces.msg import Time as TimeMsg  # noqa: F401  (clarity)
from control_msgs.action import FollowJointTrajectory, GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

RAD2DEG = 180.0 / math.pi
DEG2RAD = math.pi / 180.0
ARM_JOINTS = 6


class BridgeClient:
    """Thread-safe newline-ASCII TCP client to mycobot_tcp_bridge.py."""

    def __init__(self, ip, port, logger):
        self._ip = ip
        self._port = port
        self._log = logger
        self._lock = threading.Lock()
        self._sock = None
        self._rfile = None

    def connect(self):
        with self._lock:
            self._connect_locked()

    def _connect_locked(self):
        self._close_locked()
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(5.0)
        s.connect((self._ip, self._port))
        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock = s
        self._rfile = s.makefile('r', encoding='ascii', newline='\n')

    def _close_locked(self):
        try:
            if self._rfile:
                self._rfile.close()
        except Exception:
            pass
        try:
            if self._sock:
                self._sock.close()
        except Exception:
            pass
        self._sock = None
        self._rfile = None

    def transact(self, request):
        """Send one line, return the stripped reply, or None on socket error."""
        with self._lock:
            if self._sock is None:
                try:
                    self._connect_locked()
                except OSError as e:
                    self._log.warn(f'bridge connect failed: {e}')
                    return None
            try:
                self._sock.sendall((request + '\n').encode('ascii'))
                line = self._rfile.readline()
                if line == '':
                    raise OSError('bridge closed connection')
                return line.strip()
            except OSError as e:
                self._log.warn(f'bridge transaction failed ({request!r}): {e}')
                self._close_locked()
                return None

    def close(self):
        with self._lock:
            self._close_locked()


class MyCobotDriver(Node):
    def __init__(self):
        super().__init__('mycobot_driver')

        self.declare_parameter('robot_ip', '192.168.0.101')
        self.declare_parameter('robot_port', 9000)
        self.declare_parameter('joint_speed', 30)
        self.declare_parameter('state_rate_hz', 20.0)
        self.declare_parameter('arm_joints', [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'])
        # Gripper: maps the gripper_controller joint position (rad) to the
        # myCobot 0-100 gripper value. open_pos -> 100 (open), closed_pos -> 0.
        # Set gripper_invert:=true if open/close come out reversed on the real gripper.
        self.declare_parameter('gripper_joint', 'gripper_controller')
        self.declare_parameter('gripper_open_pos', 0.0)
        self.declare_parameter('gripper_closed_pos', -0.5)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('gripper_invert', False)

        self._ip = self.get_parameter('robot_ip').value
        self._port = int(self.get_parameter('robot_port').value)
        self._speed = int(self.get_parameter('joint_speed').value)
        self._rate = float(self.get_parameter('state_rate_hz').value)
        self._arm_joints = list(self.get_parameter('arm_joints').value)
        self._grip_joint = self.get_parameter('gripper_joint').value
        self._grip_open_pos = float(self.get_parameter('gripper_open_pos').value)
        self._grip_closed_pos = float(self.get_parameter('gripper_closed_pos').value)
        self._grip_speed = int(self.get_parameter('gripper_speed').value)
        self._grip_invert = bool(self.get_parameter('gripper_invert').value)
        self._gripper_pos = self._grip_open_pos   # published setpoint (rad)

        self._client = BridgeClient(self._ip, self._port, self.get_logger())

        # Shared state between the action (writer) and the state timer (reader).
        self._lock = threading.Lock()
        self._executing = False
        self._last_pos_rad = [0.0] * ARM_JOINTS   # last known robot angles (rad)
        self._setpoint_rad = [0.0] * ARM_JOINTS    # current commanded setpoint (rad)

        # Verify the bridge is reachable, then seed state from the real arm.
        if self._client.transact('PING') != 'PONG':
            self.get_logger().error(
                f'No PONG from bridge at {self._ip}:{self._port} — is '
                f'mycobot_tcp_bridge.py running on the Pi?')
        else:
            self.get_logger().info(f'Connected to bridge at {self._ip}:{self._port}')
        seeded = self._read_real_angles()
        if seeded is not None:
            with self._lock:
                self._last_pos_rad = seeded
                self._setpoint_rad = list(seeded)
            self.get_logger().info(f'Seeded from real arm: '
                                   f'{[round(a, 3) for a in seeded]} rad')

        cb = ReentrantCallbackGroup()
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(1.0 / self._rate, self._publish_state, callback_group=cb)

        self._action = ActionServer(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)

        # Gripper: MoveIt's GripperCommand controller (moveit_controllers.yaml).
        self._gripper_action = ActionServer(
            self, GripperCommand,
            '/gripper_action_controller/gripper_cmd',
            execute_callback=self._execute_gripper,
            goal_callback=lambda _g: GoalResponse.ACCEPT,
            cancel_callback=lambda _g: CancelResponse.ACCEPT,
            callback_group=cb)

        # Free-drive / drag-teaching services.
        self.create_service(Trigger, '~/release_servos', self._srv_release, callback_group=cb)
        self.create_service(Trigger, '~/focus_servos', self._srv_focus, callback_group=cb)
        self.create_service(Trigger, '~/get_pose', self._srv_get_pose, callback_group=cb)

        self.get_logger().info(
            'mycobot_driver ready — actions /arm_controller/follow_joint_trajectory + '
            '/gripper_action_controller/gripper_cmd, publishing /joint_states, '
            'services: ~/release_servos ~/focus_servos ~/get_pose')

    # ── robot I/O ────────────────────────────────────────────────────────────

    def _read_real_angles(self):
        """Return 6 joint angles (rad) from the robot, or None on failure/ERR."""
        reply = self._client.transact('GET_ANGLES')
        if not reply or reply == 'ERR':
            return None
        try:
            deg = [float(x) for x in reply.split(',')]
        except ValueError:
            return None
        if len(deg) != ARM_JOINTS:
            return None
        return [d * DEG2RAD for d in deg]

    def _send_angles(self, angles_rad):
        deg = [a * RAD2DEG for a in angles_rad]
        req = 'SET_ANGLES ' + ' '.join(f'{d:.3f}' for d in deg) + f' {self._speed}'
        return self._client.transact(req) == 'OK'

    # ── /joint_states publisher ────────────────────────────────────────────────

    def _publish_state(self):
        with self._lock:
            executing = self._executing
            setpoint = list(self._setpoint_rad)
            last = list(self._last_pos_rad)

        if executing:
            # Open-loop: report the commanded setpoint (get_angles fails mid-motion).
            positions = setpoint
        else:
            real = self._read_real_angles()
            if real is not None:
                positions = real
                with self._lock:
                    self._last_pos_rad = real
                    if not self._executing:
                        self._setpoint_rad = list(real)
            else:
                positions = last  # hold last-good

        with self._lock:
            grip_pos = self._gripper_pos

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._arm_joints) + [self._grip_joint]
        msg.position = list(positions) + [grip_pos]
        self._pub.publish(msg)

    # ── FollowJointTrajectory action ────────────────────────────────────────────

    def _execute_trajectory(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        # Map trajectory joint order -> our arm order.
        try:
            order = [names.index(j) for j in self._arm_joints]
        except ValueError as e:
            self.get_logger().error(f'Trajectory missing arm joint: {e}')
            goal_handle.abort()
            return self._traj_result(FollowJointTrajectory.Result.INVALID_JOINTS,
                                     'trajectory joint names do not match arm joints')

        points = traj.points
        if not points:
            goal_handle.succeed()
            return self._traj_result(FollowJointTrajectory.Result.SUCCESSFUL, 'empty trajectory')

        self.get_logger().info(f'Executing trajectory: {len(points)} points')
        with self._lock:
            self._executing = True

        start = self.get_clock().now()
        try:
            for pt in points:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    return self._traj_result(FollowJointTrajectory.Result.SUCCESSFUL, 'canceled')

                target = [pt.positions[i] for i in order]
                # Pace to this point's time_from_start.
                t_target = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                with self._lock:
                    self._setpoint_rad = list(target)
                self._send_angles(target)

                elapsed = (self.get_clock().now() - start).nanoseconds * 1e-9
                sleep_s = t_target - elapsed
                if sleep_s > 0:
                    self._sleep_ros(sleep_s)

            # Make sure the final target is held.
            final = [points[-1].positions[i] for i in order]
            with self._lock:
                self._setpoint_rad = list(final)
            self._send_angles(final)
        finally:
            with self._lock:
                self._executing = False

        goal_handle.succeed()
        self.get_logger().info('Trajectory complete')
        return self._traj_result(FollowJointTrajectory.Result.SUCCESSFUL, 'ok')

    @staticmethod
    def _traj_result(code, msg):
        result = FollowJointTrajectory.Result()
        result.error_code = code
        result.error_string = msg
        return result

    # ── gripper (GripperCommand action) ─────────────────────────────────────────

    def _pos_to_gripper_value(self, pos):
        """Map gripper_controller joint position (rad) -> myCobot value 0(closed)-100(open)."""
        op, cl = self._grip_open_pos, self._grip_closed_pos
        if op == cl:
            v = 100.0
        else:
            v = (pos - cl) / (op - cl) * 100.0
        v = max(0.0, min(100.0, v))
        if self._grip_invert:
            v = 100.0 - v
        return int(round(v))

    def _execute_gripper(self, goal_handle):
        pos = goal_handle.request.command.position
        value = self._pos_to_gripper_value(pos)
        self.get_logger().info(f'Gripper: position {pos:.3f} rad -> value {value}')
        with self._lock:
            self._gripper_pos = pos
        ok = self._client.transact(f'SET_GRIPPER {value} {self._grip_speed}') == 'OK'
        self._sleep_ros(1.5)  # let the gripper finish moving

        result = GripperCommand.Result()
        result.position = pos
        result.effort = 0.0
        result.stalled = False
        result.reached_goal = ok
        if ok:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    def _sleep_ros(self, seconds):
        # Simple wall-clock sleep; fine for this driver (system clock, not sim time).
        import time
        time.sleep(seconds)

    # ── free-drive / drag-teaching services ─────────────────────────────────────

    def _srv_release(self, _request, response):
        """Relax all servos so the arm can be moved by hand."""
        if self._executing:
            response.success = False
            response.message = 'Cannot release while a trajectory is executing'
            return response
        ok = self._client.transact('RELEASE') == 'OK'
        response.success = ok
        response.message = (
            'Servos released — arm is free to move by hand. SUPPORT THE ARM; it may '
            'sag under gravity. Call ~/focus_servos to re-engage before planning.'
            if ok else 'RELEASE failed (bridge unreachable?)')
        self.get_logger().warn(response.message)
        return response

    def _srv_focus(self, _request, response):
        """Re-energize servos; the arm holds its current position."""
        ok = self._client.transact('FOCUS') == 'OK'
        # Re-sync our setpoint to wherever the arm now is, so the next plan/execute
        # starts from the current (possibly hand-moved) pose.
        real = self._read_real_angles()
        if real is not None:
            with self._lock:
                self._last_pos_rad = real
                self._setpoint_rad = list(real)
        response.success = ok
        response.message = 'Servos focused — arm holding position' if ok else 'FOCUS failed'
        self.get_logger().info(response.message)
        return response

    def _srv_get_pose(self, _request, response):
        """Report current joint angles (rad + deg) and Cartesian coords."""
        rad = self._read_real_angles()
        coords_reply = self._client.transact('GET_COORDS')
        if rad is None:
            response.success = False
            response.message = 'Could not read joint angles (arm moving or bridge down?)'
            return response
        deg = [a * RAD2DEG for a in rad]
        msg = ('angles_rad: [' + ', '.join(f'{a:.4f}' for a in rad) + ']\n'
               'angles_deg: [' + ', '.join(f'{d:.2f}' for d in deg) + ']')
        if coords_reply and coords_reply != 'ERR':
            msg += '\ncoords [x,y,z mm, rx,ry,rz deg]: [' + coords_reply + ']'
        response.success = True
        response.message = msg
        self.get_logger().info('\n' + msg)
        return response

    def destroy_node(self):
        self._client.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyCobotDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
