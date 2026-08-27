#!/usr/bin/env python3

import rclpy
import os, time
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import time
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    JointConstraint,
)


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_node')
        self.get_logger().info('Pick and Place Node has been started.')

        self._action_client = ActionClient(self, MoveGroup, '/move_action')

        # All poses defined as joint angles in radians
        # [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3,
        #  joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6]
        self.joint_poses = {
        'home':      [ 0.000,  0.000,  0.000,  0.000,  0.000,  0.000],
        'pre_grasp': [-1.675,  1.588,  0.698,  0.855,  0.000,  0.000],
        'grasp':     [-1.536, -0.035,  0.698,  0.855,  0.000,  0.000],
        'lift':      [-1.536,  0.646,  0.698,  0.279, -0.716, -0.209],
        'pre_place': [ 1.571,  0.646,  0.698,  0.279, -0.716, -0.209],
        'place':     [ 1.571,  1.484,  0.960,  0.593, -0.716, -0.209],
        }

        self.joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ]

        self.get_logger().info('Waiting for MoveGroup action server...')
        server_ready = self._action_client.wait_for_server(timeout_sec=10.0)
        if not server_ready:
            self.get_logger().error('MoveGroup action server not available!')
        else:
            self.get_logger().info('MoveGroup ready.')
        
        self.trigger_file = '/home/youniq/ros2ws/trigger.txt'
        self.trigger_word = 'PICK'

    def wait_for_trigger(self):
        self.get_logger().info(
            f'waiting for trigger word "{self.trigger_word}" in file: {self.trigger_file}'
        )
        if not os.path.exists(self.trigger_file):
            with open(self.trigger_file, 'w') as f:
                f.write('')  # create empty file if it doesn't exist
            self.get_logger().info(f'Created trigger file: {self.trigger_file}')

        while rclpy.ok():
            try:
                with open(self.trigger_file, 'r') as f:
                    content = f.read().strip()
                if self.trigger_word in content:
                    self.get_logger().info(f'Trigger word "{self.trigger_word}" detected.')
                    with open(self.trigger_file, 'w') as f:
                        f.write('')  # clear the file after detecting trigger
                    return True
            except Exception as e:
                self.get_logger().error(f'Error reading trigger file: {e}')
                time.sleep(0.5)  # check every 0.5 seconds
        return False  # if rclpy is not ok, exit the loop   
    def move_to_joint_target(self, pose_name: str) -> bool:
        """
        Move to a pose defined by joint angles.
        Joint space planning is more reliable than Cartesian IK
        for hardcoded pick and place sequences.
        """
        if pose_name not in self.joint_poses:
            self.get_logger().error(f'Unknown pose: {pose_name}')
            return False

        positions = self.joint_poses[pose_name]
        self.get_logger().info(
            f'Moving to [{pose_name}]: '
            f'{[round(p, 3) for p in positions]}'
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        joint_constraints = []
        for joint_name, position in zip(self.joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints = [constraints]

        return self._send_goal_and_wait(goal_msg)

    def move_to_named_target(self, name: str) -> bool:
        """
        Move to a named pose using explicit joint values.
        We send joint constraints directly because the raw action
        client cannot resolve SRDF named states by string reference.
        """
        self.get_logger().info(f'Moving to named target: [{name}]')

        # Joint values for 'home' — all zeros
        # Joint values for 'ready' — from SRDF definition
        joint_positions = {
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'ready': [0.0, -0.5, 0.5, 0.0, 0.0, 0.0],
        }

        if name not in joint_positions:
            self.get_logger().error(f'Unknown named target: {name}')
            return False

        positions = joint_positions[name]
        joint_names = [
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6',
        ]

        from moveit_msgs.msg import JointConstraint

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        # Build joint constraints — one per joint
        joint_constraints = []
        for joint_name, position in zip(joint_names, positions):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = position
            jc.tolerance_above = 0.01  # 0.01 rad tolerance
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            joint_constraints.append(jc)

        constraints = Constraints()
        constraints.joint_constraints = joint_constraints
        goal_msg.request.goal_constraints = [constraints]

        return self._send_goal_and_wait(goal_msg)

    def _send_goal_and_wait(self, goal_msg: MoveGroup.Goal) -> bool:
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup')
            return False

        self.get_logger().info('Goal accepted. Executing...')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        error_code = result_future.result().result.error_code.val
        if error_code == 1:
            self.get_logger().info('Motion SUCCESS')
            return True
        else:
            self.get_logger().error(f'Motion FAILED — error code: {error_code}')
            return False

    def simulate_gripper(self, action: str):
        self.get_logger().info(
            f'GRIPPER: {action.upper()} (simulated — 1 second delay)'
        )
        time.sleep(1.0)

    def execute_pick_and_place(self):
        if not self.wait_for_trigger():
            self.get_logger().error('Trigger wait interrupted.')
            return  
        self.get_logger().info('=' * 40)
        self.get_logger().info('STARTING PICK AND PLACE SEQUENCE')
        self.get_logger().info('=' * 40)

        self.get_logger().info('\n--- Step 1/8: HOME ---')
        if not self.move_to_joint_target('home'):
            self.get_logger().error('ABORT: Failed at HOME')
            return

        self.get_logger().info('\n--- Step 2/8: PRE-GRASP ---')
        if not self.move_to_joint_target('pre_grasp'):
            self.get_logger().error('ABORT: Failed at PRE-GRASP')
            return

        self.get_logger().info('\n--- Step 3/8: GRASP ---')
        if not self.move_to_joint_target('grasp'):
            self.get_logger().error('ABORT: Failed at GRASP')
            return

        self.get_logger().info('\n--- Step 4/8: CLOSE GRIPPER ---')
        self.simulate_gripper('close')

        self.get_logger().info('\n--- Step 5/8: LIFT ---')
        if not self.move_to_joint_target('lift'):
            self.get_logger().error('ABORT: Failed at LIFT')
            return

        self.get_logger().info('\n--- Step 6/8: PRE-PLACE ---')
        if not self.move_to_joint_target('pre_place'):
            self.get_logger().error('ABORT: Failed at PRE-PLACE')
            return

        self.get_logger().info('\n--- Step 7/8: PLACE ---')
        if not self.move_to_joint_target('place'):
            self.get_logger().error('ABORT: Failed at PLACE')
            return

        self.get_logger().info('\n--- Step 8/8: OPEN GRIPPER ---')
        self.simulate_gripper('open')

        self.get_logger().info('\n--- Returning HOME ---')
        self.move_to_joint_target('home')

        self.get_logger().info('=' * 40)
        self.get_logger().info('SEQUENCE COMPLETE')
        self.get_logger().info('=' * 40)


# ── THIS IS OUTSIDE THE CLASS ──────────────────────────────────────
# main() must be at module level, not indented inside the class.
# setup.py entry_points looks for mycobot_280pi_pickplace.pick_and_place:main
# which means: in module pick_and_place, find function named main.
# If main is inside the class, Python finds it as a method, not a function.

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        node.execute_pick_and_place()
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()