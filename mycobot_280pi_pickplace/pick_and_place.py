#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import time


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_node')
        self.get_logger().info('Pick and Place Node has been started.')

        self._action_client = ActionClient(self, MoveGroup, '/move_group')

        self.down_orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

        self.poses = {
            'pre_grasp': Pose(
                position=Point(x=0.10, y=0.00, z=0.15),
                orientation=self.down_orientation
            ),
            'grasp': Pose(
                position=Point(x=0.10, y=0.00, z=0.05),
                orientation=self.down_orientation
            ),
            'lift': Pose(
                position=Point(x=0.10, y=0.00, z=0.20),
                orientation=self.down_orientation
            ),
            'pre_place': Pose(
                position=Point(x=0.15, y=0.05, z=0.15),
                orientation=self.down_orientation
            ),
            'place': Pose(
                position=Point(x=0.15, y=0.05, z=0.05),
                orientation=self.down_orientation
            ),
        }

        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('MoveGroup ready. Starting sequence...')

    def move_to_pose(self, pose: Pose, description: str) -> bool:
        self.get_logger().info(
            f'Moving to [{description}]: '
            f'x={pose.position.x:.3f} '
            f'y={pose.position.y:.3f} '
            f'z={pose.position.z:.3f}'
        )

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        pos_constraint = PositionConstraint()
        pos_constraint.header = Header(frame_id='base_link')
        pos_constraint.link_name = 'link6_flange'
        pos_constraint.weight = 1.0

        tolerance = SolidPrimitive()
        tolerance.type = SolidPrimitive.BOX
        tolerance.dimensions = [0.01, 0.01, 0.01]

        bounding_volume = BoundingVolume()
        bounding_volume.primitives = [tolerance]
        bounding_volume.primitive_poses = [pose]
        pos_constraint.constraint_region = bounding_volume  # fixed

        ori_constraint = OrientationConstraint()
        ori_constraint.header = Header(frame_id='base_link')
        ori_constraint.link_name = 'link6_flange'
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.2
        ori_constraint.absolute_y_axis_tolerance = 0.2
        ori_constraint.absolute_z_axis_tolerance = 0.2
        ori_constraint.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        goal_msg.request.goal_constraints = [constraints]

        return self._send_goal_and_wait(goal_msg)

    def move_to_named_target(self, name: str) -> bool:
        self.get_logger().info(f'Moving to named target: [{name}]')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        goal_msg.request.goal_constraints = [Constraints(name=name)]

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
        self.get_logger().info('=' * 40)
        self.get_logger().info('STARTING PICK AND PLACE SEQUENCE')
        self.get_logger().info('=' * 40)

        self.get_logger().info('\n--- Step 1/8: HOME ---')
        if not self.move_to_named_target('home'):
            self.get_logger().error('ABORT: Failed at HOME')
            return

        self.get_logger().info('\n--- Step 2/8: PRE-GRASP ---')
        if not self.move_to_pose(self.poses['pre_grasp'], 'pre_grasp'):
            self.get_logger().error('ABORT: Failed at PRE-GRASP')
            return

        self.get_logger().info('\n--- Step 3/8: GRASP ---')
        if not self.move_to_pose(self.poses['grasp'], 'grasp'):
            self.get_logger().error('ABORT: Failed at GRASP')
            return

        self.get_logger().info('\n--- Step 4/8: CLOSE GRIPPER ---')
        self.simulate_gripper('close')

        self.get_logger().info('\n--- Step 5/8: LIFT ---')
        if not self.move_to_pose(self.poses['lift'], 'lift'):
            self.get_logger().error('ABORT: Failed at LIFT')
            return

        self.get_logger().info('\n--- Step 6/8: PRE-PLACE ---')
        if not self.move_to_pose(self.poses['pre_place'], 'pre_place'):
            self.get_logger().error('ABORT: Failed at PRE-PLACE')
            return

        self.get_logger().info('\n--- Step 7/8: PLACE ---')
        if not self.move_to_pose(self.poses['place'], 'place'):
            self.get_logger().error('ABORT: Failed at PLACE')
            return

        self.get_logger().info('\n--- Step 8/8: OPEN GRIPPER ---')
        self.simulate_gripper('open')

        self.get_logger().info('\n--- Returning HOME ---')
        self.move_to_named_target('home')

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