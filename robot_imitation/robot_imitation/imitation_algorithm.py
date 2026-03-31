#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion
from robot_interfaces.msg import HandState
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, RobotState, OrientationConstraint, MoveItErrorCodes
from tf2_ros import TransformListener, Buffer, TransformException
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
import signal
import numpy as np
from scipy.spatial.transform import Rotation as R
import threading

class HandToRobotController(Node):
    def __init__(self):
        super().__init__('hand_to_robot_controller')
        
        self.get_logger().info("Hand to Robot Controller started - waiting for HandState messages")
        
        # Use reentrant callback group for parallel processing
        self.cb_group = MutuallyExclusiveCallbackGroup()
        
        # Initialize all components
        self.initialize_parameters()
        self.initialize_subscribers()
        self.initialize_tf()
        self.initialize_action_clients()
        self.initialize_gripper_actions()
            
        self.get_logger().info("Hand to Robot Controller fully initialized")

    def initialize_parameters(self):
        # Basic parameters
        self.declare_parameter('debug', True)
        self.debug_param = self.get_parameter('debug').value

        self.plan_lock = threading.Lock()
        
        self.declare_parameter('robot1_name', 'robot_one')
        self.robot1_name = self.get_parameter('robot1_name').get_parameter_value().string_value
        self.declare_parameter('robot2_name', 'robot_two') 
        self.robot2_name = self.get_parameter('robot2_name').get_parameter_value().string_value
        self.declare_parameter('position_scale', 0.30)
        self.position_scale = self.get_parameter('position_scale').get_parameter_value().double_value

        # Control parameters
        self.declare_parameter('planning_time', 5.0)
        self.planning_time = self.get_parameter('planning_time').get_parameter_value().double_value
        self.declare_parameter('velocity_scaling', 0.3)
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value().double_value
        self.declare_parameter('position_threshold', 0.001)  # 2cm movement threshold
        self.position_threshold = self.get_parameter('position_threshold').get_parameter_value().double_value
        self.declare_parameter('orientation_threshold', 0.2)  # radians
        self.orientation_threshold = self.get_parameter('orientation_threshold').get_parameter_value().double_value
        self.last_plan_time = {self.robot1_name: 0.0, self.robot2_name: 0.0}
        self.plan_cooldown_sec = 1.0

        self.declare_parameter('diagnostic_mode', 'exact_current_pose')
        self.diagnostic_mode = self.get_parameter('diagnostic_mode').get_parameter_value().string_value

        # State tracking dictionaries
        self.joint_states = None
        self.execution_in_progress = {self.robot1_name: False, self.robot2_name: False}
        self.latest_hand_states = {self.robot1_name: None, self.robot2_name: None}
        self.planning_in_progress = {self.robot1_name: False, self.robot2_name: False}
        self.last_processed_pose = {self.robot1_name: None, self.robot2_name: None}
        self.active_goals = {self.robot1_name: None, self.robot2_name: None}
        self.initial_robot_poses = {self.robot1_name: None, self.robot2_name: None}
        self.last_gripper_states = {self.robot1_name: 0.0, self.robot2_name: 0.0}

        self.hand_missing_counts = {self.robot1_name: 0, self.robot2_name: 0}
        self.max_missing_frames = 5

    def initialize_subscribers(self):
        # Subscribe to hand state topics published by the hand tracking node
        self.left_hand_sub = self.create_subscription(
            HandState, '/left_hand_state', self.left_hand_callback, 
            10, callback_group=self.cb_group)
        self.right_hand_sub = self.create_subscription(
            HandState, '/right_hand_state', self.right_hand_callback,
            10, callback_group=self.cb_group)
        
        # Subscribe to joint states for robot state information
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback,
            10, callback_group=self.cb_group)

        self.get_logger().info("Subscribed to /left_hand_state and /right_hand_state topics")

    def initialize_tf(self):
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def initialize_action_clients(self):
        self.move_group_client = ActionClient(
            self, MoveGroup, "/move_action", callback_group=self.cb_group
        )

        self.get_logger().info("Waiting for MoveGroup action server...")

        if not self.move_group_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().warning("MoveGroup action server not available")
            self.move_group_client = None
        else:
            self.get_logger().info("MoveGroup action server ready")

    def initialize_gripper_actions(self):
        # Gripper action clients
        self.robot1_gripper_client = ActionClient(
            self, GripperCommand, '/gripper_robot_one_controller/gripper_cmd',
            callback_group=self.cb_group)
        self.robot2_gripper_client = ActionClient(
            self, GripperCommand, '/gripper_robot_two_controller/gripper_cmd',
            callback_group=self.cb_group)
        
        self.get_logger().info("Gripper action clients initialized")

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def get_tip_link(self, robot_name):
        return "link6_robot_1" if robot_name == self.robot1_name else "link6_robot_2"

    def left_hand_callback(self, msg):
        """Process left hand state and control robot_one"""
        if msg.detected:
            if self.debug_param:
                self.get_logger().info(
                    f"Left hand - Pos: ({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}) "
                    f"Orient: ({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, "
                    f"{msg.orientation.z:.3f}, {msg.orientation.w:.3f}) "
                    f"Closed: {msg.is_closed}"
                )
            self.hand_missing_counts[self.robot1_name] = 0
            self.process_hand_state(self.robot1_name, msg)

        else:
            self.hand_missing_counts[self.robot1_name] += 1
            if self.hand_missing_counts[self.robot1_name] >= self.max_missing_frames:
                self.initial_robot_poses[self.robot1_name] = None
                self.last_processed_pose[self.robot1_name] = None
                self.latest_hand_states[self.robot1_name] = None
            if self.debug_param:
                self.get_logger().info("Left hand not detected")

    def right_hand_callback(self, msg):
        """Process right hand state and control robot_two"""
        if msg.detected:
            if self.debug_param:
                self.get_logger().info(
                    f"Right hand - Pos: ({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f}) "
                    f"Orient: ({msg.orientation.x:.3f}, {msg.orientation.y:.3f}, "
                    f"{msg.orientation.z:.3f}, {msg.orientation.w:.3f}) "
                    f"Closed: {msg.is_closed}"
                )
            self.hand_missing_counts[self.robot2_name] = 0
            self.process_hand_state(self.robot2_name, msg)
        else:
            self.hand_missing_counts[self.robot2_name] += 1
            if self.hand_missing_counts[self.robot2_name] >= self.max_missing_frames:
                self.initial_robot_poses[self.robot2_name] = None
                self.last_processed_pose[self.robot2_name] = None
                self.latest_hand_states[self.robot2_name] = None
            if self.debug_param:
                self.get_logger().info("Right hand not detected")

    def process_hand_state(self, robot_name, hand_state):
        try:
            self.latest_hand_states[robot_name] = hand_state
            self.control_gripper(robot_name, hand_state.is_closed)

            with self.plan_lock:
                if self.planning_in_progress[robot_name]:
                    return

            self.process_new_hand_state(robot_name)

        except Exception as e:
            self.get_logger().error(f"Error processing {robot_name} hand state: {str(e)}")

    def process_new_hand_state(self, robot_name):
        hand_state = self.latest_hand_states[robot_name]
        if hand_state is None:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        with self.plan_lock:
            if self.planning_in_progress[robot_name]:
                return
            if now_sec - self.last_plan_time[robot_name] < self.plan_cooldown_sec:
                return

        target_pose = self.hand_state_to_robot_pose(robot_name, hand_state)
        if target_pose is None:
            return

        if self.diagnostic_mode not in ["fixed_offset", "exact_current_pose"]:
            if not self.is_movement_significant(robot_name, target_pose):
                return

        with self.plan_lock:
            self.planning_in_progress[robot_name] = True
            self.last_plan_time[robot_name] = now_sec

        self.send_to_planner(robot_name, target_pose)
        self.last_processed_pose[robot_name] = target_pose

    def hand_state_to_robot_pose(self, robot_name, hand_state):
        try:
            current_pose = self.get_current_end_effector_pose(robot_name)
            if current_pose is None:
                self.get_logger().warning(f"Could not get current pose for {robot_name}")
                return None

            if self.initial_robot_poses[robot_name] is None:
                self.initial_robot_poses[robot_name] = current_pose
                self.last_processed_pose[robot_name] = current_pose
                self.get_logger().info(f"Set initial pose for {robot_name}")
                return None

            init_pose = self.initial_robot_poses[robot_name]

            # ---------------- DIAGNOSTIC MODES ----------------
            if self.diagnostic_mode == "exact_current_pose":
                target_pose = Pose()
                target_pose.position.x = current_pose.position.x
                target_pose.position.y = current_pose.position.y
                target_pose.position.z = current_pose.position.z
                target_pose.orientation = current_pose.orientation

                self.get_logger().info(
                    f"[DIAG exact_current_pose] {robot_name}: "
                    f"x={target_pose.position.x:.3f}, "
                    f"y={target_pose.position.y:.3f}, "
                    f"z={target_pose.position.z:.3f}"
                )
                return target_pose

            elif self.diagnostic_mode == "fixed_offset":
                target_pose = Pose()
                target_pose.position.x = current_pose.position.x
                target_pose.position.y = current_pose.position.y + 0.01   # 1 cm
                target_pose.position.z = current_pose.position.z
                target_pose.orientation = current_pose.orientation

                self.get_logger().info(
                    f"[DIAG fixed_offset] {robot_name}: "
                    f"x={target_pose.position.x:.3f}, "
                    f"y={target_pose.position.y:.3f}, "
                    f"z={target_pose.position.z:.3f}"
                )
                return target_pose

            # ---------------- NORMAL HAND CONTROL ----------------
            target_pose = Pose()
            target_pose.position.x = init_pose.position.x
            target_pose.position.y = init_pose.position.y - hand_state.position.x * self.position_scale
            target_pose.position.z = init_pose.position.z - hand_state.position.y * self.position_scale
            target_pose.orientation = init_pose.orientation

            y_min = init_pose.position.y - 0.10
            y_max = init_pose.position.y + 0.10
            z_min = init_pose.position.z - 0.10
            z_max = init_pose.position.z + 0.10

            target_pose.position.y = np.clip(target_pose.position.y, y_min, y_max)
            target_pose.position.z = np.clip(target_pose.position.z, z_min, z_max)

            self.get_logger().info(
                f"{robot_name} target pose: "
                f"x={target_pose.position.x:.3f}, "
                f"y={target_pose.position.y:.3f}, "
                f"z={target_pose.position.z:.3f}"
            )

            return target_pose

        except Exception as e:
            self.get_logger().error(f"Error converting hand state to pose: {str(e)}")
            return None

    def combine_orientations(self, base_orientation, relative_orientation):
        """Combine base orientation with relative orientation using quaternion multiplication"""
        try:
            # Convert to scipy Rotation objects
            base_rot = R.from_quat([base_orientation.x, base_orientation.y, 
                                  base_orientation.z, base_orientation.w])
            relative_rot = R.from_quat([relative_orientation.x, relative_orientation.y,
                                      relative_orientation.z, relative_orientation.w])
            
            # Combine: target = base * relative
            target_rot = base_rot * relative_rot
            target_quat = target_rot.as_quat()
            
            result = Quaternion()
            result.x = target_quat[0]
            result.y = target_quat[1]
            result.z = target_quat[2]
            result.w = target_quat[3]
            
            return result
            
        except Exception as e:
            self.get_logger().warning(f"Orientation combination failed, using base orientation: {str(e)}")
            return base_orientation

    def get_current_end_effector_pose(self, robot_name):
        """Get current end-effector pose using TF"""
        try:
            tip_frame = self.get_tip_link(robot_name)
            base_frame = "base_link_robot_1" if robot_name == self.robot1_name else "base_link_robot_2"
            
            tf = self.tf_buffer.lookup_transform(
                base_frame,
                tip_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = Pose()
            pose.position.x = tf.transform.translation.x
            pose.position.y = tf.transform.translation.y
            pose.position.z = tf.transform.translation.z
            pose.orientation = tf.transform.rotation
            
            return pose
            
        except TransformException as e:
            self.get_logger().warning(f"Could not get end-effector pose for {robot_name}: {str(e)}")
            return None

    def is_movement_significant(self, robot_name, target_pose):
        """Check if movement is significant enough to warrant planning"""
        last_pose = self.last_processed_pose[robot_name]
        if last_pose is None:
            return True  # First movement is always significant
        
        # Position difference
        pos_diff = np.sqrt(
            (target_pose.position.x - last_pose.position.x)**2 +
            (target_pose.position.y - last_pose.position.y)**2 +
            (target_pose.position.z - last_pose.position.z)**2
        )
        
        # Orientation difference (angle between quaternions)
        q1 = np.array([last_pose.orientation.x, last_pose.orientation.y,
                      last_pose.orientation.z, last_pose.orientation.w])
        q2 = np.array([target_pose.orientation.x, target_pose.orientation.y,
                      target_pose.orientation.z, target_pose.orientation.w])
        dot = np.clip(np.abs(np.dot(q1, q2)), -1.0, 1.0)
        orient_diff = 2 * np.arccos(dot)
        
        significant = (pos_diff > self.position_threshold or 
                      orient_diff > self.orientation_threshold)
        
        if self.debug_param and not significant:
            self.get_logger().debug(f"Movement below threshold: pos={pos_diff:.3f}, orient={orient_diff:.3f}")
            
        return significant

    def send_to_planner(self, robot_name, target_pose):
        """Send target pose to MoveIt planner with loose orientation constraints"""
        if self.move_group_client is None:
            self.get_logger().warning("MoveGroup not available, skipping planning")
            self.planning_in_progress[robot_name] = False
            return False
        
        try:
            # Create constraints with LOOSE orientation tolerances
            constraints = self.create_loose_constraints(robot_name, target_pose)
            
            # Create planning request
            request = MotionPlanRequest()
            request.planner_id = "RRTConnect"
            request.group_name = f"manipulator_{robot_name}"
            request.goal_constraints = [constraints]
            request.allowed_planning_time = self.planning_time
            request.max_velocity_scaling_factor = self.velocity_scaling
            request.max_acceleration_scaling_factor = 0.3
            
            # Send to MoveIt
            goal = MoveGroup.Goal()
            goal.request = request
            goal.planning_options.plan_only = False
            future = self.move_group_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.handle_planning_response(f, robot_name))
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Planning failed for {robot_name}: {str(e)}")
            self.planning_in_progress[robot_name] = False
            return False

    def create_loose_constraints(self, robot_name, target_pose):
        constraints = Constraints()
        base_frame = "base_link_robot_1" if robot_name == self.robot1_name else "base_link_robot_2"
        end_effector = self.get_tip_link(robot_name)

        pc = PositionConstraint()
        pc.header.frame_id = base_frame
        pc.link_name = end_effector

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.02]   # start tighter, 2 cm

        pc.constraint_region.primitives.append(sphere)

        region_pose = Pose()
        region_pose.position = target_pose.position
        region_pose.orientation.w = 1.0   # identity, sphere orientation irrelevant
        pc.constraint_region.primitive_poses.append(region_pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = base_frame
        oc.link_name = end_effector
        oc.orientation = target_pose.orientation
        oc.absolute_x_axis_tolerance = 1.0
        oc.absolute_y_axis_tolerance = 1.0
        oc.absolute_z_axis_tolerance = 1.0
        oc.weight = 0.5
        constraints.orientation_constraints.append(oc)

        return constraints

    def get_current_robot_state(self, robot_name):
        """Get current robot state for planning"""
        robot_state = RobotState()
        if self.joint_states:
            robot_state.joint_state = self.joint_states
        robot_state.is_diff = True
        return robot_state

    def handle_planning_response(self, future, robot_name):
        """Handle response from MoveIt planner"""
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().warning(f"Planning goal rejected for {robot_name}")
                self.planning_in_progress[robot_name] = False
                return
                
            self.active_goals[robot_name] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.handle_planning_result(f, robot_name))
                
        except Exception as e:
            self.get_logger().error(f"Error in planning response for {robot_name}: {str(e)}")
            self.planning_in_progress[robot_name] = False

    def handle_planning_result(self, future, robot_name):
        """Handle planning result"""
        try:
            result = future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                if self.debug_param:
                    self.get_logger().info(f"MoveIt plan+execution succeeded for {robot_name}")
            else:
                self.get_logger().warning(f"Planning/execution failed for {robot_name}: {result.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"Error handling planning result for {robot_name}: {str(e)}")
        finally:
            with self.plan_lock:
                self.planning_in_progress[robot_name] = False

    def control_gripper(self, robot_name, is_closed):
        """Control gripper based on hand open/closed state"""
        try:
            open_pos = 0.0
            closed_pos = 0.06
            target_pos = closed_pos if is_closed else open_pos

            # Avoid sending repeated commands
            if self.last_gripper_states.get(robot_name) == target_pos:
                return 

            self.last_gripper_states[robot_name] = target_pos
            
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = target_pos
            goal_msg.command.max_effort = 0.5
            
            client = (self.robot1_gripper_client if robot_name == self.robot1_name
                      else self.robot2_gripper_client)
            
            if client.wait_for_server(timeout_sec=1.0):
                send_goal_future = client.send_goal_async(goal_msg)
                send_goal_future.add_done_callback(
                    lambda future: self.gripper_response_callback(future, robot_name)
                )
                if self.debug_param:
                    self.get_logger().info(f"Gripper command sent to {robot_name}: {'CLOSED' if is_closed else 'OPEN'}")
            else:
                self.get_logger().warning(f"Gripper server not available for {robot_name}")
                
        except Exception as e:
            self.get_logger().error(f"Gripper control failed for {robot_name}: {str(e)}")

    def gripper_response_callback(self, future, robot_name):
        """Handle gripper goal response"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f"Gripper goal rejected for {robot_name}")
        except Exception as e:
            self.get_logger().error(f"Gripper response error for {robot_name}: {str(e)}")

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info("Initiating clean shutdown...")
        
        # Cancel active goals gently
        for robot_name in [self.robot1_name, self.robot2_name]:
            if self.active_goals[robot_name]:
                try:
                    self.active_goals[robot_name].cancel_goal_async()
                except:
                    pass
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandToRobotController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def sigint_handler(sig, frame):
            node.get_logger().info('Ctrl-C detected, initiating clean shutdown...')
            node.destroy_node()
            rclpy.try_shutdown()

        signal.signal(signal.SIGINT, sigint_handler)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()