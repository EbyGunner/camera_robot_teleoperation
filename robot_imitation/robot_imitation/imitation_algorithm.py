#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseStamped
from robot_interfaces.msg import HandState
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, RobotState, OrientationConstraint, MoveItErrorCodes
from tf2_ros import TransformListener, Buffer, TransformException
from control_msgs.action import FollowJointTrajectory, GripperCommand
from tf2_geometry_msgs import do_transform_pose
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState, MultiDOFJointState
from moveit_msgs.action import MoveGroup
import signal
from visualization_msgs.msg import Marker

class HandToRobotController(Node):
    def __init__(self):
        super().__init__('hand_to_robot_controller',
                        parameter_overrides=[])
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG) 

        # Use reentrant callback group for parallel processing
        self.cb_group = ReentrantCallbackGroup()
        
        # Initialize all components
        self.initialize_parameters()
        self.initialize_subscribers()
        self.initialize_tf()
        self.initialize_services()  
        self.initialize_gripper_actions()
        self.initialize_action_clients()
            
        self.get_logger().info("Hand to Robot Controller initialized")

    def initialize_parameters(self):

        self.declare_parameter('debug', False)
        self.debug_param = self.get_parameter('debug').get_parameter_value().bool_value

        self.declare_parameter('robot1_name', 'robot_one')
        self.robot1_name = self.get_parameter('robot1_name').get_parameter_value()
        self.declare_parameter('robot2_name', False)
        self.robot2_name = self.get_parameter('robot2_name').get_parameter_value()

        self.declare_parameter('planning_time', 5.0)
        self.planning_time = self.get_parameter('planning_time').get_parameter_value()
        self.declare_parameter('velocity_scaling', 0.5)
        self.velocity_scaling = self.get_parameter('velocity_scaling').get_parameter_value()
        self.declare_parameter('acceleration_scaling', 0.5)
        self.acceleration_scaling = self.get_parameter('acceleration_scaling').get_parameter_value()

        self.joint_states = None
        
        self.execution_in_progress = {
            self.robot1_name: False,
            self.robot2_name: False
        }
        self.latest_hand_positions = {
            self.robot1_name: None,
            self.robot2_name: None
        }
        self.planning_in_progress = {
            self.robot1_name: False,
            self.robot2_name: False
        }
        self.last_processed_position = {
            self.robot1_name: None,
            self.robot2_name: None
        }
        self.active_goals = {
            self.robot1_name: None,
            self.robot2_name: None
        }
        self.filtered_positions = {
            self.robot1_name: None,
            self.robot2_name: None
        }
        self.position_threshold = 0.05  # 5cm movement threshold

    def initialize_subscribers(self):
        self.left_hand_sub = self.create_subscription(
            HandState, '/left_hand_state', self.left_hand_callback, 
            10, callback_group=self.cb_group)
        self.right_hand_sub = self.create_subscription(
            HandState, '/right_hand_state', self.right_hand_callback,
            10, callback_group=self.cb_group)
        self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback,
            10, callback_group=self.cb_group)

    def initialize_tf(self):
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def initialize_services(self):

        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
    def initialize_action_clients(self):
        self.robot1_traj_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/manipulator_robot_one_controller/follow_joint_trajectory',
            callback_group=self.cb_group)
        self.robot2_traj_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/manipulator_robot_two_controller/follow_joint_trajectory',
            callback_group=self.cb_group)
        
        self.get_logger().info("Waiting for trajectory action servers...")
        for client, name in [(self.robot1_traj_client, self.robot1_name),
                            (self.robot2_traj_client, self.robot2_name)]:
            if not client.wait_for_server(timeout_sec=20.0):
                self.get_logger().error(f'{name} trajectory server not available!')
                raise RuntimeError(f'{name} trajectory server not available')
        self.get_logger().info("Action servers ready")

        self.get_logger().info("Waiting for MoveGroup action server...")
        
        self.move_group_client = ActionClient(
            self, 
            MoveGroup, 
            "/move_action",
            callback_group=self.cb_group)

        try:
            if not self.move_group_client.wait_for_server(timeout_sec=15.0):
                self.get_logger().warning("MoveGroup action server not available - running in reduced mode")
                self.move_group_client = None
                return False
            self.get_logger().info("Connected to MoveGroup action server")
            return True
        except Exception as e:
            self.get_logger().error(f"Error waiting for MoveGroup: {str(e)}")
            self.move_group_client = None
            return False
        
    def initialize_gripper_actions(self):
        self.robot1_gripper_client = ActionClient(
            self, GripperCommand,
            '/gripper_robot_one_controller/gripper_cmd',
            callback_group=self.cb_group)
        self.robot2_gripper_client = ActionClient(
            self, GripperCommand,
            '/gripper_robot_two_controller/gripper_cmd',
            callback_group=self.cb_group)
        
        self.last_gripper_states = {
            self.robot1_name: 0.0,  # 0.0 = fully open, 0.06 = fully closed
            self.robot2_name: 0.0
        }
        
        self.get_logger().info("Waiting for gripper action servers...")
        for client, name in [(self.robot1_gripper_client, self.robot1_name),
                            (self.robot2_gripper_client, self.robot2_name)]:
            if not client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warning(f'{name} gripper server not available!')

    def joint_state_callback(self, msg):
        self.joint_states = msg

    def left_hand_callback(self, msg):
        if msg.detected:
            if not self.is_valid_position(msg.position):
                self.get_logger().debug("Invalid left hand position received")
                return
            self.process_hand_position(msg.hand_type, msg.position)
            self.control_gripper(msg.hand_type, msg.is_closed)

    def right_hand_callback(self, msg):
        if msg.detected:
            if not self.is_valid_position(msg.position):
                self.get_logger().debug("Invalid right hand position received")
                return
            self.process_hand_position(msg.hand_type, msg.position)
            self.control_gripper(msg.hand_type, msg.is_closed)

    def process_hand_position(self, hand_type, position):
        """Process incoming hand positions with enhanced validation"""

        if self.debug_param:
            self.get_logger().debug(f"Raw {hand_type} hand position: x={position.x:.3f}, y={position.y:.3f}, z={position.z:.3f}")

        try:
            # Skip invalid positions
            if not self.is_valid_position(position):
                self.get_logger().debug("Invalid position - skipping")
                return

            target_robot = self.robot1_name if hand_type == "left" else self.robot2_name

            if self.debug_param:
                self.get_logger().debug(f"Target robot: {target_robot}")

            # Store the position
            self.latest_hand_positions[target_robot] = position

            if self.debug_param:
                self.get_logger().debug(f"Stored position for {target_robot}: {position}")

            
            if self.debug_param:
                self.get_logger().debug(
                    f"Processed {hand_type} hand: "
                    f"Raw({position.x:.3f},{position.y:.3f},{position.z:.3f}) → "
                )

            # Only proceed if robot is available
            if not (self.planning_in_progress[target_robot] or 
                    self.execution_in_progress[target_robot]):
                self.process_new_position(target_robot)
                
        except Exception as e:
            self.get_logger().error(f"Error processing {hand_type} hand: {str(e)}")
        
    def process_new_position(self, robot_name):
        current_pos = self.latest_hand_positions[robot_name]
        if current_pos is None:
            self.get_logger().debug(f"No current position for {robot_name}")
            return
        
        if self.debug_param:
            self.get_logger().info(f"Processing new position for {robot_name}: {current_pos}")
        
        # Create a Point message from the position
        target_point = Point()
        target_point.x = current_pos.x
        target_point.y = current_pos.y
        target_point.z = current_pos.z
        
        # Check if movement is significant enough
        last_pos = self.last_processed_position[robot_name]
        if last_pos is not None:
            dx = abs(target_point.x - last_pos.x)
            dy = abs(target_point.y - last_pos.y)
            dz = abs(target_point.z - last_pos.z)
            
            if (dx < self.position_threshold and 
                dy < self.position_threshold and 
                dz < self.position_threshold):
                return 
        
        # Mark as planning in progress
        self.planning_in_progress[robot_name] = True
        
        # Transform and send to MoveIt
        target_pose = self.transform_hand_to_robot_frame(target_point, robot_name)
        if target_pose:
            self.send_to_moveit(robot_name, target_pose)
        
        # Update last processed position
        self.last_processed_position[robot_name] = target_point

    def cancel_active_goal(self, robot_name):
        if robot_name in self.active_goals:
            goal_handle = self.active_goals[robot_name]
            if robot_name in self.active_goals:
                self.get_logger().info(f"Cancelling active goal for {robot_name}")
                goal_handle.cancel_goal_async()
            del self.active_goals[robot_name]   

    def publish_target_marker(self, position, robot_name, frame=None):
        marker = Marker()
        if frame:
            marker.header.frame_id = frame
        else:
            marker.header.frame_id = "world"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = 1.0 if robot_name == self.robot1_name else 0.0
        marker.color.g = 0.0
        marker.color.b = 0.0 if robot_name == self.robot1_name else 1.0
        marker.pose.position = position.position
        
        if not hasattr(self, 'marker_pub'):
            self.marker_pub = self.create_publisher(Marker, 'target_markers', 10)
        self.marker_pub.publish(marker)
        self.get_logger().info(f"Displaying Target pose: {position.position}")
        
    def transform_hand_to_robot_frame(self, hand_position, robot_name):
        try:
            if self.move_group_client is None:
                self.get_logger().warning("MoveGroup not available")
                return False

            # 1. Get current gripper pose in world frame
            gripper_frame = "gripper_base_robot_1" if robot_name == 'robot_one' else "gripper_base_robot_2"
            try:
                # Lookup transform from world to gripper
                tf = self.tf_buffer.lookup_transform("world", gripper_frame, 
                                                rclpy.time.Time(),
                                                timeout=rclpy.duration.Duration(seconds=1.0))
                current_pose = Pose()
                current_pose.position.x = tf.transform.translation.x
                current_pose.position.y = tf.transform.translation.y
                current_pose.position.z = tf.transform.translation.z
                current_pose.orientation = tf.transform.rotation
            except TransformException as e:
                self.get_logger().error(f"Could not get gripper pose in world frame: {str(e)}")
                return False

            # 2. Add deltas to current position (keeping orientation the same)
            target_pose = Pose()
            target_pose.position.x = current_pose.position.x + hand_position.x
            target_pose.position.y = current_pose.position.y + hand_position.y
            target_pose.position.z = current_pose.position.z + hand_position.z
            target_pose.orientation = current_pose.orientation  # Keep same orientation

            return target_pose

        except TransformException as e:
            self.get_logger().warning(f"Transform failed: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"Transform error: {str(e)}")
            return None
        
    def get_filtered_joint_state(self, robot_name):
        """Returns a JointState filtered for the specified robot, or None if timeout.
        Includes both movable and fixed joints for the specified robot."""
        start_time = self.get_clock().now()
        
        # Wait for joint states with timeout
        while (not self.joint_states or not self.joint_states.name or 
            len(self.joint_states.name) == 0) and \
            (self.get_clock().now() - start_time).nanoseconds < 1e9 * self.joint_state_timeout:
            self.get_logger().info("Waiting for joint states...", throttle_duration_sec=1)
            rclpy.spin_once(self, timeout_sec=0.1)

        if not (self.joint_states and self.joint_states.name):
            self.get_logger().error("Joint states not received after timeout")
            return None

        target_suffix = '_robot_1' if robot_name == 'robot_one' else '_robot_2'
        filtered_joint_state = JointState()
        filtered_joint_state.header = self.joint_states.header

        for i, name in enumerate(self.joint_states.name):
            if name.endswith(target_suffix):
                filtered_joint_state.name.append(name)
                filtered_joint_state.position.append(self.joint_states.position[i])
                
                # Handle optional fields
                if self.joint_states.velocity and i < len(self.joint_states.velocity):
                    filtered_joint_state.velocity.append(self.joint_states.velocity[i])
                else:
                    # For fixed joints or if velocity not provided
                    filtered_joint_state.velocity.append(0.0)
                    
                if self.joint_states.effort and i < len(self.joint_states.effort):
                    filtered_joint_state.effort.append(self.joint_states.effort[i])
                else:
                    # For fixed joints or if effort not provided
                    filtered_joint_state.effort.append(0.0)

        return filtered_joint_state

    def get_current_robot_state(self, robot_name):
        """Returns a complete RobotState for MoveIt planning, or None on failure."""
        filtered_joint_state = self.get_filtered_joint_state(robot_name)
        if filtered_joint_state is None:
            return None

        robot_state = RobotState()
        robot_state.joint_state = filtered_joint_state
        robot_state.multi_dof_joint_state = MultiDOFJointState()
        robot_state.attached_collision_objects = []
        robot_state.is_diff = False
        return robot_state
                
    def transform_pose(self, position, current_frame, transform_frame):
        try:

            target_pose_stamped = PoseStamped()
            target_pose_stamped.header.frame_id = current_frame
            target_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            target_pose_stamped.pose = position

            # Use TF2 to transform the pose
            transform = self.tf_buffer.lookup_transform(
                transform_frame,
                current_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            transformed_pose = do_transform_pose(target_pose_stamped.pose, transform)

            if self.debug_param:
                self.get_logger().debug(f"transformed orientation: {transformed_pose}")

            return transformed_pose

        except Exception as tf_err:
            self.get_logger().error(f"TF transform to base frame failed: {tf_err}")
            return False
        
    def send_to_moveit(self, robot_name, target_pose):
        try:

            # 1. Validate position
            if not self.is_valid_position(target_pose.position):
                self.get_logger().warning(f"Invalid target position for {robot_name}")
                return False
            
            frame = "base_link_robot_1" if robot_name == "robot_one" else "base_link_robot_2"

            # frame = "base_link"
            
            self.transform_pose(target_pose, "world", frame)
            
            if self.debug_param:
                self.publish_target_marker(target_pose, robot_name)

            # 3. Prepare constraints with relaxed tolerances
            constraints = Constraints()
            
            # Position constraint
            pc = PositionConstraint()
            pc.header.frame_id = frame
            pc.header.stamp = self.get_clock().now().to_msg()
            pc.link_name = "gripper_base_robot_1" if robot_name == "robot_one" else "gripper_base_robot_2"
            pc.constraint_region.primitives.append(SolidPrimitive(
                type=SolidPrimitive.SPHERE,
                dimensions=[0.05]  # Increased tolerance
            ))
            pc.constraint_region.primitive_poses.append(target_pose)
            pc.weight = 1.0
            constraints.position_constraints.append(pc)
            
            # Orientation constraint (more relaxed)
            oc = OrientationConstraint()
            oc.header.frame_id = frame
            oc.link_name = "gripper_base_robot_1" if robot_name == "robot_one" else "gripper_base_robot_2"
            oc.orientation = target_pose.orientation
            oc.absolute_x_axis_tolerance = 0.5  # Increased tolerance
            oc.absolute_y_axis_tolerance = 0.5
            oc.absolute_z_axis_tolerance = 1.0  # Most relaxed about roll
            oc.weight = 0.5  # Lower priority than position
            constraints.orientation_constraints.append(oc)

            # 4. Create planning request
            request = MotionPlanRequest()
            request.planner_id = "RRTConnect"  # Basic reliable planner
            request.group_name = f"manipulator_{robot_name}"
            request.start_state = self.get_current_robot_state(robot_name)
            request.goal_constraints = [constraints]
            request.allowed_planning_time = 10.0  # Increased time
            request.max_velocity_scaling_factor = 0.3  # Slower execution
            request.max_acceleration_scaling_factor = 0.3
            
            # 5. Send to MoveIt
            goal = MoveGroup.Goal()
            goal.request = request
            future = self.move_group_client.send_goal_async(goal)
            future.add_done_callback(lambda f: self.handle_moveit_response(f, robot_name))
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Planning failed: {str(e)}")
            return False

    def handle_moveit_response(self, future, robot_name):
        try:
            goal_handle = future.result()
            if goal_handle is None:
                self.get_logger().error("Goal handle is None")
                return
                
            if not goal_handle.accepted:
                self.get_logger().warning("Goal rejected by MoveIt")
                return
                
            self.get_logger().info("Goal accepted by MoveIt")
            self.active_goals[robot_name] = goal_handle
            
            # Get result callback
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda future: self.handle_moveit_result(future, robot_name))
                
        except Exception as e:
            self.get_logger().error(f"Error in handle_moveit_response: {str(e)}")

    def is_valid_position(self, position):
        """Check if position is valid and within workspace bounds"""
        # Check for zero position
        if abs(position.x) < 0.001 and abs(position.y) < 0.001 and abs(position.z) < 0.001:
            self.get_logger().debug("Invalid zero position received")
            return False
            
        return True

    def handle_moveit_result(self, future, robot_name):
        try:
            result = future.result().result
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f"Planning succeeded for {robot_name}")
            else:
                self.get_logger().warning(
                    f"Planning failed for {robot_name}: {result.error_code.val} - {result.error_code}")
        finally:
            self.planning_in_progress[robot_name] = False
            self.process_new_position(robot_name)

    def planning_done_callback(self, future, robot_name):
        try:
            response = future.result()
            if response.motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f"Successfully planned path for {robot_name}")
                self.execute_trajectory(robot_name, response.motion_plan_response.trajectory)
            else:
                self.get_logger().warning(
                    f"Planning failed for {robot_name} with error code: "
                    f"{response.motion_plan_response.error_code.val}")
        except Exception as e:
            self.get_logger().error(f"Planning service call failed: {str(e)}")

    def execute_trajectory(self, robot_name, trajectory):
        if self.execution_in_progress[robot_name]:
            self.get_logger().info(f"Execution in progress for {robot_name}, ignoring new command")
            return

        
        try:
            client = self.robot1_traj_client if robot_name == self.robot1_name else self.robot2_traj_client
            
            if not client.wait_for_server(timeout_sec=1.0):
                self.get_logger().error(f"{robot_name} trajectory server not ready")
                return

            self.execution_in_progress[robot_name] = True  
            
            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = trajectory.joint_trajectory

            # Apply scaling
            for point in goal_msg.trajectory.points:
                if point.velocities:
                    point.velocities = [v * self.velocity_scaling for v in point.velocities]
                if point.time_from_start:
                    point.time_from_start.sec = int(point.time_from_start.sec / self.velocity_scaling)

            send_goal_future = client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(
                lambda f: self.goal_accepted_callback(f, robot_name))
                
        except Exception as e:
            self.get_logger().error(f"Execution failed: {str(e)}")
            self.execution_in_progress[robot_name] = False

    def goal_accepted_callback(self, future, robot_name):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f"Goal rejected for {robot_name}")
                return
                
            self.active_goals[robot_name] = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.trajectory_result_callback(f, robot_name))
                
        except Exception as e:
            self.get_logger().error(f"Goal acceptance error: {str(e)}")

    def trajectory_result_callback(self, future, robot_name):
        try:
            result = future.result().result
            if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
                self.get_logger().info(f"{robot_name} execution succeeded")
            else:
                self.get_logger().warning(f"{robot_name} execution failed")
                
        except Exception as e:
            self.get_logger().error(f"Result handling error: {str(e)}")
        finally:
            # Clear execution flag and process any new positions
            self.execution_in_progress[robot_name] = False
            self.process_new_position(robot_name)

    def control_gripper(self, hand_type, is_closed):
        try:
            if hand_type == "left":
                robot_name = self.robot1_name
            else:
                robot_name = self.robot2_name

            # Define positions
            open_pos = 0.0
            closed_pos = 0.06
            
            # Choose position based on is_closed
            target_pos = closed_pos if is_closed else open_pos

            # Avoid sending repeated commands if state unchanged
            if self.last_gripper_states.get(robot_name, None) == target_pos:
                return 

            self.last_gripper_states[robot_name] = target_pos
            
            medium_effort = 0.5
            
            # Create GripperCommand goal message
            goal_msg = GripperCommand.Goal()
            goal_msg.command.position = target_pos
            goal_msg.command.max_effort = medium_effort
            
            # Choose correct gripper client
            client = (self.robot1_gripper_client if robot_name == self.robot1_name
                      else self.robot2_gripper_client)
            
            # Send goal asynchronously
            send_goal_future = client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(
                lambda future: self.gripper_response_callback(future, robot_name)
            )
        except Exception as e:
            self.get_logger().error(f"Gripper control failed: {str(e)}")

    def gripper_response_callback(self, future, robot_name):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warning(f"Gripper goal rejected for {robot_name}")
                return
            self.get_logger().info(f"Gripper goal accepted for {robot_name}")
            result_future = goal_handle.get_result_async()
            if self.debug_param:
                result_future.add_done_callback(
                    lambda f: self.gripper_result_callback(f, robot_name)
                )
        except Exception as e:
            self.get_logger().error(f"Gripper response callback error for {robot_name}: {str(e)}")

    def gripper_result_callback(self, future, robot_name):
        try:
            result = future.result().result
            self.get_logger().info(f"Gripper action completed for {robot_name} with result: {result}")
        except Exception as e:
            self.get_logger().error(f"Gripper result error for {robot_name}: {str(e)}")

    def emergency_stop(self):
        """Immediately stop all robot motion"""
        self.get_logger().info("Initiating emergency stop")
        for robot_name in [self.robot1_name, self.robot2_name]:
            self.cancel_active_goal(robot_name)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = HandToRobotController()
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        def sigint_handler(sig, frame):
            node.get_logger().info('Ctrl-C detected, shutting down...')
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