import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PositionIKRequest, RobotState
from moveit_msgs.srv import GetPositionIK
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import JointState


class BraccioMoveItSortingController(Node):
    """MoveIt-based sorting controller with real IK and live pixel-to-world."""

    def __init__(self):
        super().__init__('braccio_moveit_sorting_controller')

        # ------------------------------------------------------- parameters
        self.declare_parameter('min_confidence', 0.5)
        self.declare_parameter('detection_stable_time', 3.0)
        self.declare_parameter('auto_start', True)
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('planning_frame', 'world')

        # Camera intrinsics — derived from URDF HFOV=1.047 rad, 640x480.
        self.declare_parameter('camera_fx', 554.4)
        self.declare_parameter('camera_fy', 554.4)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)

        # Camera world pose (from URDF camera_joint xyz).
        self.declare_parameter('camera_world_x', 0.20)
        self.declare_parameter('camera_world_y', 0.00)
        self.declare_parameter('camera_world_z', 0.59)

        # Known height of the surface the cubes rest on (world Z).
        self.declare_parameter('cube_world_z', 0.28)

        self.min_confidence  = self.get_parameter('min_confidence').value
        self.stable_time     = self.get_parameter('detection_stable_time').value
        self.auto_start      = self.get_parameter('auto_start').value
        self.planning_group  = self.get_parameter('planning_group').value
        self.planning_frame  = self.get_parameter('planning_frame').value

        self.cam_fx = self.get_parameter('camera_fx').value
        self.cam_fy = self.get_parameter('camera_fy').value
        self.cam_cx = self.get_parameter('camera_cx').value
        self.cam_cy = self.get_parameter('camera_cy').value
        self.cam_wx = self.get_parameter('camera_world_x').value
        self.cam_wy = self.get_parameter('camera_world_y').value
        self.cam_wz = self.get_parameter('camera_world_z').value
        self.cube_z = self.get_parameter('cube_world_z').value

        self.get_logger().info(
            f'Camera params: fx={self.cam_fx:.1f} fy={self.cam_fy:.1f} '
            f'cx={self.cam_cx:.1f} cy={self.cam_cy:.1f} | '
            f'cam_world=({self.cam_wx},{self.cam_wy},{self.cam_wz}) '
            f'cube_z={self.cube_z}'
        )

        # ---------------------------------------------------------- clients
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self.gripper_client = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory',
        )
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        self.get_logger().info('Waiting for controllers and MoveIt services...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        while not self.ik_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /compute_ik...')
        self.get_logger().info('MoveIt and controllers ready.')

        # ------------------------------------------------------- joint info
        self.arm_joint_names = [
            'base_joint',
            'shoulder_joint',
            'elbow_joint',
            'wrist_pitch_joint',
            'wrist_roll_joint',
        ]
        self.gripper_joint_names = ['gripper_joint']

        # ------------------------------------------------------ joint state
        self.current_joint_state = None
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10,
        )

        # ---------------------------------------------------------- presets
        # Joint angles [base, shoulder, elbow, wrist_pitch, wrist_roll] in radians.
        # Drop positions calibrated for the sorting world layout:
        #   drop_red  -> red container (left side,  base ~1.8 rad)
        #   drop_blue -> blue container (right side, base ~3.2 rad)
        self.named_positions = {
            'home':       [2.5,   2.8,   2.8,   2.8,   2.6],
            'scan':       [2.5,   2.3,   2.0,   3.2,   2.6],
            'drop_red':   [1.8,   2.5,   2.5,   2.8,   2.6],
            'drop_blue':  [3.2,   2.5,   2.5,   2.8,   2.6],
        }
        self.gripper_open   = 3.2
        self.gripper_closed = 2.55

        # ------------------------------------------------------- detection state
        self.detected_objects       = []
        self.first_detection_time   = None
        self.is_sorting             = False
        self._completed             = False
        self._sort_lock             = threading.Lock()
        self.last_ik_solution       = None
        self._pos_history: dict = {'red': [], 'blue': []}
        self._pos_history_len = 10   # median window (frames)

        self.create_subscription(
            Detection2DArray, '/detections',
            self._detection_cb, 10,
        )

        if self.auto_start:
            self.create_timer(2.0, self._check_and_start)

        self.get_logger().info('Braccio MoveIt Sorting Controller ready.')

    # ---------------------------------------------------------------- callbacks
    def _joint_state_cb(self, msg):
        self.current_joint_state = msg

    # ================================================================
    # IMPLEMENTATION: Part 1 — Pixel-to-World Projection
    # ================================================================
    def _pixel_to_world(self, u, v):
        """
        Convert pixel centroid (u, v) to world XY position.

        The camera looks straight down from world position
        (self.cam_wx, self.cam_wy, self.cam_wz).

        The camera's optical axes map to world axes (from URDF rpy="0 pi/2 pi"):
            image columns (u) -> world Y
            image rows    (v) -> world X

        Standard pinhole back-projection for a surface at known world Z:
            depth   = cam_wz - cube_z
            world_x = cam_wx + (v - cam_cy) * depth / cam_fy
            world_y = cam_wy + (u - cam_cx) * depth / cam_fx

        Args:
            u (float): pixel column (x) of the bounding box centre
            v (float): pixel row    (y) of the bounding box centre

        Returns:
            dict with keys 'x', 'y', 'z' in metres (world frame)
        """
        # Perpendicular distance from camera optical centre to cube surface
        depth = self.cam_wz - self.cube_z

        # Back-project: image column u -> world Y, image row v -> world X
        world_x = self.cam_wx + (v - self.cam_cy) * depth / self.cam_fy
        world_y = self.cam_wy + (u - self.cam_cx) * depth / self.cam_fx

        return {'x': world_x, 'y': world_y, 'z': self.cube_z}

    def _detection_cb(self, msg):
        if self._completed:
            return
        if self.is_sorting:
            return
        seen_colors = set()
        for det in msg.detections:
            if not det.results:
                continue
            h = det.results[0]
            if h.hypothesis.score < self.min_confidence:
                continue
            color_id = h.hypothesis.class_id   # 'red_cube' or 'blue_cube'
            color = None
            if 'red' in color_id:
                color = 'red'
            elif 'blue' in color_id:
                color = 'blue'
            if color is None:
                continue

            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
            pos = self._pixel_to_world(u, v)

            # Append to rolling history
            hist = self._pos_history[color]
            hist.append((pos['x'], pos['y']))
            if len(hist) > self._pos_history_len:
                hist.pop(0)

            seen_colors.add((color, color_id, h.hypothesis.score))

        valid = []
        for color, color_id, score in seen_colors:
            hist = self._pos_history[color]
            if len(hist) < 3:          # need at least 3 readings
                continue
            xs = sorted(p[0] for p in hist)
            ys = sorted(p[1] for p in hist)
            mid = len(xs) // 2
            med_x = xs[mid]
            med_y = ys[mid]
            pos = {'x': round(med_x, 4), 'y': round(med_y, 4), 'z': self.cube_z}
            self.get_logger().info(
                f'Stable {color_id}: median world ({pos["x"]:.4f}, {pos["y"]:.4f}) '
                f'over {len(hist)} frames'
            )
            valid.append({
                'class_id': color_id,
                'position': pos,
                'confidence': score,
            })

        if valid:
            self.detected_objects = valid
            if self.first_detection_time is None:
                self.first_detection_time = time.time()
                self.get_logger().info(f'Stable detection started - {len(valid)} objects')

    def _check_and_start(self):
        """Timer callback - fires the sort sequence on a worker thread."""
        if self._completed:
            return
        if self.is_sorting or not self.detected_objects:
            return
        if self.first_detection_time is None:
            return
        if (time.time() - self.first_detection_time) < self.stable_time:
            return

        with self._sort_lock:
            if self.is_sorting:
                return
            self.is_sorting = True

        objects_to_sort = list(self.detected_objects)
        thread = threading.Thread(
            target=self._sort_worker, args=(objects_to_sort,), daemon=True,
        )
        thread.start()

    def _sort_worker(self, objects):
        try:
            self.get_logger().info('Starting sorting sequence (worker thread)')
            self._sort_sequence(objects)
        except Exception as exc:
            self.get_logger().error(f'Sort sequence crashed: {exc}')
        finally:
            self.detected_objects       = []
            self.first_detection_time   = None
            self.is_sorting             = False
            self._completed             = True 
            self.get_logger().info('[92m========== DONE — Mission Complete ==========[0m')

    # ---------------------------------------------------------------- IK
    # ================================================================
    # IMPLEMENTATION: Part 2 — Inverse Kinematics via MoveIt /compute_ik
    # ================================================================
    def _compute_ik(self, target):
        """Call /compute_ik service synchronously from worker thread."""
        req = GetPositionIK.Request()
        req.ik_request.group_name       = self.planning_group
        req.ik_request.avoid_collisions = False

        ps = PoseStamped()
        ps.header.frame_id  = self.planning_frame
        ps.header.stamp     = self.get_clock().now().to_msg()
        ps.pose.position.x  = float(target['x'])
        ps.pose.position.y  = float(target['y'])
        ps.pose.position.z  = float(target['z'])
        ps.pose.orientation.w = 1.0

        req.ik_request.pose_stamped   = ps
        req.ik_request.timeout.sec    = 5

        # ── Step A: Seed the IK solver with a directional estimate ─────
        # TRAC-IK is a numerical solver; a good seed dramatically improves
        # success rate. We estimate the base angle from atan2 of the target.

        angle = math.atan2(target['y'], target['x'])
        base_target = 2.5 + angle   # 2.5 rad is the robot's zero-heading offset
        # Clamp to URDF joint limits [0.05, 5.0] with some safety margin
        base_target = max(0.2, min(4.8, base_target))

        seed = JointState()
        seed.name = self.arm_joint_names
        if self.last_ik_solution is not None:
            # Warm start: reuse previous solution, only update base estimate
            seed_positions = list(self.last_ik_solution)
            seed_positions[0] = base_target
        else:
            # Cold start: neutral mid-range configuration
            seed_positions = [base_target, 2.8, 2.8, 2.8, 2.6]
        seed.position = [float(p) for p in seed_positions]
        req.ik_request.robot_state.joint_state = seed

        # ── Step B: Asynchronous service call blocked with threading.Event
        evt = threading.Event()
        future = self.ik_client.call_async(req)
        future.add_done_callback(lambda _f: evt.set())

        if not evt.wait(timeout=20.0):
            self.get_logger().warn('_compute_ik: timed out after 20 s')
            return None

        resp = future.result()

        # ── Step C: Validate and extract joint positions ────────────────
        if resp is None:
            self.get_logger().warn('_compute_ik: received None response')
            return None

        # MoveIt error code 1 == SUCCESS
        if resp.error_code.val != 1:
            self.get_logger().warn(
                f'_compute_ik: IK failed with error_code={resp.error_code.val}'
            )
            return None

        # Extract positions matched by joint name (not array index)
        sol_state = resp.solution.joint_state
        name_to_pos = dict(zip(sol_state.name, sol_state.position))

        try:
            positions = [float(name_to_pos[n]) for n in self.arm_joint_names]
        except KeyError as e:
            self.get_logger().warn(f'_compute_ik: missing joint in solution: {e}')
            return None

        # Cache for warm-starting the next call
        self.last_ik_solution = positions
        self.get_logger().info(
            f'IK solution: {[f"{p:.3f}" for p in positions]}'
        )
        return positions

    def _send_trajectory(self, client, joint_names, positions, duration_sec):
        """Send a FollowJointTrajectory goal and wait for completion."""
        self.get_logger().info(
            f'Trajectory -> {dict(zip(joint_names, [f"{p:.3f}" for p in positions]))} '
            f'over {duration_sec}s'
        )

        point = JointTrajectoryPoint()
        point.positions       = [float(p) for p in positions]
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec - int(duration_sec)) * 1e9),
        )

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names
        goal.trajectory.points      = [point]
        goal.goal_time_tolerance    = Duration(sec=1, nanosec=0)

        send_future = client.send_goal_async(goal)
        send_evt    = threading.Event()
        send_future.add_done_callback(lambda _f: send_evt.set())
        if not send_evt.wait(timeout=5.0):
            self.get_logger().warn('Goal send timed out')
            return False

        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warn(f'Goal REJECTED for joints {joint_names}')
            return False

        result_future = handle.get_result_async()
        result_evt    = threading.Event()
        result_future.add_done_callback(lambda _f: result_evt.set())
        if not result_evt.wait(timeout=duration_sec + 15.0):
            self.get_logger().warn('Trajectory execution timed out')
            return False

        result = result_future.result()
        if result and result.result and result.result.error_code != 0:
            self.get_logger().warn(
                f'Trajectory error {result.result.error_code}: '
                f'{result.result.error_string}'
            )
            return False

        self.get_logger().info(f'Trajectory complete for {joint_names}')
        return True

    def _send_arm(self, positions, duration_sec=3.0):
        return self._send_trajectory(
            self.arm_client, self.arm_joint_names, positions, duration_sec,
        )

    def _send_gripper(self, position, duration_sec=1.0):
        return self._send_trajectory(
            self.gripper_client, self.gripper_joint_names, [position],
            duration_sec,
        )

    def _move_to_pose(self, target, description=''):
        self.get_logger().info(
            f'Moving to {description}: '
            f'x={target["x"]:.4f} y={target["y"]:.4f} z={target["z"]:.4f}'
        )
        positions = self._compute_ik(target)
        if positions is None:
            return False
        return self._send_arm(positions, duration_sec=3.0)

    def _move_named(self, name):
        if name not in self.named_positions:
            self.get_logger().error(f'Unknown named position: {name}')
            return False
        return self._send_arm(self.named_positions[name], duration_sec=2.0)

    # -------------------------------------------------------- pick & place

    def _pick(self, obj, idx):
        color = 'red' if 'red' in obj['class_id'] else 'blue'
        pos   = obj['position']
        self.get_logger().info(
            f'Picking {color} cube #{idx} at '
            f'({pos["x"]:.4f}, {pos["y"]:.4f}, {pos["z"]:.3f})'
        )

        self._send_gripper(self.gripper_open)

        approach = {'x': pos['x'] - 0.09, 'y': pos['y'], 'z': 0.34}
        if not self._move_to_pose(approach, f'{color} approach'):
            return False

        grasp = {'x': pos['x'] - 0.05 , 'y': pos['y'], 'z': pos['z'] - 0.01 }
        if not self._move_to_pose(grasp, f'{color} grasp'):
            return False

        self._send_gripper(self.gripper_closed)
        time.sleep(0.4)   # let gripper close

        return self._move_to_pose(approach, 'lift')

    def _place(self, color):
        self.get_logger().info(f'Placing in {color} container')
        # Use actual world positions from world file
        container_positions = {
            'red':  {'x': 0.05, 'y':  0.17, 'z': 0.16},
            'blue': {'x': 0.05, 'y': -0.17, 'z': 0.16},
        }
        pos = container_positions[color]
        if not self._move_to_pose(pos, f'{color} container'):
            return False
        self._send_gripper(self.gripper_open)
        time.sleep(0.4)
        return True

    # ================================================================
    # IMPLEMENTATION: Part 3 — Pick-and-Place Sorting Loop
    # ================================================================
    def _sort_sequence(self, objects):
        """
        Full autonomous sorting sequence:
          1. Move to 'home', then 'scan' position.
          2. For each detected object:
             - Determine colour (red / blue).
             - Call _pick(obj, idx) -> bool.
             - If pick succeeded, call _place(color) -> bool.
             - If pick failed, log warning and skip.
             - After each object (success or fail), return to 'scan'.
          3. Move back to 'home'.
          4. Log summary of how many red/blue succeeded and failed.

        """
        self.get_logger().info(
            f'Sort sequence starting — {len(objects)} objects to sort'
        )

        # Step 1: Move to home, then scan
        self._move_named('home')
        self._move_named('scan')

        # Counters for the final summary
        results = {'red': {'ok': 0, 'fail': 0}, 'blue': {'ok': 0, 'fail': 0}}

        # Step 2: Process each object
        for idx, obj in enumerate(objects):
            color = 'red' if 'red' in obj['class_id'] else 'blue'

            self.get_logger().info(
                f'[{idx + 1}/{len(objects)}] Processing {color} cube '
                f'@ ({obj["position"]["x"]:.4f}, {obj["position"]["y"]:.4f})'
            )

            # Attempt pick
            if not self._pick(obj, idx):
                self.get_logger().warn(
                    f'Pick FAILED for {color} cube #{idx} — skipping to next'
                )
                results[color]['fail'] += 1
                self._move_named('scan')
                continue

            # Attempt place
            if self._place(color):
                results[color]['ok'] += 1
                self.get_logger().info(
                    f'Successfully sorted {color} cube #{idx} into container'
                )
            else:
                results[color]['fail'] += 1
                self.get_logger().warn(
                    f'Place FAILED for {color} cube #{idx}'
                )

            # Return to scan after each object
            self._move_named('scan')

        # Step 3: Return home
        self._move_named('home')

        # Step 4: Final summary
        self.get_logger().info(
            '=== Sorting complete ===\n'
            f'  Red  cubes: {results["red"]["ok"]} OK, '
            f'{results["red"]["fail"]} failed\n'
            f'  Blue cubes: {results["blue"]["ok"]} OK, '
            f'{results["blue"]["fail"]} failed'
        )

        self.get_logger().info('========== DONE ==========')


def main(args=None):
    rclpy.init(args=args)
    node = BraccioMoveItSortingController()

    executor = MultiThreadedExecutor(num_threads=4)
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