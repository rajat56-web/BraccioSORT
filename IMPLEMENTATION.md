# BraccioSort Autonomous Pick-and-Place System


## 1. HSV Detection Implementation

Object detection uses HSV colour segmentation in `yolo_detector_node.py` rather than a neural-network model. The node subscribes to `/camera/image_raw` and publishes `vision_msgs/Detection2DArray` on `/detections`.

**BGR → HSV conversion:**
```python
hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
```

**Red mask (dual-range — hue wraps at 0°/180° in OpenCV HSV):**
```python
red_mask_1 = cv2.inRange(hsv, [0,   100, 80],  [10,  255, 255])
red_mask_2 = cv2.inRange(hsv, [160, 100, 80],  [180, 255, 255])
red_mask   = cv2.bitwise_or(red_mask_1, red_mask_2)
```

**Blue mask (single range):**
```python
blue_mask = cv2.inRange(hsv, [100, 100, 50], [130, 255, 255])
```

**Morphological cleaning:**  
Closing (fills holes inside objects) followed by Opening (removes isolated noise pixels), using a 5×5 rectangular structuring element.

---

## 2. Contour Filtering

After producing clean masks, external contours are extracted with `cv2.findContours`. Each contour passes through a five-stage validation pipeline:

| Filter | Threshold | Rejects |
|--------|-----------|---------|
| Area (min) | 400 px² | image noise |
| Area (max) | 4 000 px² | containers and table surface |
| Bounding box size | 60 px | large rectangular regions |
| Vertical position | top 10% of image | background / ceiling |
| Aspect ratio | > 3.0 | elongated non-cube shapes |

A confidence score is computed as `min(area / 5000, 1.0)`. Detections below the configurable `confidence_threshold` (default 0.5) are discarded.

**Why area-based filtering works here:**  
The cubes (2 cm × 2 cm) appear as ~36×36 px blobs from 59 cm overhead, while the containers (wider and brighter) appear as ≥ 150 px blobs. The gap is large enough for a simple area threshold to separate them reliably.

---

## 3. Pixel-to-World Coordinate Conversion

Implemented in `BraccioMoveItSortingController._pixel_to_world()`.

**Camera geometry (from URDF `camera_joint`):**
- Position: `xyz = (0.20, 0.00, 0.59)` relative to `base_link`
- Orientation: `rpy = (0, π/2, π)` — pitch 90° points the camera straight down; yaw π flips both image axes

**Known cube surface height:** `z = 0.28 m`  
(table Z = 0.255 m + table thickness 0.005 m + cube half-height 0.01 m + cube half-height 0.01 m = 0.28 m)

**Pinhole back-projection:**
```python
depth   = cam_wz - cube_z          # = 0.59 - 0.28 = 0.31 m

x_cam   = (v - cy) / fy            # image row    → camera X
y_cam   = (u - cx) / fx            # image column → camera Y

world_x = cam_wx + x_cam * depth   # cam_wx = 0.20
world_y = cam_wy + y_cam * depth   # cam_wy = 0.00
world_z = cube_z
```

**Axis mapping rationale:**  
With pitch = π/2, the camera looks straight down (−Z world). The optical frame has image columns (+u) mapping to world +Y and image rows (+v) mapping to world +X. After yaw = π, the formula uses addition (not negation) — verified by checking that the red cube at pixel (389, 240) maps to world (0.20, 0.039), which matches the world file pose `<pose>0.20 0.04 0.27</pose>`.

**Stabilisation:**  
A rolling median filter over the last 10 frames is applied before the world position is accepted. A minimum of 3 consistent readings is required, followed by 3 seconds of stable detections before the sort sequence begins. This prevents the arm from targeting transient noise detections.

---

## 4. MoveIt IK Request Construction

Implemented in `BraccioMoveItSortingController._compute_ik()`.

```python
request = GetPositionIK.Request()
request.ik_request.group_name       = 'arm'
request.ik_request.avoid_collisions = False   # position-only mode
request.ik_request.timeout.sec      = 5

pose = PoseStamped()
pose.header.frame_id      = 'world'
pose.pose.position.x      = target['x']
pose.pose.position.y      = target['y']
pose.pose.position.z      = target['z']
pose.pose.orientation.w   = 1.0       # identity — position-only IK
```

**IK solver: TRAC-IK**  
`kinematics.yaml` uses `trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin` with `position_only_ik: true`. This was necessary because the Braccio's 5-DOF chain cannot satisfy arbitrary end-effector orientations — requesting a fixed "gripper pointing down" quaternion (0, 0.707, 0, 0.707) consistently returned error code −31 (NO_IK_SOLUTION). Position-only mode decouples orientation from the IK solve, giving TRAC-IK freedom to find valid joint configurations.

**Seeding strategy:**  
The `base_joint` seed is estimated from `atan2(target_y, target_x)` to guide TRAC-IK toward geometrically sensible solutions. Remaining joints are warm-started from the previous successful IK solution (`last_ik_solution`), falling back to `[base_seed, 2.7, 2.5, 2.8, 2.6]` on the first call.

**Valid IK height range (verified empirically):**

| Z height | IK result |
|----------|-----------|
| 0.50 m   | FAIL (−31) |
| 0.40 m   | FAIL (−31) |
| 0.35 m   | SUCCESS (val=1) |
| 0.30 m   | SUCCESS (val=1) |

The arm reach boundary lies near z = 0.37 m for x = 0.15–0.20 m. The pick sequence therefore uses approach z = 0.35 m and grasp z = 0.31 m.

---

## 5. Drop Pose Calibration

Container poses are read directly from `braccio_sorting.world`:

```
red_container:  <pose>0.05  0.20 0.03 0 0 0</pose>
blue_container: <pose>0.05 -0.20 0.03 0 0 0</pose>
```

The `_place` method uses hardcoded joint configurations (not IK) for the drop positions, because the containers are fixed and IK for those lateral positions gave inconsistent results:

```python
drop_joints = {
    'red':  [3.5, 2.5, 2.5, 2.8, 2.6],   # base rotates left  → +Y container
    'blue': [1.5, 2.5, 2.5, 2.8, 2.6],   # base rotates right → −Y container
}
```

These values were determined by:
1. Testing IK for the container world positions at z = 0.12 m (confirmed `val=1`)
2. Observing which `base_joint` direction correctly pointed the arm at each container
3. Tuning `shoulder_joint` and `elbow_joint` to extend the arm far enough to drop above the container

The `gripper_joint` range is 2.6 (open) to 3.85 (closed). Open = 3.85, Close = 2.65 in the final controller.

---

## 6. Sorting Sequence and Failure Handling

The sort sequence in `_sort_sequence()` follows this structure:

```
home → scan → [for each detected object]:
    open_gripper
    → move_to_approach (z=0.35)
    → move_to_grasp    (z=0.31)
    → close_gripper
    → lift_to_approach (z=0.35)
    → move_to_drop_joints
    → open_gripper
    → scan
→ home
→ print DONE
```

**Failure handling at each step:**

| Failure point | Behaviour |
|---------------|-----------|
| IK returns None | Log warn, skip this object, return to scan |
| Trajectory rejected | Log warn, skip this object, return to scan |
| Trajectory timeout | Log warn, skip this object, return to scan |
| Pick fails | Skip place, increment fail counter, continue to next object |
| Place fails | Increment fail counter, continue to next object |

The sequence never aborts entirely — every failure is caught and the arm returns to the scan position before attempting the next object.

**Completion flag:**  
A `_completed` boolean is set in the `finally` block of `_sort_worker`. Both `_check_and_start` and `_detection_cb` check this flag and return immediately if True, preventing the system from re-sorting objects that have already been placed.

---

## 7. Assumptions

- **Static camera:** The camera is rigidly attached to `base_link`. Its world position `(0.20, 0.00, 0.59)` is taken directly from the URDF and treated as ground truth.
- **Known cube height:** All cubes rest on the table surface. `cube_world_z = 0.28 m` is derived from the world file geometry and remains constant throughout a run.
- **Single object per colour:** The controller stores one position per colour class. If two red cubes appear, only the median of all red detections is tracked.
- **Fixed container positions:** Drop positions are hardcoded from the world file. The system would fail if containers were moved.
- **Gazebo physics approximation:** The gripper closes to a fixed joint angle rather than using force feedback. Grasp success depends on the gripper being correctly centred over the cube, not on contact sensing.
- **No collision objects in MoveIt planning scene:** `avoid_collisions = False` in the IK request. The table and containers are not added to the MoveIt planning scene.

---

## 8. Limitations

- **Gripper slip:** The Braccio gripper uses a mimic joint with no force sensing. If the approach offset is slightly off, the cube is pushed rather than grasped. The approach is offset by `x - 0.04 m` to mitigate this but it remains sensitive to exact cube placement.
- **5-DOF IK reachability:** The Braccio has no wrist roll degree of freedom that is free during pick. The `position_only_ik: true` setting compensates, but reachable positions are constrained to a narrow band around z = 0.28–0.35 m for forward-facing targets.
- **HSV sensitivity:** The HSV thresholds were tuned for the specific lighting in `braccio_sorting.world`. A different world file with different ambient light or object colours would require retuning.
- **No re-detection after pick:** The controller captures a single stable detection snapshot before sorting begins. If a cube moves during the sequence (e.g., knocked by the arm), the stored position becomes stale.
- **Sequential sorting only:** Objects are sorted one at a time. A red cube must be fully placed before the blue cube is attempted.
- **Docker/container resource constraint:** RViz consumes approximately 2 GB RAM in this environment. Running Gazebo + MoveIt + RViz simultaneously causes Gazebo to crash. A production deployment on native hardware would not face this limitation.

---

## Key Debugging Decisions

Several non-obvious choices were made during implementation:

**CycloneDDS over FastDDS:**  
The Docker container's shared memory is insufficient for FastDDS's default participant count. Switching to CycloneDDS (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`) eliminated all `RTPS_TRANSPORT_SHM` errors and daemon connection timeouts.

**`braccio.launch.py` as base, not `braccio_gazebo.launch.py`:**  
`braccio.launch.py` contains two critical settings absent from the scaffolded `braccio_gazebo.launch.py`:
1. `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/jazzy/lib` — without this, the `gz_ros2_control` plugin silently fails to load and `controller_manager` never starts.
2. A static TF publisher for `world → base_link` — without this, every MoveIt IK call returns `FRAME_TRANSFORM_FAILURE`.

**Position-only IK:**  
Requesting a fixed downward-pointing end-effector orientation (quaternion 0, 0.707, 0, 0.707) returned error −31 at all tested positions. Switching to `position_only_ik: true` in `kinematics.yaml` and passing `orientation.w = 1.0` in the IK request resolved this immediately.

**`controllers.yaml` path via `$(find ...)`:**  
The URDF plugin block uses `$(find braccio_description)/config/braccio_controllers.yaml`. Gazebo Harmonic resolves this via xacro at spawn time — the file must exist at the exact installed path or the plugin silently drops the controller configuration and `controller_manager` never initialises.
