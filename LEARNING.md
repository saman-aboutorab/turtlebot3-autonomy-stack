# TurtleBot3 Autonomy Stack — Learning Notes

> Personal study notes — one entry per step. Written to build a mental model,
> not just working code. Each section explains *why* before *how*.

---

## Table of Contents

- [Stage 1 — Foundation](#stage-1--foundation)
  - [Step 1 — Package Scaffolding](#step-1--package-scaffolding)
  - [Step 2 — URDF Robot Description](#step-2--urdf-robot-description)
  - [Step 3 — Odometry Publisher](#step-3--odometry-publisher)
  - [Step 4 — TF2 Broadcaster](#step-4--tf2-broadcaster)
  - [Step 5 — IMU Republisher](#step-5--imu-republisher)

---

# Stage 1 — Foundation

Hardware: TurtleBot3 Burger (RPi4 + OpenCR + RPLiDAR A1M8 + MPU-9250 IMU)
Branch: `stage-1` | Milestone tag: `v1.0.0`

---

## Step 1 — Package Scaffolding

**Goal:** Create the three ROS2 packages and get `colcon build` to pass before writing any robot logic.

### What is a ROS2 package?

A package is the basic unit of organisation in ROS2. It is a directory containing at minimum:
- `package.xml` — declares the package's identity and dependencies
- `CMakeLists.txt` — tells colcon how to build it

Every node, launch file, and config file lives inside a package. Packages let you declare what other packages you depend on, so `rosdep install` can auto-install them.

### The three packages we created

| Package | Role |
|---|---|
| `tb3_bringup` | Launch-only. No nodes. Wires the other two together via launch files. |
| `tb3_odometry` | Sensor data in → robot pose out. Odometry, TF2, IMU, EKF. |
| `tb3_navigation` | Pose + map → autonomous movement. SLAM, Nav2, waypoint following. |

Separating them means you can restart navigation without restarting odometry, which is useful when tuning Nav2.

### `package.xml` — the manifest

```xml
<package format="3">
  <name>tb3_odometry</name>
  <version>0.1.0</version>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_cmake_python</buildtool_depend>
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  ...
</package>
```

Three dependency types:
- `<buildtool_depend>` — tools needed to *compile* (ament_cmake, ament_cmake_python)
- `<depend>` — libraries needed at both build time and run time (rclpy, sensor_msgs)
- `<exec_depend>` — only needed at run time (used in tb3_bringup which has no compiled code)

### `CMakeLists.txt` — the build script

Two critical lines for Python support:

```cmake
# Makes `import tb3_odometry` work from any Python node
ament_python_install_package(${PROJECT_NAME})

# Makes `ros2 run tb3_odometry some_node.py` work
install(PROGRAMS
  tb3_odometry/some_node.py
  DESTINATION lib/${PROJECT_NAME}
)
```

The `install(PROGRAMS ...)` line copies (or symlinks) the script into `install/tb3_odometry/lib/tb3_odometry/`, which is on the PATH that `ros2 run` searches.

### `colcon build --symlink-install`

`--symlink-install` means instead of copying files into `install/`, colcon creates symlinks back to the source. This means editing a Python file takes effect immediately without rebuilding — essential for fast iteration.

**Important gotcha:** because symlinks inherit the source file's permissions, Python nodes must have `chmod +x` set on the *source* file, not just the installed copy.

### Key mental model

```
workspace/
  src (your packages)  ←── you edit here
  build/               ←── colcon's intermediate files
  install/             ←── what ROS2 actually runs
    tb3_odometry/
      lib/tb3_odometry/  ←── symlinks to your .py files
      share/tb3_odometry/ ←── symlinks to your launch/, config/
```

After `source install/setup.bash`, `ros2 run` and `ros2 launch` search the `install/` tree.

---

## Step 2 — URDF Robot Description

**Goal:** Describe the robot's physical geometry so ROS2 knows where each sensor is. Verify the TF2 transform tree is correct.

### What is URDF?

URDF (Unified Robot Description Format) is an XML file that answers: **what is the robot's physical shape and how are its parts connected?** It is data, not code. Other nodes read it and act on it.

### What is Xacro?

Xacro is a macro preprocessor for URDF. It adds variables, math, and conditionals to plain XML:

```xml
<xacro:property name="wheel_radius" value="0.033"/>
<origin xyz="0.0 ${wheel_separation / 2.0} -0.023"/>
```

Before `robot_state_publisher` sees the file, the `xacro` tool expands all macros into plain URDF XML. Define a constant once, reference it everywhere.

### Links and Joints — the two primitives

Everything in a URDF is either a **link** (a physical body) or a **joint** (a connection between two links).

```
base_footprint  ← virtual floor frame (massless, z=0 always)
  └── base_link         (fixed joint, +10mm up)
        ├── wheel_left_link   (continuous joint, spins)
        ├── wheel_right_link  (continuous joint, spins)
        ├── caster_back_link  (fixed, rear ball wheel)
        ├── imu_link          (fixed, -32mm, +68mm)
        └── base_scan         (fixed, -32mm, +172mm)
```

**Fixed joint** — frames never move relative to each other. `robot_state_publisher` publishes this *once* to `/tf_static`. Always available, even if the node restarts.

**Continuous joint** — spins without limit. `robot_state_publisher` updates `/tf` every time `/joint_states` arrives with new encoder values.

### Why `base_footprint` exists

Nav2 and SLAM work in the 2D floor plane (z=0). They need a frame that is *always* at floor level regardless of robot tilt or vibration. `base_footprint` is that frame. `base_link` (the robot body) sits 10mm above it. The separation is small but semantically important.

### Coordinate convention (ROS right-hand rule)

```
x = forward
y = left
z = up
```

This applies to every frame in the system. A positive rotation around z (yaw) is counter-clockwise when viewed from above.

### Why the LiDAR offset matters critically

The RPLiDAR reports all distances relative to its own origin (`base_scan` frame). When SLAM Toolbox receives a laser scan, it uses the TF chain `base_footprint → base_scan` to project those distances into the map. The LiDAR is 172mm above and 32mm behind `base_link`. If either value is wrong by even 2cm, every wall in your SLAM map appears in the wrong place.

### `robot_state_publisher` — the C++ node that reads the URDF

This is a well-tested C++ node from Robotis/OSRF. You don't write it; you configure it.

Internally it:
1. Parses the URDF XML using `urdf::Model` (C++ library)
2. Builds a kinematic tree using `KDL::Tree` (kinematics library)
3. For each **fixed** joint → broadcasts once to `/tf_static`
4. For each **continuous** joint → subscribes to `/joint_states`, updates `/tf` on every message

### The launch file — `description.launch.py`

A ROS2 Python launch file is not a script that runs top-to-bottom. It is a **description** — it builds a data structure that the `ros2 launch` system then executes. The one required function is `generate_launch_description()`.

**`get_package_share_directory()`** is the correct ROS2 API for finding installed files:

```python
pkg_share = get_package_share_directory('tb3_bringup')
urdf_path = os.path.join(pkg_share, 'hardware', 'urdf', '...')
```

After `colcon build`, files are read from `install/`, not source. `get_package_share_directory` always returns the correct installed path. Using `__file__` or `os.getcwd()` is fragile.

**`Command()`** runs a shell command at launch time and captures stdout as a string:

```python
'robot_description': ParameterValue(
    Command(['xacro ', LaunchConfiguration('urdf_file')]),
    value_type=str
)
```

`xacro /path/to/file.urdf.xacro` expands the macros and prints the resulting URDF XML to stdout. `Command()` captures that. `ParameterValue(..., value_type=str)` prevents the launch system from trying to YAML-parse the XML (which starts with `<?xml` and would crash).

**`DeclareLaunchArgument`** makes parameters overridable from CLI:

```python
# In the launch file:
DeclareLaunchArgument(name='urdf_file', default_value=default_urdf)

# At the terminal:
ros2 launch tb3_bringup description.launch.py urdf_file:=/other/robot.urdf.xacro
```

### TF2 in Python — the three-object pattern

This pattern appears in every node that consumes transforms:

```python
self.tf_buffer   = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
```

- **Buffer** — in-memory database of TF messages, indexed by time. Stores 10 seconds of history.
- **TransformListener** — subscribes to `/tf` and `/tf_static` in the background, feeding every message into the Buffer. You never call it directly.
- **`lookup_transform(target, source, time)`** — asks "what transform maps a point from `source` frame to `target` frame at this time?"

The result is a `TransformStamped` with `.transform.translation (x, y, z)` and `.transform.rotation (x, y, z, w quaternion)`.

### Quaternions — brief introduction

ROS2 uses quaternions to represent rotation, not Euler angles (roll, pitch, yaw). A quaternion is four numbers `(x, y, z, w)` where the magnitude is always 1.0.

For a ground robot that only rotates around the z-axis (yaw):
```
qx = 0
qy = 0
qz = sin(yaw / 2)
qw = cos(yaw / 2)
```

Identity (no rotation) = `(0, 0, 0, 1)`. The `w=0.707, z=-0.707` you see on the wheel joints is a -90° rotation around x — the URDF's `rpy="-1.5707963 0 0"` to align the wheel cylinder geometry with the robot's axle direction.

---

## Step 3 — Odometry Publisher

**Goal:** Read wheel encoder ticks from `/joint_states`, apply differential-drive kinematics, publish the robot's estimated pose as `nav_msgs/Odometry` on `/odom`.

### What is odometry?

Odometry is **dead reckoning** — estimating position by integrating motion. No external reference (no GPS, no camera, no map). The robot knows only how much each wheel turned.

It drifts over time because:
- Wheels slip on smooth floors (angle ≠ distance traveled)
- Discrete encoder resolution (4096 ticks/revolution = ~0.05mm per tick)
- Effective wheel radius varies slightly under load

Over 1m, well-calibrated TurtleBot3 odometry is within ~5%. Over 10m, drift can reach 50cm. The EKF (Step 6) and SLAM (Step 9) correct for this.

### Differential-drive kinematics

The TurtleBot3 Burger has two independently driven wheels and one passive caster. Given encoder angle deltas `Δθ_L` and `Δθ_R`:

```
dist_left  = Δθ_L × wheel_radius      # arc length in metres
dist_right = Δθ_R × wheel_radius

linear_vel  = (dist_right + dist_left)  / 2      # avg = straight-line distance
angular_vel = (dist_right - dist_left)  / wheel_separation  # difference = rotation
```

The robot's centre moves the average of the two wheels. If both move 5cm, the robot moved 5cm forward. If one moves +5cm and the other -5cm, the robot spun in place.

### Midpoint (Runge-Kutta 2nd order) heading

Naive integration projects forward using the heading at the *start* of the step:

```python
self.x += linear_dist * cos(self.theta)   # overshoots curves
```

Better: use the heading at the *midpoint* of the step (RK2):

```python
mid_theta = self.theta + angular_dist / 2.0
self.x += linear_dist * cos(mid_theta)
self.y += linear_dist * sin(mid_theta)
```

The difference matters on curves. Over many small steps, RK2 accumulates less geometric error.

### Theta normalisation

```python
self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
```

Keeps theta in `[-π, π]`. Without this, theta accumulates to large floats after many rotations. `atan2(sin(θ), cos(θ))` is the standard idiom for normalising any angle.

### The `nav_msgs/Odometry` message

```
nav_msgs/Odometry
  header
    frame_id:       'odom'          ← pose is expressed in this frame
  child_frame_id:   'base_footprint' ← the frame being described
  pose
    pose
      position:     (x, y, 0)
      orientation:  quaternion (yaw only for 2D)
    covariance:     6×6 matrix (flattened to 36 values)
  twist
    twist
      linear:       (vx, 0, 0)
      angular:      (0, 0, ωz)
    covariance:     6×6 matrix
```

### Covariance matrices

The 6×6 covariance matrix encodes uncertainty in `[x, y, z, roll, pitch, yaw]` order. For a 2D robot, z/roll/pitch are known exactly (near-zero variance). x, y, yaw get realistic uncertainty values.

```python
msg.pose.covariance = [
    1e-3, 0, 0, 0, 0, 0,    # x variance: ±1mm per step
    0, 1e-3, 0, 0, 0, 0,    # y variance
    0, 0, 1e-6, 0, 0, 0,    # z (not used in 2D)
    0, 0, 0, 1e-6, 0, 0,    # roll (not used)
    0, 0, 0, 0, 1e-6, 0,    # pitch (not used)
    0, 0, 0, 0, 0, 1e-2,    # yaw variance: more uncertain than position
]
```

The EKF (Step 6) reads these values to decide how much to trust wheel odometry vs IMU. Too-small covariance = EKF over-trusts odometry. Too-large = EKF ignores it.

### QoS — why `BEST_EFFORT` for sensor topics

```python
sensor_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    ...
)
```

`BEST_EFFORT` = fire and forget. If a packet is lost, it's gone.
`RELIABLE` = guaranteed delivery, retransmits dropped packets.

For 50Hz sensor data: if one `/joint_states` message is dropped, the next one arrives in 20ms. Retransmitting a stale sensor reading 20ms later is worse than just missing it. The OpenCR's publisher also uses `BEST_EFFORT` — the QoS profiles must match or ROS2 refuses to connect them.

### `/joint_states` — always look up by name

```python
joint_positions = dict(zip(msg.name, msg.position))
left_pos = joint_positions['wheel_left_joint']
```

The `name` and `position` arrays are parallel but the order is NOT guaranteed. A firmware update could reorder them. Always build a dict and look up by name — never by index.

### Data flow for Step 3

```
OpenCR firmware (hardware)
  └── /joint_states (sensor_msgs/JointState, ~50Hz)
        └── OdometryPublisher.joint_states_callback()
              ├── extract wheel positions by name
              ├── compute Δθ per wheel (delta from previous)
              ├── arc_length = Δθ × wheel_radius
              ├── differential-drive kinematics
              ├── integrate pose with RK2 midpoint heading
              └── publish nav_msgs/Odometry on /odom
                    ├── (Step 4) tf2_broadcaster reads /odom → broadcasts TF
                    └── (Step 6) EKF reads /odom → fuses with IMU
```

### What this node deliberately does NOT do

- Does **not** broadcast TF transforms (Step 4)
- Does **not** fuse with IMU (Step 6)
- Does **not** know about the map (Step 9)

Keeping each concern in its own node makes each one independently testable and replaceable.

---

---

## Step 4 — TF2 Broadcaster

**Goal:** Bridge the `/odom` message into the TF2 transform tree by broadcasting the dynamic `odom → base_footprint` transform.

### Topics vs the TF2 tree — two different systems

After Step 3, the robot's pose exists as a stream of `nav_msgs/Odometry` messages on `/odom`. But Nav2, SLAM Toolbox, and RViz2 don't subscribe to `/odom` directly. They consume the **TF2 transform tree** — a different, queryable system.

| `/odom` topic | TF2 transform tree |
|---|---|
| A message stream — subscribe and receive one at a time | A database — query any frame pair at any time from any node |
| Pull model: you wait for the next message | Push + pull: nodes broadcast transforms, others look them up |
| Used by the EKF, odometry consumers | Used by Nav2, SLAM, RViz2, sensor fusion |

The TF2 broadcaster's job is to translate between these two systems: read from `/odom`, write to `/tf`.

### Static vs Dynamic transforms — recap

| Type | Broadcast to | When | Example |
|---|---|---|---|
| Static | `/tf_static` (latched) | Once at startup | `base_link → base_scan` (LiDAR never moves) |
| Dynamic | `/tf` | Continuously, every update | `odom → base_footprint` (robot moves) |

`robot_state_publisher` (Step 2) handled all the static transforms from the URDF. This node handles the one dynamic transform that changes as the robot moves.

### Why this is a separate node from the odometry publisher

In Step 6 (EKF), the filtered pose (`/odometry/filtered`) is more accurate than raw `/odom`. We want the TF tree to use the filtered pose. Because the broadcaster is a separate node, we simply change one parameter — which topic it subscribes to — and the entire TF tree automatically uses the better estimate. The odometry publisher stays completely unchanged.

```
Step 4 (now):  /odom             → tf2_broadcaster → odom → base_footprint
Step 6 (later): /odometry/filtered → tf2_broadcaster → odom → base_footprint
```

### `TransformBroadcaster` — not a regular publisher

```python
self.broadcaster = tf2_ros.TransformBroadcaster(self)
self.broadcaster.sendTransform(transform)
```

`TransformBroadcaster` is a thin wrapper around a publisher on the `/tf` topic. It is NOT created with `self.create_publisher()` — it manages its own internal publisher. You call `sendTransform(TransformStamped)` and it handles the rest. The TF buffer in every other node (including `tf2_checker` from Step 2) will receive and cache these broadcasts automatically via their `TransformListener`.

### Why timestamp comes from the odom message, not `now()`

```python
t.header.stamp = odom.header.stamp   # correct
# t.header.stamp = self.get_clock().now().to_msg()  # wrong
```

The TF2 system is time-indexed. When Nav2 asks "where was the robot at time T?", the TF buffer looks up the transform that was valid at exactly time T. If we timestamp the transform with `now()` instead of the odom message's timestamp, there is a small but nonzero gap between when the pose was computed and when it was stamped. Under high load on a slow RPi4, this gap can be tens of milliseconds — enough for the TF lookup to fail with "extrapolation into the future" errors. Always use the source message's timestamp.

### `_odom_to_transform()` — pure conversion, no state

```python
def _odom_to_transform(self, odom: Odometry) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp    = odom.header.stamp
    t.header.frame_id = odom.header.frame_id       # 'odom'
    t.child_frame_id  = odom.child_frame_id        # 'base_footprint'
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = 0.0
    t.transform.rotation = odom.pose.pose.orientation
    return t
```

This method has no side effects and no internal state — given the same input, it always returns the same output. This makes it trivially unit-testable without a running ROS2 system, a broadcaster, or any timing concerns. The actual `sendTransform()` call is in `odom_callback()`, kept separate so tests don't need to send anything to the middleware.

`z = 0.0` is hardcoded because `base_footprint` is defined as the floor-level projection of the robot — it is always at z=0 by definition, regardless of what the odometry message says.

### The complete TF chain after Step 4

```
/tf_static (from robot_state_publisher, Step 2):
  base_footprint → base_link     (fixed, +10mm up)
  base_link → base_scan          (fixed, LiDAR position)
  base_link → imu_link           (fixed, IMU position)
  base_link → wheel_*_link       (updated from /joint_states)

/tf (from tf2_broadcaster, Step 4):
  odom → base_footprint          (dynamic, updates with /odom)

Full chain:
  odom → base_footprint → base_link → base_scan
                                    → imu_link
```

SLAM Toolbox (Step 9) will add `map → odom` to the top of this chain. At that point, the complete path `map → odom → base_footprint → base_link → base_scan` allows SLAM to project every laser hit from sensor frame all the way into map frame.

### Data flow for Steps 3 + 4 combined

```
OpenCR firmware
  └── /joint_states
        └── odometry_publisher.py
              ├── differential-drive kinematics
              ├── integrate pose (x, y, theta)
              └── /odom  (nav_msgs/Odometry)
                    └── tf2_broadcaster.py
                          └── TransformBroadcaster.sendTransform()
                                └── /tf  (odom → base_footprint)
                                      └── cached in TF buffer of every node
```

---

## Step 5 — IMU Republisher

**Goal:** Take the raw `sensor_msgs/Imu` from the OpenCR board, fix its metadata (frame_id and covariance matrices), and republish on `/imu/data` so the EKF can consume it correctly.

### What the MPU-9250 measures

The MPU-9250 inside the OpenCR board has three sensors in one chip:

| Sensor | Measures | Field in Imu message |
|---|---|---|
| Gyroscope | Rotational velocity around each axis (rad/s) | `angular_velocity.x/y/z` |
| Accelerometer | Linear acceleration including gravity (m/s²) | `linear_acceleration.x/y/z` |
| DMP filter | Fused orientation estimate | `orientation` (quaternion) |

The OpenCR firmware already processes the raw chip readings and publishes a complete `sensor_msgs/Imu` on `/imu`. We don't need to talk to the chip directly.

### The two metadata problems

**Problem 1 — `frame_id` is empty.**
The EKF uses `frame_id` to look up the TF transform between the IMU and the robot body. It calls `tf_buffer.lookup_transform('base_link', msg.header.frame_id, ...)`. If `frame_id` is empty, the TF lookup fails and the EKF silently ignores the IMU entirely. It must be `'imu_link'` — matching the URDF joint from Step 2.

**Problem 2 — covariance matrices are all zeros.**
The `sensor_msgs/Imu` message has three 3×3 covariance matrices (one per sensor). The EKF uses these to weight how much to trust each measurement — higher variance = less trust. All-zero covariances mean "perfect certainty," which causes the EKF's math to become numerically unstable. The EKF may also interpret `covariance[0] = 0` as the special value `-1` (which means "ignore this sensor entirely" — a ROS convention).

### The `sensor_msgs/Imu` message structure

```
sensor_msgs/Imu
  header
    stamp:     when the measurement was taken
    frame_id:  which coordinate frame the data is in → must be 'imu_link'

  orientation:             quaternion (from DMP fusion)
  orientation_covariance:  [9 floats] 3x3 matrix, uncertainty in orientation

  angular_velocity:             gyro reading (rad/s)
  angular_velocity_covariance:  [9 floats] 3x3 matrix

  linear_acceleration:             accelerometer reading (m/s²)
  linear_acceleration_covariance:  [9 floats] 3x3 matrix
```

All three covariance matrices are 3×3, flattened row-major to 9 values: `[xx, xy, xz, yx, yy, yz, zx, zy, zz]`.

**Special sentinel value:** if `covariance[0] = -1`, the EKF ignores that entire sensor component. This is by ROS convention. Our republisher must never produce this value.

### Diagonal covariance matrices

```python
[variance, 0,        0,
 0,        variance, 0,
 0,        0,        variance]
```

Setting only the diagonal means we assume the three axes are **independent** — uncertainty in the x-axis gyro reading doesn't tell us anything about the y-axis. This is a valid assumption for most IMUs in normal operation. Off-diagonal (correlation) terms would require careful calibration to measure correctly; zero is the safe default.

**MPU-9250 values used:**
- Orientation: `0.0001 rad²` — the DMP filter is fairly accurate
- Angular velocity: `0.0001 rad²/s²` — from gyro noise spec
- Linear acceleration: `0.001 m²/s⁴` — accel is noisier than gyro

These are starting values. The correct procedure is to record the IMU stationary for 30 seconds, compute the variance of each axis, and use those measured values. We'll do this during robot calibration.

### Why sensor data passes through unchanged

```python
clean.orientation           = raw.orientation
clean.angular_velocity      = raw.angular_velocity
clean.linear_acceleration   = raw.linear_acceleration
```

The OpenCR already applies the MPU-9250's internal coordinate frame alignment. The measurements are already expressed in the IMU chip's frame — which in the Burger is aligned with `base_link` (`rpy="0 0 0"` in the URDF). Modifying the readings here would introduce errors. We only fix the metadata; we never touch the numbers.

### Why covariances are pre-built at startup

```python
# In __init__:
self.orientation_cov = self._diagonal_covariance(orient_var)

# In callback:
clean.orientation_covariance = list(self.orientation_cov)
```

The callback fires at ~50Hz on the RPi4. Allocating a new 9-element list on every callback, 50 times per second, creates continuous memory allocation pressure. Building the list once at startup and copying it in the callback is more efficient. The `list()` call makes a shallow copy so each message gets its own list object (no aliasing between messages).

### Data flow

```
OpenCR hardware
  └── /imu  (raw, frame_id='', covariances=[0,0,...])
        └── imu_republisher.py
              ├── copy timestamp unchanged
              ├── override frame_id = 'imu_link'
              ├── copy sensor readings unchanged
              ├── replace covariances with MPU-9250 values
              └── /imu/data  (clean, ready for EKF)
                    └── ekf_node.py (Step 6)
```

### What this node deliberately does NOT do

- Does **not** filter noise (low-pass filtering, outlier rejection)
- Does **not** apply calibration offsets (gyro bias, accel bias)
- Does **not** fuse sensors (that is the EKF's job)

A production IMU pipeline would include calibration correction. For Stage 1, the MPU-9250's built-in DMP calibration is sufficient. Calibration refinement would be a future improvement.

*More steps to be added as the project progresses.*
