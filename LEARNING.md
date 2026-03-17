# TurtleBot3 Autonomy Stack — Learning Notes

> Personal study notes — one entry per step. Written to build a mental model,
> not just working code. Each section explains *why* before *how*.

---

## Table of Contents

- [Stage 1 — Foundation](#stage-1--foundation)
  - [Step 1 — Package Scaffolding](#step-1--package-scaffolding)
  - [Step 2 — URDF Robot Description](#step-2--urdf-robot-description)
  - [Step 3 — Odometry Publisher](#step-3--odometry-publisher)

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

*More steps to be added as the project progresses.*
