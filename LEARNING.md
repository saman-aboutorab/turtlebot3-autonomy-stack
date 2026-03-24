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
  - [Step 6 — EKF Node](#step-6--ekf-node-extended-kalman-filter)

---

# Stage 1 — Foundation

Hardware: TurtleBot3 Burger (RPi4 + OpenCR + RPLiDAR A1M8 + MPU-9250 IMU)
Branch: `stage-1` | Milestone tag: `v1.0.0`

---

## Step 1 — Package Scaffolding

**Goal:** Create the three ROS2 packages and get `colcon build` to pass before writing any robot logic.

### Simple analogy

Think of a ROS2 package like a **drawer in a toolbox**.

Each drawer (package) has a label on the front saying what's inside and what tools it needs to work (`package.xml`). Inside are the actual tools — scripts, configs, launch files. The toolbox itself (`install/`) is what you open when you want to use something; the workshop bench (`src/`) is where you build and modify tools.

You wouldn't throw all your tools into one giant drawer. Similarly, we split into three packages: `tb3_odometry` (sensing), `tb3_navigation` (planning), `tb3_bringup` (wiring them together). If the navigation drawer breaks, you can fix it without touching the sensing drawer.

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

### Simple analogy

Imagine you hired a delivery driver who has never seen your building. You tell them: *"The front door is at ground level. The mailroom is 10 metres inside and 5 metres to the left. The loading dock is out back, 20 metres behind the front door."*

That description is the URDF. The driver (SLAM, Nav2, RViz2) now knows the layout and can reason about where things are relative to each other — without ever having physically seen the building.

Without it, SLAM would receive laser hits from the LiDAR and have no idea whether those hits came from 17cm above the robot or from ground level. It would be like trying to map a building when your laser pointer could be anywhere.

**Concrete example from our robot:**
The LiDAR is mounted 172mm above and 32mm behind the robot's centre. When the LiDAR says "there's a wall 50cm ahead," SLAM uses the URDF to work out that this measurement came from a point that is 50cm + 32mm ahead of the robot's centre, and 172mm up. Without that offset, the wall appears in the wrong place in the map.

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

### Simple analogy

Imagine you're **blindfolded, counting your steps** as you walk across a room.

- You know your stride is roughly 75cm.
- You count 10 steps forward → you estimate you're 7.5m ahead of where you started.
- You count 3 steps while turning right → you estimate your heading changed by ~30°.

You never looked up. You never checked a map. You just integrated your own motion over time. That's odometry.

The problem: if you slip on a rug (wheel slip), your step count is wrong and your estimate drifts. After 50 steps, you might think you're 37.5m ahead but you're actually only 35m. The error grows with distance. That's why we add the IMU (Step 5) and EKF (Step 6) — a second opinion that doesn't depend on your steps.

**Concrete example:**
Both wheels rotate 1 radian. `dist = 1.0 × 0.033m = 0.033m`. Robot moved 3.3cm forward. Simple.
Right wheel rotates 1 rad, left stays still. `angular = (0.033 - 0) / 0.160 = 0.206 rad`. Robot turned ~12° left.

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

### Simple analogy

Think of `/odom` like a **private text message** from the odometry node: "hey, the robot is now at position (1.2, 0.3)." Only nodes that subscribe to that specific topic receive it, and they receive it as a stream — one message at a time, in order.

The TF2 tree is like a **public noticeboard** that anyone can walk up to and query at any time: "excuse me, where is `base_scan` relative to `odom` right now?" The noticeboard gives you the answer instantly, and it can also chain transforms automatically — if you ask "where is `base_scan` relative to `map`?" it multiplies four transforms together for you without you having to know the chain.

The TF2 broadcaster's job is to **take the private text and pin it to the noticeboard** — same information, different system, different audience.

**Concrete example:**
After this step:
- SLAM can ask: "where is the LiDAR in the odom frame?" → TF chains `odom→base_footprint→base_link→base_scan`
- RViz2 can ask: "where should I draw the robot model?" → same chain
- Nav2 can ask: "how far is the robot from the goal?" → same chain

None of those systems subscribe to `/odom`. They all use the noticeboard.

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

### Simple analogy — what is an IMU?

Imagine you're **blindfolded in the back seat of a car**. You can't see anything outside. But you can still *feel*:

- When the car accelerates forward — you get pushed back into the seat *(accelerometer)*
- When it brakes — you lurch forward *(accelerometer)*
- When it turns left — you slide right in the seat *(gyroscope)*

That's exactly what an IMU does. It *feels* motion without needing to see anything external.

**Why do we need it alongside wheel odometry?**

Wheel odometry counts steps. But it can't detect wheel slip — if the wheel spins on a smooth floor without gripping, the encoder counts ticks but the robot didn't actually move. The odometry is now wrong and doesn't know it.

The gyroscope doesn't care about wheels. It physically *feels* the rotation. If the wheels slipped during a turn, the gyroscope still gives you the correct rotation angle.

Think of them as two people navigating the same blindfolded walk:
- Person A (odometry): counts steps carefully, but occasionally slips on ice without noticing
- Person B (IMU): feels every turn and acceleration, but their sense of direction drifts after a long time

The EKF (Step 6) is the **third person** who listens to both and produces a better estimate than either alone. Person A is more reliable going straight, Person B is more reliable turning. Together they cover each other's weaknesses.

**Concrete example:**
Robot drives straight, then wheels slip sideways on a shiny tile:
- Odometry says: "I moved 20cm forward" ✓ (straight part was fine)
- Odometry says: "I didn't turn at all" ✗ (didn't feel the sideways slip)
- Gyroscope says: "I rotated 5° to the left" ✓ (felt the slip physically)
- EKF blends both → "you moved ~20cm forward and rotated ~5° left" ✓

### What this node specifically does

The raw IMU data from the OpenCR is *correct* — the measurements are fine. But two pieces of metadata are broken: the `frame_id` is empty and the covariance matrices are all zeros. This node fixes only those two things. The numbers pass through untouched.

It's like receiving a correctly-filled-out form but the sender forgot to write their name and department. The data is right — you just need to stamp it with the right labels before filing it.

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

---

## Step 6 — EKF Node (Extended Kalman Filter)

**Goal:** Fuse wheel odometry (`/odom`) and IMU heading (`/imu/data`) into a single, better pose estimate on `/odometry/filtered`. Update `tf2_broadcaster` to use the filtered output for TF2.

---

### High-level overview

You have two sensors. Both estimate the robot's pose, but each fails in a different way:

| Sensor | What it measures | How it fails |
|---|---|---|
| Wheel odometry (`/odom`) | Counts rotations → integrates x, y, θ | Heading error compounds: small drift → large position error over distance |
| IMU (`/imu/data`) | Absolute orientation from DMP fusion | Noisy moment-to-moment; gives no information about x or y |

The EKF's job: **produce one estimate that is better than either sensor alone**.

It runs a two-step loop every time `/odom` arrives:

1. **Predict** — apply the motion model to advance the state using the wheel velocities
2. **Correct** — pull the predicted heading toward the IMU reading, weighted by how much each source is trusted

The weight is computed automatically from the covariance matrices. You do not tune it manually — you just set how noisy each sensor is (`Q` and `R`), and the filter figures out the optimal blend.

---

### Simple analogy

Imagine navigating in fog with two tools:

1. **A pedometer** (wheel odometry) — counts your steps and direction. Works well over short distances, but if your compass is off by even 2°, after 50 steps you are visibly in the wrong place. The error grows with every step.
2. **A compass** (IMU heading) — gives your absolute facing direction at any moment. Not affected by step-counting errors, but jiggles slightly with every bump.

A Kalman filter is a **trust allocator**. It continuously asks "how confident am I in each source right now?" and blends them proportionally. If the pedometer has been drifting for a while (high uncertainty P), lean on the compass. If the compass just gave a noisy reading (high R), lean on the pedometer.

The result: heading error stays *bounded* instead of growing forever, and position accuracy is dramatically better over long distances.

---

### Concrete example with real numbers

Robot drives 5 m in a straight line. Left wheel is slightly slippery → heading drifts 3°.

**Without EKF (raw odometry only):**
- Heading error: 3°
- Lateral position error after 5 m: `5 × sin(3°) ≈ 26 cm`
- After 20 m: over 1 m off — the robot misses doorways and obstacles

**With EKF (odom + IMU):**
- IMU corrects heading 50 times per second
- Residual heading error: ~0.5°
- Lateral position error after 5 m: `5 × sin(0.5°) ≈ 4 cm`

The gain comes entirely from fixing heading early. Position error is the *integral* of heading error, so a small, bounded heading error produces a small, bounded position error.

---

### What is a Kalman filter, really?

Think of it as a scientist who maintains two things: a **best estimate** and a **confidence level** about where the robot is. Every time the odometry fires, the scientist runs two steps:

#### PREDICT

"Based on the wheel velocities, where should the robot be now?"

```
x_new = x + v * cos(θ) * dt      ← move forward along current heading
y_new = y + v * sin(θ) * dt      ← move sideways along current heading
θ_new = θ + ω * dt               ← rotate by angular velocity × time
```

The scientist also updates their confidence:

```
P_new = F * P * F^T + Q
```

- `F * P * F^T` — stretches the old uncertainty through the motion model's Jacobian
- `+ Q` — adds process noise: even if we were perfectly confident, driving a bit makes us less sure (wheels slip)

Confidence *decreases* during prediction — time passing always increases uncertainty.

#### CORRECT

"The IMU says θ = 0.52 rad. My prediction says θ = 0.48 rad. How much should I move my estimate?"

Step 1 — compute the **Kalman gain** K:
```
K = P * H^T * (H * P * H^T + R)^{-1}
```
- `P` is our prediction uncertainty: larger P → we trust IMU more → K is larger
- `R` is IMU noise: larger R → we trust IMU less → K is smaller
- `H = [0, 0, 1]` — selects just the heading component of the state

Step 2 — **update** state and confidence:
```
innovation = θ_imu − θ_pred              ← how surprising is the measurement?
state      = state + K * innovation      ← pull estimate toward measurement
P          = (I − K * H) * P            ← confidence grows (uncertainty shrinks)
```

If K is large → the IMU reading moves the estimate a lot.
If K is small → the IMU barely changes the estimate, prediction wins.

Confidence *increases* after correction — new data always reduces uncertainty.

---

### Why "Extended" Kalman Filter?

A standard Kalman filter requires a **linear** motion model (a matrix multiplication). Our model contains `cos(θ)` and `sin(θ)` — non-linear.

The EKF fix: at every prediction step, linearise the model around the current state by computing the Jacobian (partial derivatives of the model with respect to the state):

```
F = d(motion model) / d(state)  =

    [[1,  0,  -v * sin(θ) * dt],    ← how x_new changes if x, y, θ change
     [0,  1,   v * cos(θ) * dt],    ← how y_new changes
     [0,  0,   1              ]]    ← how θ_new changes
```

We recompute `F` at every step because θ changes. This local linearisation is what makes it "Extended" — everything else is a standard Kalman filter.

---

### The 3-DOF state vector

```python
self._state = np.zeros(3)   # [x_pos, y_pos, theta]
```

We track three numbers: position (x, y) and heading (θ). We do **not** include velocities in the state. The velocities `v` and `ω` come directly from the odometry message each step as "control inputs" — no need to estimate them separately.

Why not 6-DOF? The robot moves on a flat floor. z, roll, and pitch are always zero. Keeping the state small keeps the maths fast and the code readable. A 6×6 EKF would add complexity with no benefit here.

---

### Detailed code walkthrough

#### Initialisation — matrices built once at startup

```python
# State starts at origin with large uncertainty
self._state = np.zeros(3)        # [x=0, y=0, θ=0]
self._P     = np.eye(3) * 1.0   # 1 m² / 1 rad² — "I have no idea where I am"

# Process noise Q — how much we distrust the motion model per step
# Diagonal: [x uncertainty, y uncertainty, heading uncertainty]
self._Q = np.diag([proc_pos, proc_pos, proc_heading])

# Measurement noise R — how noisy the IMU heading is (1×1 scalar)
self._R = np.array([[meas_imu]])

# Measurement model H: maps 3D state to scalar heading observation
# [0, 0, 1] means "look at element 2 of the state (theta)"
self._H = np.array([[0.0, 0.0, 1.0]])
```

`P`, `Q`, and `R` are the three knobs you tune. In practice:
- Make `Q` larger → predict conservatively, rely on measurements more
- Make `R` larger → IMU is noisy, rely on prediction more
- Start `P` large → the filter will converge quickly once measurements arrive

---

#### _imu_callback — just stores the latest heading

```python
def _imu_callback(self, msg: Imu) -> None:
    self._imu_heading   = self._yaw_from_quaternion(msg.orientation)
    self._imu_available = True
```

The IMU fires at ~50 Hz. We do **not** run the EKF correction here. We just store the latest heading. The correction runs inside `_odom_callback`, which fires at the odometry rate (~30 Hz). This prevents the filter from over-weighting the IMU by correcting 50 times between every odometry update.

---

#### _odom_callback — the main EKF loop

```python
def _odom_callback(self, msg: Odometry) -> None:
    t  = stamp.sec + stamp.nanosec * 1e-9
    dt = t - self._last_odom_time       # time since last message

    v = msg.twist.twist.linear.x        # linear velocity from odometry
    w = msg.twist.twist.angular.z       # angular velocity from odometry

    self._state, self._P = self._predict(self._state, self._P, v, w, dt)

    if self._imu_available:
        self._state, self._P = self._correct(self._state, self._P, self._imu_heading)

    self._publish_filtered(stamp)
```

The velocities `v` and `w` come from the odometry message's `twist` field — this is exactly what `odometry_publisher.py` filled in from the encoder kinematics. We use them as control inputs to the EKF instead of recomputing kinematics from scratch.

---

#### _predict — motion model + Jacobian

```python
# Non-linear motion model (same equations as odometry_publisher.py)
x_new     = x + v * math.cos(theta) * dt
y_new     = y + v * math.sin(theta) * dt
theta_new = theta + w * dt
theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))  # normalise to [-π, π]

# Jacobian of the motion model — re-computed every step because theta changes
F = np.array([
    [1.0, 0.0, -v * math.sin(theta) * dt],   # d(x_new)/d(x, y, θ)
    [0.0, 1.0,  v * math.cos(theta) * dt],   # d(y_new)/d(x, y, θ)
    [0.0, 0.0,  1.0                      ],  # d(θ_new)/d(x, y, θ)
])

# Covariance propagation: P grows (prediction adds uncertainty)
P_new = F @ P @ F.T + self._Q
```

Notice the off-diagonal terms in F: `−v·sin(θ)·dt` and `+v·cos(θ)·dt`. These encode the geometric fact that a heading error causes a position error in the perpendicular direction. The Jacobian lets the covariance matrix reflect this coupling correctly.

---

#### _correct — Kalman gain and state update

```python
# Innovation: how far off is our prediction from the IMU reading?
# atan2(sin(Δ), cos(Δ)) handles the ±π wraparound correctly
innovation = math.atan2(
    math.sin(theta_imu - theta_pred),
    math.cos(theta_imu - theta_pred)
)

# Innovation covariance S = H * P * H^T + R  (1×1 scalar)
S = H @ P @ H.T + R

# Kalman gain K = P * H^T * S^{-1}   (3×1 vector)
# K[0]: how much to adjust x per radian of innovation
# K[1]: how much to adjust y per radian of innovation
# K[2]: how much to adjust θ per radian of innovation
K = P @ H.T @ np.linalg.inv(S)

# State update: pull toward the measurement
state_new = state + K.flatten() * innovation

# Covariance update: we got new information, so we are more confident
P_new = (I - K @ H) @ P
```

Why does `K` have x and y components even though the IMU only measures θ? Because `P` has off-diagonal terms coupling x, y, and θ. If the filter knows "x and θ are correlated" (because they are — heading errors cause x errors), then a heading correction can also slightly adjust x. With a diagonal P (as at startup), K[0] and K[1] are zero and only θ is corrected.

The angle wraparound fix (`atan2(sin(Δ), cos(Δ))`) is critical. Without it, if the robot is facing θ=3.1 rad and the IMU says −3.1 rad (same direction, opposite sign), the raw difference would be −6.2 rad and the correction would violently spin the estimated heading the wrong way.

---

#### _publish_filtered — mapping 3×3 P into the 6×6 covariance

```python
# nav_msgs/Odometry uses 6-DOF order: [x, y, z, roll, pitch, yaw]
# We track [x, y, yaw] — indices 0, 1, 5 in that order
cov     = [0.0] * 36
cov[0]  = P[0, 0]   # x-x
cov[1]  = P[0, 1]   # x-y
cov[5]  = P[0, 2]   # x-yaw
cov[6]  = P[1, 0]   # y-x
cov[7]  = P[1, 1]   # y-y
cov[11] = P[1, 2]   # y-yaw
cov[30] = P[2, 0]   # yaw-x
cov[31] = P[2, 1]   # yaw-y
cov[35] = P[2, 2]   # yaw-yaw
```

The `nav_msgs/Odometry` covariance is a 6×6 matrix (36 values) for `[x, y, z, roll, pitch, yaw]`. Our EKF only tracks 3 of those 6 DOF, so we zero-fill the rest and splice our 3×3 P into the correct positions. Downstream nodes (Nav2, SLAM) use this covariance to know how much to trust the pose.

---

### What changed in tf2_broadcaster (Step 6)

In Step 4, `tf2_broadcaster.py` subscribed to `/odom` (raw wheel odometry). In Step 6 it now subscribes to `/odometry/filtered` (EKF output). This means the TF2 transform — which every downstream node uses to locate the robot — now benefits from the fused heading estimate.

An `input_topic` parameter lets you revert to `/odom` for debugging:
```bash
ros2 run tb3_odometry tf2_broadcaster.py --ros-args -p input_topic:=/odom
```

---

### Data flow

```
OpenCR
  ├── /joint_states → odometry_publisher.py → /odom
  │                                               │
  │   ekf_node.py ←────────────────────────────── │ (PREDICT: v, ω from twist)
  │       │                                        │
  └── /imu → imu_republisher.py → /imu/data       │
                                       │           │
                             ekf_node.py ← ────────┘ (stored for CORRECT step)
                                 │
                                 └── /odometry/filtered
                                           │
                                     tf2_broadcaster.py
                                           │
                                          /tf  (odom → base_footprint)
                                           │
                              SLAM / Nav2 / RViz
```

---

### What this node deliberately does NOT do

- Does **not** fuse position measurements (only heading). x and y are corrected indirectly through the covariance coupling, not directly observed.
- Does **not** handle IMU dropout gracefully. If `/imu/data` stops arriving, the filter runs predict-only and heading drift returns.
- Does **not** initialise from a known pose. Always starts at origin `[0, 0, 0]`.
- Does **not** implement the Joseph form of the covariance update (`(I−KH)P(I−KH)^T + KRK^T`) for numerical stability. The simpler form `(I−KH)P` is sufficient for this application.

A production EKF (e.g., `robot_localization`) handles all of these. Our implementation keeps the maths visible so you understand what is happening before using a black-box solution.

---

## Step 7 — Velocity Controller

**Goal:** Sit between the motion planner and the robot's wheels. Apply three safety layers — velocity clamping, acceleration limiting, and safety timeout — before forwarding commands to the OpenCR.

---

### High-level overview

Nav2, teleop, and waypoint followers all send velocity commands on `/cmd_vel` without knowing the physical limits of the specific robot. A Burger's wheels physically cap out at 0.22 m/s and 2.84 rad/s. If Nav2 sends 2.0 m/s, the OpenCR clamps it internally — but in a discontinuous way that causes wheel slip and corrupts odometry.

More importantly: if the planner crashes or freezes, the last command keeps driving the robot into a wall.

The velocity controller solves all three problems in one place:

| Problem | Solution |
|---|---|
| Speed exceeds physical limit | Clamp to `[−max_vel, +max_vel]` on input |
| Step change in velocity → wheel slip | Ramp: advance at most `max_acc × dt` per control step |
| Planner crash → robot keeps moving | Timeout: if no command for > 0.5 s, ramp to zero |

---

### Simple analogy

Think of a new driver and a driving instructor in a car with dual controls.

- The **new driver** (Nav2/teleop) sends commands: "go 80 km/h, turn hard right"
- The **instructor** (velocity controller) intercepts:
  - "This road is 30 km/h — clamped." (velocity limit)
  - "You can't jump to 80 km/h instantly — ease in." (acceleration ramp)
  - "You fell asleep? Brake." (safety timeout)

The OpenCR (engine) only ever sees the instructor's smoothed, safe commands — never the raw student inputs.

---

### Concrete example with numbers

Robot needs to go from stopped to full speed (0.22 m/s) then stop.

**Without ramp (step change):**
- Step 1: publish 0.22 m/s → wheels try to spin up instantly → slip
- Odometry reads jump → EKF diverges → SLAM map tears

**With ramp (max_acc = 0.5 m/s², dt = 0.05 s → max_step = 0.025 m/s per tick):**
```
tick 1: 0.000 → 0.025
tick 2: 0.025 → 0.050
...
tick 9: 0.200 → 0.220   ← full speed reached in 0.44 s
```
Wheels spin up smoothly. No slip. Odometry stays accurate.

**Timeout in action:**
- Nav2 publishes 0.15 m/s at t=0
- Nav2 crashes at t=0.3 s
- Timeout fires at t=0.8 s (0.5 s after last command)
- Velocity ramps to 0 — robot stops safely

---

### Design pattern — timer-driven output

This is a key architectural choice worth understanding:

```
/cmd_vel_raw  →  _cmd_callback()    ← only STORES the target, never publishes
                      │
                  _target_lv
                  _target_av

20 Hz timer  →  _control_loop()    ← READS target, applies ramp, PUBLISHES
                      │
                  _current_lv
                  _current_av
                      │
                   /cmd_vel
```

The subscriber and publisher are **decoupled**. Nav2 might publish commands at 10 Hz, teleop at 30 Hz — none of that matters. The output always fires at exactly `control_rate` Hz (default 20 Hz). This also means the timeout check fires reliably even when the input topic goes silent.

Compare this to imu_republisher (Step 5), which published directly inside the callback. That works for pass-through republishing. Here we need a fixed-rate loop, so a timer is the right tool.

---

### Detailed code walkthrough

#### Two separate velocity variables

```python
self._target_lv  = 0.0   # what /cmd_vel_raw last requested (clamped)
self._target_av  = 0.0
self._current_lv = 0.0   # what we are actually publishing right now (ramped)
self._current_av = 0.0
```

Having two variables is the key to the ramp. `_target` can jump instantly (that's fine — it's just a stored value). `_current` moves toward `_target` slowly, at most `max_acc × dt` per step. The robot only ever sees `_current`.

---

#### _cmd_callback — clamp and store only

```python
def _cmd_callback(self, msg: Twist) -> None:
    self._target_lv     = self._clamp(msg.linear.x,  -self._max_lv, self._max_lv)
    self._target_av     = self._clamp(msg.angular.z, -self._max_av, self._max_av)
    self._last_cmd_time = self.get_clock().now()
```

Two things happen here:
1. Hard clamp to the Burger's physical limits. If Nav2 sends 2.0 m/s, it becomes 0.22 m/s immediately.
2. Record the timestamp — the timeout check compares this against `now()` in every control loop tick.

Nothing is published here. The subscriber's only job is to update state.

---

#### _control_loop — timeout, ramp, publish

```python
def _control_loop(self) -> None:
    # 1. Timeout: how long since the last command?
    elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
    if elapsed > self._timeout:
        self._target_lv = 0.0
        self._target_av = 0.0

    # 2. Ramp current toward target
    self._current_lv = self._ramp(self._current_lv, self._target_lv, self._max_la, self._dt)
    self._current_av = self._ramp(self._current_av, self._target_av, self._max_aa, self._dt)

    # 3. Publish
    msg = Twist()
    msg.linear.x  = self._current_lv
    msg.angular.z = self._current_av
    self._pub.publish(msg)
```

Notice the order matters: timeout check zeroes `_target` first, then ramp moves `_current` toward the (now-zeroed) target. The robot slows down gradually, not instantly — the deceleration ramp applies to the stop too.

---

#### _clamp — hard velocity limit

```python
@staticmethod
def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))
```

`max(lo, min(hi, value))` is the Python idiom for clamping. The inner `min(hi, value)` prevents exceeding the upper limit; the outer `max(lo, ...)` prevents going below the lower limit. Works for both positive and negative velocities because `lo` is set to `−max_vel`.

---

#### _ramp — acceleration limit

```python
@staticmethod
def _ramp(current: float, target: float, max_acc: float, dt: float) -> float:
    max_step = max_acc * dt             # largest allowed change this tick
    delta    = target - current         # how far we need to go
    step     = max(-max_step, min(max_step, delta))  # clamp delta to ±max_step
    return current + step
```

This is also a clamp, but on the *change* rather than the *value*. `max_acc × dt` converts acceleration (m/s²) to the maximum velocity increment per tick (m/s). If the required delta is smaller, go all the way. If larger, take only the allowed step.

Note: `_ramp` is symmetric — it limits both acceleration (ramping up) and deceleration (ramping down). This means `max_linear_acc` controls both how quickly the robot speeds up *and* how quickly it slows down.

---

### Data flow

```
Nav2 / teleop
     │
     │  /cmd_vel_raw  (geometry_msgs/Twist — raw, unlimited)
     ▼
velocity_controller.py
     │
     ├─ _cmd_callback:   clamp to Burger limits → _target_lv, _target_av
     │
     └─ _control_loop (20 Hz timer):
           1. timeout check → zero _target if stale
           2. ramp _current toward _target  (±max_acc × dt per tick)
           3. publish
     │
     │  /cmd_vel  (geometry_msgs/Twist — safe, smooth)
     ▼
OpenCR (robot hardware)
     │
     ├── left wheel motor
     └── right wheel motor
```

---

### What this node deliberately does NOT do

- Does **not** implement PID control. It does not compare the actual velocity (from `/odom`) against the target. A real PID would close the loop and compensate for disturbances (e.g., pushing the robot). That complexity is left for a future step.
- Does **not** forward `/cmd_vel` fields other than `linear.x` and `angular.z`. The Burger is a differential-drive robot — the other four fields (y, z velocity; roll, pitch rate) are always zero.
- Does **not** check for obstacle proximity before forwarding commands. That is Nav2's job via the costmap.

---

## Step 8 — Bringup Launch Files

**Goal:** Replace 6+ individual terminal commands with three composable launch files: `sensors.launch.py`, `robot.launch.py`, and `slam.launch.py`. Add `burger.yaml` to configure slam_toolbox for the Burger's hardware.

---

### High-level overview

Up to this point you ran each node by hand in a separate terminal. That works for debugging one node at a time, but it's brittle: if you open the terminals in the wrong order, forget one node, or mistype a parameter, things break silently.

Launch files solve this. A ROS2 launch file is a Python script that describes which nodes to start, with which parameters, in which order — and lets you compose (include) other launch files so you never repeat yourself.

The result of Step 8 is three commands that cover every use case:

```bash
# Test the full stack on the laptop (no hardware needed)
ros2 launch tb3_bringup robot.launch.py

# Build a map on the real robot
ros2 launch tb3_bringup slam.launch.py fake_joints:=false

# Run just the sensor processing layer
ros2 launch tb3_bringup sensors.launch.py
```

---

### Simple analogy

Think of each launch file as a **recipe card** that references other recipe cards.

- `sensors.launch.py` is "make the sauce" (2 ingredients)
- `robot.launch.py` is "make the pasta dish" — it says *include the sauce recipe*, then adds noodles, cheese, and plating (EKF + TF + velocity controller)
- `slam.launch.py` is "make the full dinner" — it says *include the pasta recipe*, then adds dessert (slam_toolbox)

If you want to change how the sauce is made, you edit `sensors.launch.py` once and every recipe that includes it picks up the change automatically.

---

### Concrete example — what happens when you run `robot.launch.py`

```
ros2 launch tb3_bringup robot.launch.py
```

ROS2 processes this in order:

1. Reads `robot.launch.py` → sees `IncludeLaunchDescription(description.launch.py)`
2. Expands `description.launch.py`:
   - Runs `xacro turtlebot3_burger_sensors.urdf.xacro` → gets URDF XML string
   - Starts `robot_state_publisher` with that XML
   - Starts `joint_state_publisher` (because `use_joint_state_publisher=true`)
3. Expands `sensors.launch.py`:
   - Starts `odometry_publisher`
   - Starts `imu_republisher`
4. Starts `ekf_node`, `tf2_broadcaster`, `velocity_controller` directly
5. All 7 nodes are up. Total time: ~2 seconds.

Without launch files, step 2 alone requires finding the URDF path, running xacro, and passing the XML as a CLI argument by hand.

---

### The composition pattern — IncludeLaunchDescription

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(pkg, 'launch', 'sensors.launch.py')
    ),
)
```

`IncludeLaunchDescription` is a launch *action* — it tells the ROS2 launch system to open the other file, parse it, and merge its nodes into the current launch description. It is **not** a Python `import` — the included file is parsed at launch time, not at import time. This means the included file can use `LaunchConfiguration` values that are only known at runtime.

---

### Forwarding arguments between launch files

```python
# In robot.launch.py — pass fake_joints down to description.launch.py
IncludeLaunchDescription(
    PythonLaunchDescriptionSource(...),
    launch_arguments={
        'use_joint_state_publisher': LaunchConfiguration('fake_joints'),
    }.items(),
)
```

`launch_arguments={...}.items()` is the ROS2 way to pass arguments to an included launch file. It works like keyword arguments in Python, but they are resolved lazily at launch time — `LaunchConfiguration('fake_joints')` is a *substitution* object that reads the value from the command line when the launch system actually starts the nodes.

This is why argument forwarding needs `.items()` (to convert a dict to key-value pairs) and why you cannot just pass a Python string directly.

---

### IfCondition — conditional node start

In `description.launch.py`, we added:

```python
from launch.conditions import IfCondition

Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
)
```

`IfCondition` evaluates a launch substitution as a boolean. If the argument is `'true'`, the node starts. If `'false'`, the entire `Node(...)` action is skipped — the node never appears in `ros2 node list`. This is the standard ROS2 pattern for optional nodes (sim vs. real, debug vs. release).

---

### burger.yaml — the ROS2 parameter YAML convention

```yaml
slam_toolbox:           # ← must match the node name= in the launch file
  ros__parameters:      # ← this exact key is required by ROS2
    resolution: 0.05
    max_laser_range: 3.5
    ...
```

ROS2 loads parameters from YAML files by matching the top-level key against the node name. The `ros__parameters` sub-key is mandatory — it tells the parameter loader where the actual parameters begin. If either key is wrong, the parameters are silently ignored and the node uses its defaults.

The node in `slam.launch.py` is declared as `name='slam_toolbox'`, so the YAML top-level key must also be `slam_toolbox`.

---

### Key slam_toolbox parameters explained

| Parameter | Value | Why |
|---|---|---|
| `resolution` | 0.05 m | 5 cm per grid cell — fine enough for indoor navigation |
| `max_laser_range` | 3.5 m | LDS-01 hardware maximum — ignores far noisy readings |
| `minimum_travel_distance` | 0.3 m | Only process a new scan after moving 30 cm — avoids wasting CPU while still |
| `minimum_travel_heading` | 0.3 rad | Same for rotation (~17°) |
| `do_loop_closing` | true | Corrects drift when the robot revisits a location |
| `mode` | mapping | Build a new map; switch to `localization` in Step 10 |
| `transform_publish_period` | 0.02 s | Publish map→odom TF at 50 Hz |

---

### Data flow

```
burger.yaml (config)
     │
     ▼
slam.launch.py
  ├── robot.launch.py
  │     ├── description.launch.py
  │     │     ├── robot_state_publisher  (/robot_description, /tf_static)
  │     │     └── joint_state_publisher  (/joint_states)  ← laptop only
  │     ├── sensors.launch.py
  │     │     ├── odometry_publisher     (/odom)
  │     │     └── imu_republisher        (/imu/data)
  │     ├── ekf_node                     (/odometry/filtered)
  │     ├── tf2_broadcaster              (/tf: odom→base_footprint)
  │     └── velocity_controller          (/cmd_vel)
  │
  └── slam_toolbox
        subscriptions: /scan, /tf
        publishes:     /map, /tf (map→odom)
```

---

### What changed in description.launch.py (Step 8)

Added a `use_joint_state_publisher` argument (default `true`) and wrapped `joint_state_publisher` in `IfCondition`. When `robot.launch.py` passes `fake_joints:=false`, this node is skipped so it does not conflict with the OpenCR's real `/joint_states` stream.

---

### What these files deliberately do NOT do

- Do **not** start the OpenCR driver node. On the real robot the OpenCR firmware runs independently and just publishes to `/joint_states` and `/imu`. Starting it from our launch files would require the `turtlebot3_node` package (separate RPi4 setup).
- Do **not** start RViz. You can add it locally with `rviz2` if needed without baking it into the bringup files.
- Do **not** include a namespace. All topics are in the global namespace `/` — fine for a single robot.

---

## Step 9 — SLAM Map Building

**Goal:** Drive the robot manually through a real space while slam_toolbox builds a 2D occupancy grid. Save the finished map as `.pgm` + `.yaml` for use in autonomous navigation (Step 10).

**This step requires the real robot.** The laptop portion (teleop launch file, maps directory, verification test) is done first.

---

### High-level overview

SLAM stands for **Simultaneous Localisation and Mapping**. The robot needs to do two things at once that depend on each other:

- To build a map, it needs to know where it is
- To know where it is, it needs a map

slam_toolbox solves this by maintaining a *probability distribution* over both the map and the robot's pose at the same time, and updating both as new LiDAR scans arrive.

Our role in Step 9 is purely the plumbing:
- `slam.launch.py` starts everything (robot stack + slam_toolbox)
- `teleop.launch.py` gives keyboard control through the velocity controller
- We drive the robot around — slam_toolbox does the maths
- `map_saver_cli` writes the finished map to disk

---

### Simple analogy

Imagine you are blindfolded in a dark room, holding a flashlight that only illuminates 3.5 metres (the LiDAR range). You spin slowly, sketch what you see, take a step, sketch again. Each sketch overlaps with the previous one — slam_toolbox aligns the overlaps to figure out where you moved and extend the map.

When you return to a doorway you sketched earlier and recognise it from the scan pattern, you know exactly where you are — this is **loop closure**. It corrects any drift that accumulated while you were walking.

---

### Why teleop goes through the velocity controller

```
teleop_twist_keyboard
        │
        │  /cmd_vel_raw   ← remapped from default /cmd_vel
        ▼
velocity_controller
        │  clamp → ramp → timeout
        │  /cmd_vel
        ▼
     OpenCR
```

If teleop published directly to `/cmd_vel`, a fast keypress could send a velocity spike to the wheels, causing slip that corrupts the odometry the SLAM algorithm depends on. By routing through the velocity controller, every keypress is smoothed and bounded — the map stays clean.

---

### Detailed code walkthrough

#### teleop.launch.py — the remap and emulate_tty

```python
Node(
    package='teleop_twist_keyboard',
    executable='teleop_twist_keyboard',
    output='screen',
    emulate_tty=True,                            # ← gives the node a real terminal for stdin
    remappings=[('cmd_vel', '/cmd_vel_raw')],    # ← routes through velocity_controller
)
```

`emulate_tty=True` is the key to making keyboard teleop work inside `ros2 launch`. Normally, when a subprocess is launched from Python, its stdin is not connected to the terminal — it gets a pipe instead. `emulate_tty=True` tells the launch system to create a pseudo-terminal (PTY) for the node, so `teleop_twist_keyboard` can read keypresses the same way it would if run directly with `ros2 run`.

`remappings=[('cmd_vel', '/cmd_vel_raw')]` remaps the node's internal topic name `cmd_vel` to the fully qualified `/cmd_vel_raw`. This is a ROS2 node-level remap — it works regardless of what the node's source code calls the topic.

---

### Data flow during SLAM

```
OpenCR (hardware)
  ├── /joint_states ──► odometry_publisher ──► /odom ──► ekf_node
  ├── /imu          ──► imu_republisher    ──► /imu/data ──► ekf_node
  └── /scan         ──────────────────────────────────────────────────────────────────┐
                                                                                       │
ekf_node ──► /odometry/filtered ──► tf2_broadcaster ──► /tf (odom→base_footprint)     │
                                                                                       │
robot_state_publisher ──► /tf_static (base_footprint→base_scan, all fixed frames)     │
                                                                                       ▼
                                                                              slam_toolbox
                                                                                  │
                                                                    reads: /scan, /tf tree
                                                                    writes: /map (occupancy grid)
                                                                            /tf  (map→odom)
```

slam_toolbox needs the full TF chain `map → odom → base_footprint → base_scan` to know where the LiDAR is in the world. Our nodes from Steps 2–6 provide everything except `map → odom`, which slam_toolbox itself publishes as it tracks the robot's position in the growing map.

---

### Verify the stack is healthy before mapping

Before driving, confirm all nodes and topics are running correctly:

```bash
# All 8 nodes should be listed (turtlebot3_node appears twice — that is normal)
ros2 node list

# /cmd_vel should be publishing zeros at rest (velocity_controller always outputs)
ros2 topic echo /cmd_vel --once

# Critical param: must be False so plain Twist commands are accepted
ros2 param get /turtlebot3_node enable_stamped_cmd_vel

# Sensors: IMU should be ~20 Hz (LiDAR check omitted — scan is visible in RViz)
ros2 topic hz /imu
```

Expected:
- `enable_stamped_cmd_vel` → `Boolean value is: False`
- `/cmd_vel` → Twist with all zeros
- `/imu` rate → ~20 Hz

---

### On-robot procedure (when hardware is available)

**Terminal 1 (on RPi4 or over SSH):**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch tb3_bringup slam.launch.py fake_joints:=false
```

**Terminal 2 (on RPi4 or over SSH — new terminal, source first):**
```bash
source ~/ros2_ws/install/setup.bash

# NOTE: use ros2 run, NOT ros2 launch teleop.launch.py
# teleop_twist_keyboard needs a real TTY for keyboard input.
# When launched via ros2 launch over SSH, emulate_tty=True is not enough
# and the node crashes with: termios.error: (25, 'Inappropriate ioctl for device')
# ros2 run connects stdin directly to the terminal — works correctly.
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw

# Keys: i=forward  ,=back  j=left  l=right  k=stop
# q/z = increase/decrease all speeds by 10%
```

**Terminal 3 (laptop) — watch the map build in RViz:**
```bash
rviz2
# Add: Map → topic /map
# Add: RobotModel
# Add: LaserScan → topic /scan
# Fixed Frame: map
```

**Driving strategy for a clean map:**
1. Drive slowly (50% of max speed: `z` key twice)
2. Cover every area twice — once forward, once return
3. Close every loop: return to doorways and junctions you have already mapped
4. Avoid sharp turns at full speed (wheel slip = odometry error = map tear)

**Save the map when done:**
```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
# Creates: my_map.pgm  (grayscale image: white=free, black=wall, grey=unknown)
#          my_map.yaml (metadata: resolution, origin, thresholds)
```

---

### What the saved map files contain

`my_map.pgm` — a greyscale image where each pixel is one grid cell (0.05 m × 0.05 m):
- White (255) = free space (LiDAR confirmed no obstacle)
- Black (0) = occupied (LiDAR hit a wall)
- Grey (205) = unknown (LiDAR never reached there)

`my_map.yaml`:
```yaml
image: my_map.pgm
resolution: 0.05        # metres per pixel — must match burger.yaml
origin: [-x, -y, 0.0]  # map frame origin in world coordinates
negate: 0
occupied_thresh: 0.65   # pixels darker than this = occupied
free_thresh: 0.196      # pixels lighter than this = free
```

Nav2 (Step 10) loads both files. The `.yaml` tells Nav2 how to interpret the pixel values and where to place the map origin in the world frame.

---

### What this step deliberately does NOT do

- Does **not** save the SLAM graph (keyframes + constraints). `map_saver_cli` saves only the rasterised occupancy grid, not the internal slam_toolbox representation. To resume mapping later, use `serialize_map_saver` instead.
- Does **not** tune slam_toolbox for the specific room. The `burger.yaml` parameters are good defaults; room-specific tuning (scan buffer size, loop closure thresholds) can be done later.

*More steps to be added as the project progresses.*
