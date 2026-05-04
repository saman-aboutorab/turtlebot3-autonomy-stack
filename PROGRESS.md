# TurtleBot3 Autonomy Stack — Progress Journal

> A running record of every significant error, debugging session, and resolution.
> Written in the style of a lab notebook: what was observed, what was hypothesised,
> what was tried, what worked, and what was learned.

---

## Format

Each entry follows this structure:

```
### [STAGE-STEP] Short title
Date: YYYY-MM-DD
Symptom: What the error looked like
Hypothesis: What we thought was causing it
Root cause: What was actually causing it
Fix: What we changed
Lesson: What rule to carry forward
```

---

# Stage 1 — Foundation

---

### [1-2a] URDF xacro expansion failure — illegal `--` in XML comments

**Date:** 2026-03-16
**File:** `hardware/urdf/turtlebot3_burger_sensors.urdf.xacro`

**Symptom:**
```
XML parsing error: not well-formed (invalid token): line 62, column 9
xacro.XacroException: No such file or directory ...
```
Running `xacro hardware/urdf/turtlebot3_burger_sensors.urdf.xacro` failed before producing any output.

**Hypothesis:**
Initially suspected a character encoding issue or a missing xacro namespace declaration.

**Root cause:**
The XML specification forbids the sequence `--` anywhere inside an XML comment (because `-->` ends the comment, and `--` is syntactically ambiguous). Our comment block used separator lines like:
```xml
<!--
     ----------------------------------------------------------------
-->
```
Each of those `--------` lines contains multiple `--` sequences, which is invalid XML regardless of how deeply nested inside a comment block they are.

**Fix:**
Replaced all `---...---` separator lines inside comments with `===...===`:
```bash
sed -i 's/       ----------------------------------------------------------------/       ================================================================/g' hardware/urdf/...
```

**Verified with:**
```bash
xacro file.urdf.xacro > /tmp/expanded.urdf
check_urdf /tmp/expanded.urdf
# → Successfully Parsed XML
```

**Lesson:**
Inside XML comments (`<!-- ... -->`), the sequence `--` is illegal — even in separator lines. Use `=`, `~`, or spaces as dividers. Always validate a URDF with `check_urdf` before running it in a launch file.

---

### [1-2b] URDF not found at launch — hardware/ directory not installed

**Date:** 2026-03-16
**File:** `tb3_bringup/launch/description.launch.py`

**Symptom:**
```
FileNotFoundError: No such file or directory:
'.../install/tb3_bringup/hardware/urdf/turtlebot3_burger_sensors.urdf.xacro'
```

**Hypothesis:**
The launch file used a relative path from `__file__` to reach `hardware/urdf/`. After `colcon build`, `__file__` resolves to a path inside `install/`, not the workspace source.

**Root cause:**
Two compounding issues:

1. **Path resolution was wrong.** The launch file used `os.path.dirname(__file__)` and navigated with `'..', '..', '..'` back to the workspace root. After installation, `__file__` is `install/tb3_bringup/share/tb3_bringup/launch/description.launch.py`. The relative path from there to `hardware/` is not the same as from the source tree.

2. **`hardware/` was never installed.** The `hardware/` directory has no `package.xml`, so it is not a ROS2 package and is not automatically installed anywhere. `colcon` only installs what you explicitly tell it to. Without an `install()` directive, `hardware/` only exists in the source tree.

**Fix:**

Step 1 — Add an `install()` directive to `tb3_bringup/CMakeLists.txt`:
```cmake
install(DIRECTORY ../hardware
  DESTINATION share/${PROJECT_NAME}
)
```
This copies `hardware/` into `install/tb3_bringup/share/tb3_bringup/hardware/` during `colcon build`.

Step 2 — Replace the fragile relative path with the correct ROS2 API:
```python
# Before (wrong):
pkg_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'hardware', ...)

# After (correct):
pkg_share = get_package_share_directory('tb3_bringup')
default_urdf = os.path.join(pkg_share, 'hardware', 'urdf', 'turtlebot3_burger_sensors.urdf.xacro')
```

`get_package_share_directory('tb3_bringup')` always returns `install/tb3_bringup/share/tb3_bringup/` regardless of where the workspace lives on disk or how it was built.

**Lesson:**
Any file that a launch file needs at runtime must be explicitly installed via `install(DIRECTORY ...)` or `install(FILES ...)` in `CMakeLists.txt`. Never use `__file__`-relative paths in launch files. Always use `get_package_share_directory()`. If you need to ship data files (URDF, config, models) with a package, that package's `CMakeLists.txt` must install them.

---

### [1-2c] Launch crash — YAML parser rejects URDF string as robot_description parameter

**Date:** 2026-03-16
**File:** `tb3_bringup/launch/description.launch.py`

**Symptom:**
```
ERROR [launch]: Caught exception in launch (see debug for traceback):
Unable to parse the value of parameter robot_description as yaml.
If the parameter is meant to be a string, try wrapping it in
launch_ros.parameter_descriptions.ParameterValue(value, value_type=str)
```

**Hypothesis:**
The `Command(['xacro ', ...])` substitution was producing the URDF XML string correctly. The error was happening *after* that, during parameter assignment.

**Root cause:**
In ROS2 Jazzy (and patched Humble), the launch system tries to auto-detect the type of every parameter value by attempting to YAML-parse it. A URDF string starts with `<?xml version="1.0"?>` which is not valid YAML, so the parser throws an exception before the node even starts.

**Fix:**
Wrap the `Command()` substitution in `ParameterValue` with an explicit `value_type=str`:
```python
from launch_ros.parameter_descriptions import ParameterValue

'robot_description': ParameterValue(
    Command(['xacro ', LaunchConfiguration('urdf_file')]),
    value_type=str
)
```
`ParameterValue(..., value_type=str)` tells the launch system "this is a string, do not try to YAML-parse it."

**Lesson:**
In ROS2 Jazzy and recent Humble, any parameter whose value could be mistaken for non-string YAML (XML strings, multi-line strings, strings that start with `{` or `[`) must be wrapped in `ParameterValue(..., value_type=str)`. This is particularly common for `robot_description`.

---

### [1-2d] `ros2 run` cannot find node — Python script not executable

**Date:** 2026-03-16
**File:** `tb3_odometry/tb3_odometry/tf2_checker.py`

**Symptom:**
```
saman@robot:~$ ros2 run tb3_odometry tf2_checker.py
No executable found
```

**Hypothesis:**
The script was registered in `install(PROGRAMS ...)` in `CMakeLists.txt` and `colcon build` completed successfully. Suspected a typo in the script name.

**Root cause:**
With `--symlink-install`, `install(PROGRAMS ...)` creates a symlink from `install/tb3_odometry/lib/tb3_odometry/tf2_checker.py` back to the source file `tb3_odometry/tb3_odometry/tf2_checker.py`. The OS executes the symlink target — the source file. The source file had permissions `-rw-rw-r--` (no execute bit), so the OS refused to run it.

Without `--symlink-install`, colcon *copies* the file and sets the execute bit during the copy. With `--symlink-install`, the source file's permissions are used as-is.

**Fix:**
```bash
chmod +x tb3_odometry/tb3_odometry/tf2_checker.py
```
No rebuild needed — the symlink already points to the source file.

**Lesson:**
**Every Python node source file must have `chmod +x` set before it will work with `ros2 run` under `--symlink-install`.** This is a permanent rule for this project. Apply it immediately after creating a new `.py` node file, before registering it in `CMakeLists.txt`.

The file mode change is tracked by git:
```
mode change 100644 => 100755 tb3_odometry/tb3_odometry/tf2_checker.py
```

---

---

### [1-8a] Real robot: `KeyError: 'LDS_MODEL'` on launch

**Date:** 2026-03-24
**File:** turtlebot3_bringup system environment

**Symptom:**
```
KeyError: 'LDS_MODEL'
```
Running `ros2 launch tb3_bringup robot.launch.py fake_joints:=false` crashed immediately.

**Root cause:**
`turtlebot3_bringup` reads the `LDS_MODEL` environment variable to select the LiDAR driver. It was not set in `.bashrc` on the RPi4.

**Fix:**
```bash
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc && source ~/.bashrc
```

**Lesson:**
When setting up a new RPi4, always add `LDS_MODEL=LDS-01` (and `TURTLEBOT3_MODEL=burger`) to `.bashrc` before running any turtlebot3_bringup commands.

---

### [1-8b] Real robot: OpenCR `Failed connection with Devices`

**Date:** 2026-03-24
**File:** OpenCR firmware

**Symptom:**
turtlebot3_ros started but immediately logged:
```
Failed connection with Devices
```
OpenCR LEDs: rhythmic blinking red pattern (not solid).

**Hypothesis:**
Serial port permission issue or wrong baud rate.

**Root cause:**
OpenCR was not flashed with the TurtleBot3 firmware. The rhythmic blinking red LED pattern (not a solid LED) indicates the OpenCR firmware could not connect to the Dynamixel motors, meaning it was running generic OpenCR firmware rather than the TurtleBot3-specific firmware.

**Fix:**
```bash
# On RPi4
wget https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xjf opencr_update.tar.bz2
cd opencr_update
./update.sh /dev/ttyACM0 burger.opencr
```

**Lesson:**
OpenCR must be flashed with the TurtleBot3 firmware before first use. Solid LEDs = firmware connected to motors. Rhythmic blinking red = firmware did not connect to Dynamixel motors (wrong firmware or hardware fault).

---

### [1-8c] Real robot: packages not found on RPi4 after clone

**Date:** 2026-03-24
**File:** git repository

**Symptom:**
```
ignoring unknown package 'tb3_bringup'
colcon build: Failed to find tb3_navigation/share/tb3_navigation/package.sh
```

**Root cause:**
RPi4 had cloned the `main` branch. All custom packages (`tb3_bringup`, `tb3_odometry`, `tb3_navigation`) are on the `stage-1` branch, which had never been pushed to GitHub until this session.

**Fix:**
```bash
# On RPi4
git checkout stage-1
git pull origin stage-1
colcon build --symlink-install --packages-select tb3_navigation tb3_bringup tb3_odometry
```
The explicit `--packages-select` is needed because `tb3_navigation` (a placeholder for Step 10) has no source yet and would fail a full build.

**Lesson:**
Always confirm the correct branch is checked out on the robot. After any `git pull`, rebuild with `colcon build --symlink-install`.

---

### [1-8d] Real robot: turtlebot3_ros crash — `'opencr.id' must be initialized`

**Date:** 2026-03-24
**File:** `tb3_bringup/launch/hardware.launch.py`

**Symptom:**
```
UninitializedStaticallyTypedParameterException: 'opencr.id' must be initialized before it can be gotten
```
turtlebot3_ros crashed immediately after starting.

**Root cause:**
`hardware.launch.py` was not passing the `burger.yaml` parameter file. Without it, turtlebot3_ros has no hardware configuration and cannot initialise the OpenCR parameters. Also, the serial port was passed as a ROS parameter instead of a CLI argument.

**Fix:**
```python
turtlebot3_node = Node(
    ...
    parameters=[tb3_params, {...}],   # burger.yaml must be first
    arguments=['-i', '/dev/ttyACM0'], # serial port is a CLI arg, not a ROS param
)
```

**Lesson:**
`turtlebot3_ros` requires `burger.yaml` as a parameter file AND takes the serial port via `-i` CLI argument. Both are required. The serial port is not a ROS parameter.

---

### [1-8e] Real robot: turtlebot3_ros crash — `'namespace' must be initialized`

**Date:** 2026-03-24
**File:** `tb3_bringup/launch/hardware.launch.py`

**Symptom:**
```
UninitializedStaticallyTypedParameterException: 'namespace' must be initialized
```

**Root cause:**
Jazzy's version of `turtlebot3_ros` requires `namespace` as a statically-typed parameter. It is not present in `burger.yaml`.

**Fix:**
Add inline override in `hardware.launch.py`:
```python
{'namespace': ''}
```

**Lesson:**
In ROS2 Jazzy, `turtlebot3_ros` requires several params not present in `burger.yaml`. They must be set explicitly as inline overrides in the launch file.

---

### [1-8f] Real robot: turtlebot3_ros crash — `'odometry.frame_id' must be initialized`

**Date:** 2026-03-24
**File:** `tb3_bringup/launch/hardware.launch.py`, `turtlebot3_bringup/param/burger.yaml`

**Symptom:**
```
UninitializedStaticallyTypedParameterException: 'odometry.frame_id' must be initialized
```

**Root cause:**
`burger.yaml` places odometry parameters under `diff_drive_controller`, not under `turtlebot3_node`. `turtlebot3_ros` never receives them from the file and requires them explicitly.

Additionally, `burger.yaml` sets `enable_stamped_cmd_vel: true`, which makes `turtlebot3_ros` expect `geometry_msgs/TwistStamped` on `/cmd_vel`. Our `velocity_controller` publishes plain `geometry_msgs/Twist`. Commands were silently discarded.

**Fix:**
Add all required inline overrides in `hardware.launch.py`:
```python
{
    'namespace':               '',
    'odometry.frame_id':       'odom',
    'odometry.child_frame_id': 'base_footprint',
    'odometry.use_imu':        True,
    'odometry.publish_tf':     False,   # our tf2_broadcaster owns odom→base_footprint
    'enable_stamped_cmd_vel':  False,   # velocity_controller publishes plain Twist
}
```

**Lesson:**
`burger.yaml` cannot be trusted to provide all params that `turtlebot3_ros` needs in Jazzy. Always check which params are statically typed and override them inline. `enable_stamped_cmd_vel` must be `False` if your velocity source publishes `geometry_msgs/Twist`.

---

### [1-9a] Real robot: wrong teleop node used, TwistStamped conflict on `/cmd_vel`

**Date:** 2026-03-24

**Symptom:**
Robot did not move when driving with `turtlebot3_teleop teleop_keyboard`. Echo of `/cmd_vel` failed:
```
Cannot echo topic '/cmd_vel', as it contains more than one type:
[geometry_msgs/msg/Twist, geometry_msgs/msg/TwistStamped]
```

**Root cause:**
Two problems:
1. `turtlebot3_teleop teleop_keyboard` publishes `TwistStamped` directly to `/cmd_vel` — bypassing our `velocity_controller` entirely and using the wrong message type (we set `enable_stamped_cmd_vel: false`).
2. This created a type conflict on `/cmd_vel`: velocity_controller published `Twist`, turtlebot3_teleop published `TwistStamped`. `turtlebot3_ros` could not accept either reliably.

**Fix:**
Use the correct teleop node:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw
```
This routes through `velocity_controller` and publishes plain `Twist`.

**Lesson:**
Never use `turtlebot3_teleop teleop_keyboard` in this stack. Always use `teleop_twist_keyboard` remapped to `/cmd_vel_raw`.

---

### [1-9b] Real robot: `teleop.launch.py` crashes over SSH — termios error

**Date:** 2026-03-24

**Symptom:**
```
termios.error: (25, 'Inappropriate ioctl for device')
[ERROR] [teleop_twist_keyboard-1]: process has died [pid ..., exit code 1]
```
Running `ros2 launch tb3_bringup teleop.launch.py` over SSH failed immediately.

**Root cause:**
`teleop_twist_keyboard` reads keyboard input via `termios` (Unix terminal I/O). When launched via `ros2 launch` over SSH, the node's stdin is a pipe, not a real TTY. `emulate_tty=True` in the launch file is intended to fix this but does not work reliably over SSH connections.

**Fix:**
Run the node directly with `ros2 run` instead of through the launch file:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_raw
```
`ros2 run` connects stdin directly to the terminal — keyboard input works correctly.

**Lesson:**
`teleop_twist_keyboard` cannot be launched via `ros2 launch` over SSH even with `emulate_tty=True`. Always use `ros2 run` with an explicit remap when operating the robot over SSH.

---

### [1-9d] slam_toolbox crashes on aarch64 (RPi4) — switched to cartographer

**Date:** 2026-03-24
**Package:** ros-jazzy-slam-toolbox 2.8.3-1noble.20260125.011542 (arm64)

**Symptom:**
`async_slam_toolbox_node` (and `sync_slam_toolbox_node`) crash silently immediately after startup. With a params file, the node prints exactly one line then exits:
```
[INFO] [slam_toolbox]: Node using stack size 40000000
```
With no params file, it prints nothing and exits. No error message is produced. `ps aux` confirms the process is gone.

**Diagnosis steps taken:**
- Confirmed not a TF chain issue (`/tf` odom→base_footprint was publishing correctly)
- Confirmed not a timing issue (slam_toolbox started 27 seconds after full robot stack was up)
- Confirmed not a parameter loading issue (crash happens even with no `--params-file`)
- Confirmed not a memory issue (3.0 GiB available)
- `enable_interactive_mode: false` had no effect
- Both `async_slam_toolbox_node` and `sync_slam_toolbox_node` crash identically
- Reinstalling `ros-jazzy-slam-toolbox` did not fix it

**Root cause:**
Binary-level crash in the ros-jazzy-slam-toolbox arm64 apt package on Ubuntu 24.04 (Raspberry Pi 4, aarch64). Most likely a Ceres solver initialization bug in the packaged binary for this architecture. No fix available without building slam_toolbox from source.

**Resolution:**
Switched to **cartographer** (`ros-jazzy-cartographer` + `ros-jazzy-cartographer-ros`), which is also recommended by the official TurtleBot3 Jazzy documentation and is known stable on arm64.

**Lesson:**
The ros-jazzy-slam-toolbox arm64 binary may be broken on RPi4. If slam_toolbox crashes after one line with no error output, switch to cartographer — it is a fully equivalent SLAM solution for 2D mapping and is the official TurtleBot3 recommendation.

---

### [1-9c] Workspace not sourced in new terminals

**Date:** 2026-03-24

**Symptom:**
```
Package 'tb3_bringup' not found: "package 'tb3_bringup' not found, searching: ['/opt/ros/jazzy']"
```
Any `ros2 launch` or `ros2 run` command with a custom package fails in a new terminal.

**Root cause:**
`install/setup.bash` must be sourced in every new terminal. Only the system ROS installation is sourced by default.

**Fix:**
Either source manually in each terminal:
```bash
source ~/ros2_ws/install/setup.bash
```
Or add to `.bashrc` for automatic sourcing:
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

**Lesson:**
Always `source install/setup.bash` before running any custom package commands. Adding it to `.bashrc` eliminates this permanently.

---

### [1-9e] Real robot: `hlds_laser_publisher` runs but `/scan` publishes nothing — UNRESOLVED

**Date:** 2026-03-24

**Symptom:**
Cartographer built zero submaps even after moving the robot. Traced to `/scan` never receiving any messages despite `hlds_laser_publisher` being in the node list.
```
ros2 topic hz /scan          # shows nothing
ros2 topic echo /scan        # hangs indefinitely
/submap_list: submap: []     # even after driving around
```

**Diagnosis steps taken (do not repeat these):**

1. **Confirmed cartographer was not the problem** — `ros2 topic info /scan` showed publisher count 1, subscriber count 2, QoS compatible. Cartographer was subscribed and waiting. Submap list empty confirmed empty root cause is upstream.

2. **Confirmed TF chain is fine** — `ros2 run tf2_ros tf2_echo base_footprint base_scan` returned the transform `[-0.032, 0.000, 0.182]` immediately. Cartographer can look up sensor location.

3. **`ros2 topic hz /scan` appearing empty is normal — QoS mismatch** — `ros2 topic hz` and `ros2 topic echo` default to RELIABLE QoS. The LiDAR publishes BEST_EFFORT. Use:
   ```bash
   ros2 topic echo --qos-profile sensor_data /scan --no-arr
   ```
   Even with this, no messages appeared.

4. **Hardware confirmed working** — LiDAR visibly spinning. `/dev/ttyUSB0` present. User in `dialout` group. Serial data confirmed flowing at 230400 baud:
   ```bash
   sudo stty -F /dev/ttyUSB0 230400 raw && sudo timeout 3 cat /dev/ttyUSB0 | xxd | head -5
   # → hex bytes confirmed, data IS streaming from hardware
   ```

5. **No duplicate processes** — `ps aux | grep hlds` showed exactly one `hlds_laser_publisher` process. No port contention.

6. **`ros2 param dump /hlds_laser_publisher` returns empty** — this is normal. `hls_lfcd_lds_driver` does not expose its parameters through the ROS2 param server even though they are passed via `--params-file`. Confirmed correct params via `cat /tmp/launch_params_*.`: `port: /dev/ttyUSB0`, `frame_id: base_scan`.

7. **Standalone run shows driver initialises then goes silent:**
   ```
   [INFO] [hlds_laser_publisher]: Init hlds_laser_publisher Node Main
   [INFO] [hlds_laser_publisher]: port : /dev/ttyUSB0 frame_id : base_scan
   # (nothing more — driver blocking in read loop, no /scan published)
   ```

8. **No 0xFA at any baud rate** — performed full baud rate scan (57600, 115200, 230400, 460800, 921600). All returned 1000 bytes quickly with zero 0xFA occurrences. Also confirmed:
   - Official `turtlebot3_bringup robot.launch.py` produces the same result (not our stack)
   - Driver version: `ros-jazzy-hls-lfcd-lds-driver` 2.1.1
   - `lsof /dev/ttyUSB0` confirmed driver has the port open (fd 21)
   - `brltty` is inactive (not a serial port hijack)
   - USB device confirmed as Silicon Labs CP2102 (VID 10c4, PID ea60) — correct chip for LDS-01
   - `pyserial` confirmed 0xFA at zero positions at 230400 baud in 500 bytes
   - Sending `b` (LDS-01 start command) had no effect

**ROBOTIS diagnostic run (their requested commands):**
```
ls /dev/ttyUSB*           → /dev/ttyUSB0  ✓
sudo stty -F /dev/ttyUSB0 230400 raw
sudo timeout 5 cat /dev/ttyUSB0 | xxd | head -30
```
Output showed 480 bytes with zero `0xFA`. ROBOTIS expected `fa a0 XX...` at every row. The data is all garbage bytes (0xFE, 0x98, 0x80, 0x66, 0x7E — never 0xFA). This output was forwarded to ROBOTIS for their assessment.

**Root cause:** Most likely a **broken internal UART connection** between the LiDAR sensor PCB and its USB board (CP2102). A floating or disconnected UART TX line generates electrical noise, which the CP2102 forwards as random bytes to USB — explaining why data flows at all baud rates but never contains a valid 0xFA packet header. The LiDAR motor spins independently of the serial interface, so the motor being active does not confirm the data path is intact.

**Status:** HARDWARE FAULT — not a software issue. The driver, our launch files, and the stack are all correct.

**Next steps (start here next session):**

```bash
# 1. Physical: inspect and reseat the flat ribbon/connector cable
#    between the rotating LiDAR sensor module and the LiDAR base board (USB side).
#    This cable is inside the LiDAR unit — check it is fully seated.

# 2. After reseating, test with:
sudo pkill -9 -f hlds_laser 2>/dev/null
python3 -c "
import serial
s = serial.Serial('/dev/ttyUSB0', 230400, timeout=5)
s.flushInput()
s.write(b'b')
data = s.read(500)
s.close()
positions = [i for i,b in enumerate(data) if b == 0xFA]
print('0xFA count:', len(positions), '— expect ~11 for a healthy LiDAR')
"

# 3. If still no 0xFA after reseating, the LiDAR unit needs to be replaced.
#    Replacement options:
#      - LDS-01 (original): available from ROBOTIS or robotis.com (~$30-50)
#      - RPLIDAR A1M8: compatible with nav2, driver available as ros-jazzy-rplidar-ros
#      - YDLIDAR X4: compatible, driver available as ros-jazzy-ydlidar-ros2-driver
```

**If replacing with a different LiDAR model:**
Our `hardware.launch.py` uses `hls_lfcd_lds_driver` hardcoded. A different LiDAR needs a different driver node and package. Update `hardware.launch.py` accordingly and keep `frame_id: base_scan` to preserve the TF chain.

---

### [1-9f] Real robot: LDS-03 hardware fault confirmed — unit DOA

**Date:** 2026-05-04

**Symptom:**
After mounting the ROBOTIS LDS-03 (replacement for the faulty LDS-01) and connecting
its USB2LDS adapter to the RPi4, the serial test returned zero bytes:
```
Total bytes received: 0
No data — hardware fault confirmed
```

**Diagnosis:**
LDS-03 specs: Tx-only USART, 115200 baud, should auto-transmit on power-up.
No command needed — data should flow immediately.
`ls /dev/ttyUSB0` confirmed the USB2LDS adapter (CP2102) was detected by the OS.
Test was run with the LiDAR fully assembled and connected; photo earlier in session
showed pieces unconnected but the actual test was run with everything seated.

**Root cause:**
Unit received was hardware DOA. Zero bytes at 115200 baud despite the CP2102 being
enumerated and `/dev/ttyUSB0` present.

**Resolution:**
Replaced with Slamtec RPLIDAR C1. See [1-9g].

**Lesson:**
LDS-03 is Tx-only — no wake-up command is possible or needed. If zero bytes are
returned with `/dev/ttyUSB0` present and the unit powered, the hardware is faulty.

---

### [1-9g] Real robot: RPLIDAR C1 — driver setup and working configuration

**Date:** 2026-05-04

**Hardware:**
Slamtec RPLIDAR C1. USB chip: STM32 Virtual COM Port (VID 0483:5740) — appears as
`/dev/ttyACM0` is the OpenCR, `/dev/ttyUSB0` is the RPLIDAR C1 via its CP2102 adapter.

**Baud rate discovery:**
LDS-01 and LDS-03 used 115200/230400 baud. The C1 uses **460800 baud**.
Confirmed via GET_INFO command sweep:
```python
s.write(b'\xA5\x50')  # GET_INFO command
# Baud 115200: 0 bytes
# Baud 460800: 27 bytes — a5 5a 14 00 00 00 04 ...  ← correct response header
# Baud 256000: 0 bytes
```
`a5 5a` is the RPLIDAR response descriptor — confirms hardware alive and communicating.

**Driver:**
The apt package `ros-jazzy-rplidar-ros` (v2.1.0) provides `rplidar_composition`
(not `rplidar_node` — the executable name changed in v2.x):
```bash
ros2 pkg executables rplidar_ros   # → rplidar_ros rplidar_composition
```

**Scan mode issue:**
`rplidar_composition` with no scan_mode or with `scan_mode:=Standard` both failed:
```
[ERROR]: Cannot start scan: '80008002'  (RESULT_OPERATION_TIMEOUT)
[ERROR]: Cannot start scan: '80008000'  (RESULT_INVALID_DATA)
```
Root cause: SDK 1.12.0 calls the deprecated `checkExpressScanSupported()` API first,
which the C1 doesn't support. Setting `angle_compensate:=true` without a `scan_mode`
param allows the driver to fall through to auto-mode selection, which picks "Standard"
correctly.

**Working command:**
```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/ttyUSB0 \
  -p serial_baudrate:=460800 \
  -p frame_id:=base_scan \
  -p angle_compensate:=true
```
Output:
```
current scan mode: Standard, max_distance: 16.0 m, Point number: 2.1K, angle_compensate: 1
```
`/scan` publishing at **10.0 Hz** confirmed.

**Changes made to codebase:**
- `tb3_bringup/launch/hardware.launch.py`: replaced `hls_lfcd_lds_driver`/`hlds_laser_publisher`
  with `rplidar_ros`/`rplidar_composition`. Old driver blocks kept as commented reference.
- `tb3_bringup/config/burger_cartographer.lua`: updated range limits
  (min 0.12→0.05, max 3.5→12.0, missing_data_ray_length 3.0→10.0).
  Old LDS-01 values kept as commented reference.

**Lesson:**
- RPLIDAR C1 baud rate is 460800, not 115200.
- The executable in rplidar_ros v2.x is `rplidar_composition`, not `rplidar_node`.
- Do not set `scan_mode` for C1 with SDK 1.12.0 — let the driver auto-select.
  Setting `angle_compensate:=true` is required for correct cartographer geometry.
- Keep `frame_id: base_scan` to preserve the TF chain regardless of LiDAR model.
