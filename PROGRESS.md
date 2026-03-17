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

*New entries to be added as the project progresses through Steps 4–12 and beyond.*
