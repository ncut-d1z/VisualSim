# VisualSim

A small standalone visual simulation data generator derived from the trajectory
and EuRoC-output ideas used in
[KaiserKatze/ros2-vio-node/src/VisualSim](https://github.com/KaiserKatze/ros2-vio-node/tree/master/src/VisualSim).

It generates two deterministic monocular datasets for validating visual
odometry and spherical-flow algorithms.

## Scenarios

### 1. Forward constant-velocity translation

The camera orientation is fixed and the camera translates along its initial
optical axis (+Z):

```text
R_wc(t) = R_wc(0)
p_w(t)  = p_w(0) + v t R_wc(0) e_z
omega_c = 0
a_w     = 0
```

Default parameters:

- duration: 8 s
- camera rate: 20 Hz
- speed: 0.25 m/s
- initial pose: identity / origin

### 2. Constant pure rotation in a rectangular room

The camera center is fixed while the camera rotates around a user-specified
axis in the initial camera frame at constant angular velocity:

```text
p_w(t)  = p_w(0)
R_wc(t) = R_wc(0) Exp((omega t a_c)^)
v_w     = 0
omega_c = omega a_c
```

Default parameters:

- duration: 8 s
- camera rate: 20 Hz
- angular velocity: 0.35 rad/s
- rotation axis: (0, 0, 1), normalized internally
- initial pose: identity / origin

The pure-rotation scene uses the rectangular, densely sampled room from the
reference implementation in
[KaiserKatze/ros2-vio-node/src/VisualSim/Room.hpp](https://github.com/KaiserKatze/ros2-vio-node/blob/master/src/VisualSim/Room.hpp):
10 m depth, 10 m width, 3 m height, with a 0.5 m boundary grid. The camera is
placed at the room center, so the image contains wall, floor, and ceiling
texture rather than only a few front-facing planes.

## Output

Running the generator creates:

```text
datasets/
  forward_constant_velocity/
    motion.yaml
    mav0/
      cam0/
        sensor.yaml
        data.csv
        data/*.png
      imu0/
        sensor.yaml
        data.csv
      state_groundtruth_estimate0/
        data.csv
      README.txt
  optical_axis_constant_rotation/
    motion.yaml
    mav0/
      cam0/
        sensor.yaml
        data.csv
        data/*.png
      state_groundtruth_estimate0/
        data.csv
      README.txt
```

Each dataset also contains a top-level `motion.yaml`. It provides
dataset-level motion ground truth, including the scenario name, duration,
camera rate, constant-kinematics flag, true linear velocity in the world frame,
true linear acceleration in the world frame, true angular velocity in the
camera frame, and the initial/final poses. For example, the two main vector
entries have the form:

```yaml
kinematics:
  constant: true
  true_linear_velocity:
    frame: world
    unit: m/s
    value: [0, 0, 0.25]
  true_angular_velocity:
    frame: camera
    unit: rad/s
    value: [0, 0, 0]
```

The per-frame ground-truth CSV contains timestamp, position, quaternion, linear
velocity, linear acceleration, and body/camera-frame angular velocity.  The
generated `imu0/data.csv` is EuRoC-compatible and contains noiseless gyroscope
and specific-force samples at 200 Hz, so the dataset can be replayed by an
IMU-fused MSCKF without adding machine-specific paths.

The forward-translation images are rendered from a deterministic asymmetric
3-D landmark field on three depth planes. The pure-rotation images use the
deterministic rectangular-room boundary grid described above. Together these
scenes produce parallax in the forward-translation sequence and pure
rotational image motion in the rotation sequence.

## Build

Dependencies:

- CMake >= 3.20
- C++20 compiler
- Eigen3
- OpenCV 4 (core, imgproc, imgcodecs)

Example on Ubuntu:

```bash
sudo apt install build-essential cmake libeigen3-dev libopencv-dev
cmake -S . -B build
cmake --build build -j
ctest --test-dir build --output-on-failure
./build/visual_sim ./datasets \
  --angular-velocity-rad-per-sec 0.35 \
  "--rotation-axis=(0,0,1)"
```

### Windows 11 + CMake + vcpkg

The Windows build environment follows
`ncut-d1z/vis-spherical-flow`:

- Windows 11 x64
- Visual Studio 18 2026 with the C++ desktop workload
- CMake >= 3.25
- vcpkg manifest mode
- `x64-windows` triplet
- `VCPKG_ROOT` pointing to the vcpkg installation

In an **x64 Developer PowerShell**:

```powershell
git clone https://github.com/microsoft/vcpkg C:\dev\vcpkg
C:\dev\vcpkg\bootstrap-vcpkg.bat -disableMetrics
$env:VCPKG_ROOT = 'C:\dev\vcpkg'

git clone https://github.com/ncut-d1z/VisualSim.git
cd VisualSim
.\build.ps1
```

`build.ps1` runs the `windows` configure preset, builds the Release
configuration with `--clean-first`, and then runs CTest. The executable is:

```text
out/build/Release/visual_sim.exe
```

Generate both datasets with the one-click runner:

```powershell
.\run.ps1
```

`run.ps1` checks for `out/build/Release/visual_sim.exe`. If the executable
does not exist, it automatically invokes `build.ps1`, then runs the simulator
and prints the absolute dataset output path.

A custom output directory can also be supplied:

```powershell
.\run.ps1 -OutputDir D:\VisualSimData
```

The pure-rotation angular velocity and axis can be selected from the command
line. Use exactly one angular-velocity option; the axis is normalized after
parsing:

```powershell
.\build\Release\visual_sim.exe D:\VisualSimData `
  --angular-velocity-rad-per-sec 0.50 `
  "--rotation-axis=(1,-2,4)"

.\build\Release\visual_sim.exe D:\VisualSimData `
  --angular-velocity-deg-per-sec 20 `
  "--rotation-axis=(1,2,3)"
```

PowerShell requires quotes around the parenthesized option. The generated
`optical_axis_constant_rotation/motion.yaml` records the normalized angular
velocity vector and the room-centered initial pose for reproducibility.

To remove the complete CMake build tree:

```powershell
.\clean.ps1
```

The preset follows the reference project and uses
`D:/AppData/vcpkg_installed/VisualSim` as `VCPKG_INSTALLED_DIR`. If the
machine does not use that drive/layout, edit or remove that cache variable in
`CMakePresets.json`; vcpkg will then use its normal manifest-mode installed
location.

The vcpkg manifest uses the same pinned baseline as `vis-spherical-flow` and
installs only the dependencies needed here: Eigen3 and OpenCV 4 with JPEG/PNG
image-codec support.

## Coordinate convention

The synthetic pinhole camera uses the conventional computer-vision frame:

- +X: image right
- +Y: image down
- +Z: optical axis / forward

`R_wc` rotates vectors from camera coordinates into world coordinates.

