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

### 2. Constant rotation around the optical axis

The camera center is fixed while the camera rolls around its own +Z optical
axis at constant angular velocity:

```text
p_w(t)  = p_w(0)
R_wc(t) = R_wc(0) Exp((omega t e_z)^)
v_w     = 0
omega_c = [0, 0, omega]^T
```

Default parameters:

- duration: 8 s
- camera rate: 20 Hz
- angular velocity: 0.35 rad/s
- initial pose: identity / origin

The phrase "沿着光轴做匀速圆周运动" is implemented as **constant rotation
about the optical axis with a fixed camera center**, matching the stated
constraint "相机位置不变".

## Output

Running the generator creates:

```text
datasets/
  forward_constant_velocity/
    mav0/
      cam0/
        sensor.yaml
        data.csv
        data/*.png
      state_groundtruth_estimate0/
        data.csv
      README.txt
  optical_axis_constant_rotation/
    mav0/
      cam0/
        sensor.yaml
        data.csv
        data/*.png
      state_groundtruth_estimate0/
        data.csv
      README.txt
```

Ground truth contains timestamp, position, quaternion, linear velocity,
linear acceleration, and body/camera-frame angular velocity.

The images are rendered from a deterministic asymmetric 3-D landmark field on
three depth planes. This produces parallax in the forward-translation sequence
and pure rotational image motion in the optical-axis sequence.

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
./build/visual_sim ./datasets
```

On Windows with vcpkg, provide an Eigen3/OpenCV toolchain in the normal CMake
way and build the same target.

## Coordinate convention

The synthetic pinhole camera uses the conventional computer-vision frame:

- +X: image right
- +Y: image down
- +Z: optical axis / forward

`R_wc` rotates vectors from camera coordinates into world coordinates.
