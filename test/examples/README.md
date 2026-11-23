# BotOp C++ examples

Each folder is one small program (`main.cpp`) with its own `rai.cfg`.
Most use `BotOp`, so the same code runs in simulation or on the real robot
(`-real false` / `-real true`).

## Build and run

```bash
# from the repo root, once
pixi run cmake -S . -B build/examples -G Ninja -DBUILD_TESTS=ON -DUSE_LIBFRANKA=OFF -DUSE_REALSENSE=OFF
pixi run cmake --build build/examples -- -k 0          # all, or: --target test_07-leapCtrl

# run from the example folder (it reads the local rai.cfg and .g files)
cd test/examples/07-leapCtrl
pixi run ./test_07-leapCtrl -real false
```

* `-<key> <value>` overrides a `rai.cfg` parameter, e.g. `-real false`,
  `-bot/useOptitrack false`, `-botsim/engine kinematic`.
* Some `rai.cfg` files set `real`, so pass `-real false` for simulation.
* Press `q` (or Esc) in the viewer to quit; some examples wait for a key
  press in the viewer window, not in the terminal.
* `-noInteractivity` skips most "press a key" pauses.
* For hardware, configure the `hw` environment (`-DUSE_LIBFRANKA=ON
  -DUSE_REALSENSE=ON`) and see the main README, "Run on hardware".
* Many `main()` functions contain several test functions; only one is
  called, the others are commented out. Change the call to run another.

## Overview

| Example | What it shows | Needs hardware |
|---|---|---|
| `01-openClose` | Open/close the gripper, via BotOp (`testBotop`) or directly through the Franka / Robotiq gripper drivers (`testDirect`) | sim ok |
| `02-camera` | Camera images, depth and point clouds through BotOp (`getImageAndDepth`, `getImageDepthPcl`). Default `testBotop` reads two RealSense cameras; `testBotopPcl` and `testMini` use the simulated `cameraWrist` (run with `-bot/forceRealCamera false`) | RealSense (default) |
| `02-multi-camera` | Several RealSense cameras at once (`MultiRealSenseThread`), each shown in a window | RealSense |
| `02-optitrack` | Low-level OptiTrack client: stream rigid-body poses into a configuration | OptiTrack |
| `03-blobTracking` | Webcam/ZED image processing: ArUco marker detection and HSV color-blob tracking (OpenCV) | camera |
| `04-trivialCtrl` | Low-level `FrankaThread` without BotOp, with a minimal custom reference feed (gravity compensation / damping) | Franka |
| `05-moveTo` | Basic motion commands: `move` with timed waypoints, `moveTo`, spline reference without BotOp, and reactive control with `stepObservation`/`stepAction` + small IK | sim ok |
| `06-fastPath` | Execute the same path at increasing speeds and log joint data (for tracking-error analysis, `plt/`) | sim ok |
| `07-leapCtrl` | Reactive control: a receding-horizon KOMO (`ShortPathMPC`) plus timing optimization follows a jumping target | sim ok |
| `08-pnp` | Pick-and-place loop: plan grasp/place with KOMO, path planning, gripper, repeated trials | sim ok |
| `09-rndPoses` | Random collision-free poses, planned with `getStartGoalPath` and executed with `moveAutoTimed`; optional data logging | sim ok |
| `10-compliant` | Compliance control: hold the arm and make it compliant in a task space (`setCompliance`) | sim ok |
| `11-calibStation` | Two-arm OptiTrack calibration station: `collectData` (drive poses, record OptiTrack), `computeCalibration` (fit base/marker/OptiTrack poses with KOMO), `demoCalibration` (default). Models: `calibStation.g`, `calibStation-calibrated.g` | OptiTrack for data |
| `12-controlCalib` | Excite a single joint (`joint` parameter) and record controller data for tuning the Franka controller gains | Franka |
| `13-reactiveGrasp` | Sequence-of-constraints MPC (`SecMPC` via `skills/control/SecMPC_Stepper`): ball following/reaching, pick-and-place (default), pushing, drone race | sim ok |
| `14-calibPoints` | Wrist-camera calibration from colored dots: `collectData` (images from several views), `selectHSV`, `computeCalibration` (default, needs recorded `dat.g`) | RealSense |
| `15-arucoSystem` | Multi-camera ArUco tracking (`ArucoSystemThread`) fusing 4 cameras into an object pose; works with simulated cameras (`scene.yml`) | sim ok |
| `20-simulation` | Plain `rai::Simulation` (PhysX) without BotOp: pushing, grasping, gripper open/close (default), random scenes, friction, stacking | sim ok |
| `21-grasp` | Minimal grasp: KOMO two-waypoint approach, open gripper, move, close gripper | sim ok |

"sim ok" means the default test function runs in simulation.
