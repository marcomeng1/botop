# BotOp

Robot operation interface of the
[LIS lab](https://argmin.lis.tu-berlin.de/): motion is commanded as splines
via the `BotOp` class, which works with real hardware (Franka Panda, Robotiq,
RealSense, OptiTrack) or fully in simulation (PhysX) for any model in
`rai-robotModels`. Comes with C++ and Python (`robotic`) interfaces.

## Installation

All dependencies (compilers, C++ libraries, Python) come from conda-forge via
[pixi](https://pixi.sh), are pinned in `pixi.lock`, and live only in `.pixi/`
inside this repo. Nothing is installed into the system, `~/.local`, or other
Python/conda environments.

```bash
curl -fsSL https://pixi.sh/install.sh | bash      # once; installs into ~/.pixi
git clone --recurse-submodules git@github.com:marcomeng1/botop.git
cd botop
pixi run install-all                              # deps once, then botop for simulation + hardware
pixi run test-py                                  # python smoke test (simulation)
pixi run bot -sim -loop
```

If `pixi` is not found, add it to the path: `export PATH="$HOME/.pixi/bin:$PATH"`.
There are two environments: `default` (simulation only) and `hw` (adds
libfranka, librealsense). Prefix commands with `pixi run` (or `pixi run -e hw`),
or activate an environment with `pixi shell` (or `pixi shell -e hw`) and call
`python`, `bot`, `kinEdit`, `meshTool` directly.

| pixi task | |
|---|---|
| `deps` | source-build fcl 0.5, non-reentrant qhull, PhysX 104.1 into the env (`scripts/build_deps.sh`) |
| `build` | compile the current environment into `build/<env>`, no install (`-e hw` for hardware) |
| `install` | `build` + install into the current environment |
| `install-all` | `install` for both `default` and `hw` |
| `test-py` | run `test/py/test.py` |
| `stubs` | generate `_robotic.pyi` |
| `clean` | remove `build/` |

PyPI-only Python packages can be added under `[pypi-dependencies]` in
`pixi.toml` (installed via uv).

## Run in simulation

```bash
pixi run install
pixi run bot -sim -home                    # move home
pixi run bot -sim -loop                    # move back and forth
pixi run bot -sim -open                    # also -close, -float, -damp, -hold
pixi run bot -sim -confFile rai-robotModels/scenarios/pandasTable.g -home   # other model
pixi run kinEdit rai-robotModels/scenarios/pandaSingle.g                    # view a model
```

In Python: `bot = ry.BotOp(C, useRealRobot=False)`. The simulation (PhysX)
drives all joints of the loaded model, so any model works; the simulated
gripper is the frame `l_gripper`.

## Run on hardware

```bash
pixi run -e hw install                     # once; builds with libfranka + librealsense
pixi run -e hw bot -home                   # same flags as above, without -sim
pixi run -e hw bot -home -bot/useArm both  # two arms (pandasTable-calibrated.g)
pixi run -e hw python my_script.py         # with ry.BotOp(C, useRealRobot=True)
```

All libraries (libfranka, librealsense, Poco, libusb, ...) come from pixi;
binaries find them via RPATH. Pixi cannot set up the OS, so once per PC (sudo):

* Real-time kernel (`uname -r` shows `-rt`/`PREEMPT_RT`) and real-time
  priority for your user (`ulimit -r` must not be 0, e.g. a `realtime` group
  with `rtprio 99` and `memlock unlimited` in `/etc/security/limits.conf`),
  see the [libfranka docs](https://frankaemika.github.io/docs/installation_linux.html).
* RealSense udev rules (`99-realsense-libusb.rules` from
  [librealsense](https://github.com/IntelRealSense/librealsense/tree/master/config))
  in `/etc/udev/rules.d/`.
* Robotiq on USB serial: your user in the `dialout` group.

Before the first run:

* Network connection to the robot:
  left Franka `172.16.0.2`, right Franka `172.17.0.2` (`src/Franka/`).
* libfranka must match the robot's system version; pin it under
  `[feature.hw.dependencies]` in `pixi.toml` and re-run `pixi run -e hw install`.
* Unlock the brakes and activate FCI in the Franka Desk interface.
* `-bot/blockRealRobot true` forces simulation even when real is requested.

`BotOp` decides which drivers to start from frame names in the model
(`src/BotOp/bot.cpp`):

| frame in the model | driver |
|---|---|
| `l_panda_base`, `r_panda_base` | `FrankaThread` (`src/Franka`) |
| `l_panda_finger_joint1`, `r_panda_finger_joint1` | `FrankaGripper` |
| `l_robotiq_base`, `r_robotiq_base` | `RobotiqGripper` (`src/Robotiq`) |
| `omnibase_world` | `OmnibaseThread` (`src/Omnibase`) |

## Build simulation and hardware together

```bash
pixi run install-all                       # = pixi run -e default install && pixi run -e hw install
pixi run bot -sim -home                    # simulation build (.pixi/envs/default)
pixi run -e hw bot -home                   # hardware build (.pixi/envs/hw)
```

Each environment has its own build folder (`build/default`, `build/hw`) and
install location, so both stay incremental and do not overwrite each other.
`pixi run clean` removes both build folders.

## Adding a new robot arm or hand

1. **Model.** Convert the URDF and meshes into `rai-robotModels/<robot>/`
   (see `rai-robotModels/robotiq/HOWTO.sh`): `python rai/bin/urdf2rai.py
   robot.urdf > robot.g` (needs `lxml`: `pixi add --feature dev lxml`), then
   `kinEdit -file robot.g -cleanOnly -prune`. Write a scenario file like
   `scenarios/pandaSingle.g` with `Prefix: "l_"`, a unique base frame
   (e.g. `l_myarm_base`) and a `gripper` marker frame on the hand.
2. **Test in simulation.** `pixi run kinEdit <scenario>.g`, then
   `pixi run bot -sim -confFile <scenario>.g -home`. Simulation needs no driver.
3. **Driver.** Add `src/<MyArm>/`:
   * an arm is a `rai::RobotAbstraction` + `Thread` that reads the reference
     from `cmd` and writes `q`, `qDot` to `state` in `step()` (template:
     `src/Omnibase`, or `src/Franka` for torque control);
   * a hand is a `rai::GripperAbstraction` implementing `open`, `close`,
     `pos`, `isDone` (template: `src/Robotiq`).

   Wrap the vendor SDK code in `#ifdef RAI_<MYARM>` with a stub in `#else`,
   as `src/Franka` does, so the `default` environment still builds.
4. **Build.** In `CMakeLists.txt` add `file(GLOB SRC_<MyArm> src/<MyArm>/*.cpp)`
   to the source list and a `USE_<MYARM>` option that defines `RAI_<MYARM>`
   and links the SDK. Add the SDK to `[feature.hw.dependencies]` in
   `pixi.toml` if it is on conda-forge, else as a build function in
   `scripts/build_deps.sh`.
5. **Register.** In `BotOp::BotOp` (`src/BotOp/bot.cpp`) start the driver when
   its base frame exists, next to the Franka/Robotiq branches. Python
   (`ry.BotOp`) and `bot` then work without further changes.

## Build and run the tests

Python (`test/py`, needs `pixi run install` first so that `import robotic` works):

```bash
pixi run test-py                           # = python test/py/test.py
pixi run python my_script.py               # any own script using `import robotic as ry`
```

C++ examples (`test/examples`): build them in a separate build dir, so the
normal `pixi run build` is unaffected. Executables are written into the
example's folder (`test_<name>`) and must be run from there (they read the
local `rai.cfg`):

```bash
pixi run cmake -S . -B build/examples -G Ninja -DBUILD_TESTS=ON -DUSE_LIBFRANKA=OFF -DUSE_REALSENSE=OFF
pixi run cmake --build build/examples --target test_10-compliant
cd test/examples/10-compliant
pixi run ./test_10-compliant -real false   # simulation; press 'q' in the viewer to quit
```

Command line arguments `-<key> <value>` override parameters in `rai.cfg`
(e.g. `-real false`, `-bot/useGripper false`). Examples that currently build:
01-openClose, 02-camera, 02-optitrack, 03-blobTracking, 10-compliant,
12-controlCalib, 15-arucoSystem; the others need updating to the current rai API.

## Layout

| path | |
|---|---|
| `src/` | BotOp and hardware drivers (Franka, Robotiq, RealSense, OptiTrack, Omnibase, Audio, MarkerVision) |
| `bin/bot/` | `bot` command line tool, minimal BotOp example |
| `test/examples/` | C++ examples; build with `-DBUILD_TESTS=ON` (several need updating to the current rai API) |
| `test/py/` | Python smoke test |
| `rai/` | submodule: core algorithms (kinematics, KOMO, optimization, ...) |
| `rai-robotModels/` | submodule: robot and scene models |

## Own code

Link against `librai.so`: after `pixi run install`, headers are in
`.pixi/envs/default/include/rai` and the library in `.pixi/envs/default/lib`.
Build from within `pixi shell` so the same compiler and libraries are used.

## Documentation and related repos

* [docs/howto.md](docs/howto.md): how to command motion with BotOp, examples
* [rai](https://github.com/marcomeng1/rai/tree/pixi-cleanup): core code base
  (upstream: [MarcToussaint/rai](https://github.com/MarcToussaint/rai))
* [rai-robotModels](https://github.com/MarcToussaint/rai-robotModels): models
* [robotic](https://github.com/MarcToussaint/robotic): Python package
  ([documentation](https://marctoussaint.github.io/robotic/))
* upstream BotOp: [MarcToussaint/botop](https://github.com/MarcToussaint/botop)
