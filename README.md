# LIS robot operation repo

This repo is how we, in the
[Learning & Intelligent Systems Lab](https://argmin.lis.tu-berlin.de/),
operate our robots. We decided for a spline-based convention to
command motion (in class BotOp), which turned out the cleanest
interface to our motion optimization methods, and now also MPC
methods.

This repo is really our working environment, is not clean, contains
much outdated code, and might be too specific for others to use, but
we're happy to share. When run without hardware, the BotOp interface
can emulate/simulate any robot (e.g., found in rai-robotModels).

See the [robotic python package](https://github.com/MarcToussaint/rai)
for a python wrapper.

The [rai](https://github.com/MarcToussaint/rai) module contains our
actual algorithmic code base (data structures, optimizers, kinematics,
KOMO, LGP, etc).

We don't use ROS at all anymore.

## Installation

All dependencies (compilers, C++ libraries, Python) are managed by
[pixi](https://pixi.sh) from conda-forge, pinned in `pixi.lock`, and live
only in `.pixi/` inside this repo. Nothing is installed into the system,
`~/.local`, or other Python/conda environments. The only system requirement
is a graphics driver (and an X11/XWayland display for the viewer).

* Install pixi once (into `~/.pixi`; add `~/.pixi/bin` to your PATH):

      curl -fsSL https://pixi.sh/install.sh | bash

* Clone and build:

      git clone --recurse-submodules git@github.com:MarcToussaint/botop.git
      cd botop
      pixi run install      # builds fcl/qhull/PhysX once, then botop; installs into .pixi/envs/default

* Test in simulation:

      pixi run test-py
      pixi run bot -sim -loop

* For the real robot (libfranka + librealsense) use the `hw` environment.
  libfranka must match your robot's system version; pin it in `pixi.toml` if needed:

      pixi run -e hw install
      pixi run -e hw bot -home

* Work interactively with `pixi shell` (or `pixi shell -e hw`): `python`, `bot`,
  `kinEdit`, `meshTool` and `import robotic` then come from this environment.

Environments and tasks (see `pixi.toml`):

| | |
|---|---|
| `default` | simulation only (no libfranka/librealsense) + jupyter |
| `hw` | adds libfranka, librealsense, poco |
| `pixi run deps` | source-build fcl 0.5, qhull (non-reentrant), PhysX 104.1 into the env (`scripts/build_deps.sh`) |
| `pixi run build` / `install` | configure + build into `build/<env>` / install into the env |
| `pixi run stubs` | generate `_robotic.pyi` stubs |
| `pixi run clean` | remove `build/` |

C++ tests are not built by default; configure with `-DBUILD_TESTS=ON` to build `test/*/main.cpp`.
Python-only packages from PyPI can be added under `[pypi-dependencies]` in `pixi.toml` (installed via uv).


## Panda robot operation

* The user needs to be part of the `realtime` and `dialout` unix group:

      sudo usermod -a -G realtime <username>
      sudo usermod -a -G dialout <username>

You need to log out and back in (or even reboot) for this to take effect. Check with `groups` in a terminal.
* Turn on the power switch at the control box
* Open the panda web interface at `https://172.16.0.2/desk/`. Accept the security risk. You'll need to log in with user `mti` and passwd `mti-engage`
* Unlock the joints
* **ALWAYS 2 PEOPLE ARE REQUIRED! ONE TO HOLD THE EMERGENCY STOP**
* Perform a series of tests
  * `bot -close`
  * `bot -open`
  * `bot -home`
  * `bot -loop`


## Setting up your own coding environment/directory

* Use a separate repository for your own code
* Place the repository in $HOME/git (parallel to ~/git/botop)
* In a working directory with your `main.cpp` place the following `Makefile` - and compile

      BASE = ../../botop/rai
      
      include $(BASE)/build/generic.mk

* If you have more cpp-files, you can add `OBJS = main.o code.o etc.o`
* Otherwise, use your own build system and just link with librai.so. After `pixi run install`,
  headers are in `.pixi/envs/default/include/rai` and the library in `.pixi/envs/default/lib`;
  build from within `pixi shell` so the same compiler and libraries are used.


## cmd line tool `bot`

* `bot -open -close`
* `bot -home`
* `bot -loop -speed 2`
* `bot -float`
* `bot -hold`
* `bot -damp`



