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
pixi run install                                  # builds deps once, then botop
pixi run test-py                                  # python smoke test (simulation)
pixi run bot -sim -loop
```

With real hardware (libfranka, librealsense) use the `hw` environment, e.g.
`pixi run -e hw install`. libfranka must match the robot's system version;
pin it in `pixi.toml` if needed. Use `pixi shell` (or `pixi shell -e hw`) to
work interactively with `python`, `import robotic`, `bot`, `kinEdit` and
`meshTool`.

| pixi task | |
|---|---|
| `deps` | source-build fcl 0.5, non-reentrant qhull, PhysX 104.1 into the env (`scripts/build_deps.sh`) |
| `build` / `install` | build into `build/<env>` / install into the env |
| `test-py` | run `test/py/test.py` |
| `stubs` | generate `_robotic.pyi` |
| `clean` | remove `build/` |

PyPI-only Python packages can be added under `[pypi-dependencies]` in
`pixi.toml` (installed via uv).

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
