# Installation

## Supported platform

Ubuntu 24.04, Python 3.12, ROS 2 Jazzy and Gazebo Harmonic. The numerical mode
needs only Python, NumPy, SciPy and Matplotlib. ROS/Gazebo is an additional
installation path. Neither mode requires Crazyradio, Motive or a real aircraft.

Clone the desired release using the command in the root README. Use an ordinary
terminal outside any CrazySim/Conda/ROS virtual environment.

## Numerical dependencies

```bash
bash scripts/setup.sh numerical
bash scripts/verify.sh
```

`setup.sh` uses Ubuntu apt packages and requests sudo only for installation.
The tested versions are NumPy 1.26.4, SciPy 1.11.4 and Matplotlib 3.6.3.
`requirements-numerical.txt` describes compatible Python ranges; the tested
path is apt with `/usr/bin/python3`. Scripts disable user-site packages to avoid
mixing user-installed SciPy with system NumPy. They do not modify shell startup
files or the original simulation/hardware workspaces.

## ROS/Gazebo dependencies and build

First install ROS Jazzy from the official guide linked in the README. Then:

```bash
bash scripts/setup.sh ros
bash scripts/build.sh --executor sequential
bash scripts/verify.sh --ros
```

The setup installs colcon, rosdep, ros_gz, RViz, Gazebo vendor dependencies,
Qt5 development dependencies and numerical packages. It initializes rosdep only
when needed and resolves dependencies from the included package manifests.

All eight project packages and the required meshes are included. The empty
`dependencies.repos` means no additional Git repositories need importing.
Build scripts start from the base Jazzy environment; build products stay inside
this checkout. A successful verification must find generated interfaces and
native libraries under this checkout's `install/` directory.

Do not copy `build/` or `install/` from another folder or edition. Colcon build
products contain paths tied to their checkout.
