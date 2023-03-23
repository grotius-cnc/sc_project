
# sc_project

a c++ sc_engine library

## Features

sc_engine:
- scurvature motion profiles.
- constant jerk.
- using periods t1,t2,t3,t4,t5,t6,t7.
- realtime performance ~0.02, ~0.06 ms cycle.

sc_planner, sc_planner_gui:
- start, stop, pause, pause-resume.
- adaptive-feed.
- waypoints.
- acceleration start, end values.
- "vm" velocity max interupts.
- interpolation of waypoints.

sc_optimizer, sc_optimizer_gui:
- reduces corner speeds.
- reduces gforce for arc's.
- set's velocity transitions.
- calculate gforce.

sc_pid, sc_pid_gui:
- stand alone motion follower.

sc_interpolate:
- 9 axis interpolation module. 

sc_primitives, sc_primitives_gui:
- Example how curvatures work in this library.

sc_curves_gui:
- Example to construct one motion. vo->vm->ve.

sc_common:
- Basic's to use in the sc_project.

## Optimizations

sc_engine:
- Using inline.
- No std::cout in time critical area's.
sc_planner:
- state machine.

## Todo

[Todo file](https://github.com/grotius-cnc/sc_project/tree/main/todo.txt)

## Documentation

[Formula's and defenitions](https://github.com/grotius-cnc/sc_project/blob/main/sc_engine/sc_formula.h)

[Science papers](https://github.com/grotius-cnc/sc_project/tree/main/sc_documents)

## Dependencies

The sc_engine has no dependencies and is written in c++20.

Gui apps depends on Qt.

Some classes depends on Eigen3. 

## Deployment

Read the comments in the toplevel CMakeLists.txt file.

To deploy this project run

```bash
  mkdir build
  cd build
  cmake ..
  make
```

## License

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)

## Authors

[Skynet-Cyberdyne](https://www.github.com/grotius-cnc)

