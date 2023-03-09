
# sc_project

a c++ sc_engine library

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_documents/profile.jpg?raw=true)

## Features

- scurvature motion profiles.
- constant jerk.
- using periods t1,t2,t3,t4,t5,t6,t7.
- realtime performance ~0.02, ~0.06 ms cycle.
- waypoints.
- acceleration start, end values.
- "vm" velocity max interupts.
- motion planner, sc_planner.
- 9 axis 3d interpolation for line and arc.

## Optimizations

- Using inline
- No std::cout in time critical area's

## Todo

- Look ahead.
- Get the desired corner speeds when ve>0.

## Documentation

[Formula's and defenitions](https://github.com/grotius-cnc/sc_project/blob/main/sc_engine/sc_formula.h)
[Science papers](https://github.com/grotius-cnc/sc_project/tree/main/sc_documents)

## Dependencies

The sc_engine has no dependencies and is written in CXX20.
The sc_planner & sc_interpolation uses Eigen3 and Qt 

Read the comments in the toplevel CMakeLists.txt file how
to install Eigen3.

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

