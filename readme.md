
# sc_project

a c++ sc_engine library

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/screen.jpg?raw=true)

## Features

- scurvature motion profiles
- constant jerk 
- using periods t1,t2,t3,t4,t5,t6,t7
- realtime performance ~0.02, ~0.06 ms cycle
- waypoints primitives line and arc.
- acceleration start, end values
- "vm" velocity max interupts
- motion planner, sc_planner
- 9 axis 3d interpolation for line and arc.

## Optimizations

- Using inline
- No std::cout in time critical area's

## Documentation

[Formula's and defenitions](https://github.com/grotius-cnc/sc_project/blob/main/sc_engine/sc_formula.h)

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

