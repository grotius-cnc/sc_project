# sc_pid

sc_pid library

## Discription

A module to simulate a live scurve.

This is a algoritme that updates every cycle.

It's looking every cycle step in what period the curve is,

and then takes a next step.

Currently works best without updating

the tarpos every 1 ms.

## Optimizations

## Todo

## Documentation

[Formula's and defenitions](https://github.com/grotius-cnc/sc_project/blob/main/sc_engine/sc_formula.h)

[Science papers](https://github.com/grotius-cnc/sc_project/tree/main/sc_documents)

## Dependencies

c++20
c++ std lib is used.

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

