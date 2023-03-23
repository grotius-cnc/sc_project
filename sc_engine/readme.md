# sc_engine

sc_engine library

## Discription

A core module to create scurved motion profiles.

This module primairely contains formula functions 

related to the science papers. 

The formula's are expanded for dcc periods t5,t7 to

avoid scurve mirroring. This formula expanding 

enables forward calculations.

## Optimizations

Using inline.

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

