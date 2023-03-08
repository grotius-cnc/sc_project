
# sc_project
![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/screen.jpg?raw=true)

Motion control using a scurvature velocity profile.

The sc_engine c++ library.
 




## License

[![MIT License](https://img.shields.io/badge/License-MIT-green.svg)](https://choosealicense.com/licenses/mit/)


## Authors

- [@skynet](https://www.github.com/grotius-cnc)


## Deployment

To deploy this project run

```bash
  mkdir build
  cd build
  cmake ..
  make
```


## Features



## Examples

sc_primitives_gui

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_primitives_gui/screen.jpg?raw=true)

sc_curves_gui

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_curves_gui/screen.jpg?raw=true)

sc_planner

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_planner/screen.jpg?raw=true)



## Environment Variables

To run this project, you will need to add the following environment variables to your .env file

`API_KEY`

`ANOTHER_API_KEY`


## Optimizations

- Using inline.
- No std::cout in time critical area's


## Tech Stack

**Client:** React, Redux, TailwindCSS

**Server:** Node, Express


## Running Tests

To run tests, run the following command

```bash
  npm run test
```


## Documentation

- "jm" jerk max
- "a" acceleration
- "acc" acceleration stage
- "dcc" deceleration stage
- "as" acceleration at inflection point 

    `as=2*a`

- "acs" acceleration start
- "ace" acceleration end
- "vo" velocity start
- "ve" velocity end
- "vm" velocity max
- "s" displacement
- "dv" delta velocity

    `time from 0 acc to as, back to 0 dcc`

- "t1" concave acc period
- "t2" linear acc period
- "t3" convex acc period
- "t4" steady period
- "t5" convex dcc period
- "t6" linear dcc period
- "t7" concave dcc period






