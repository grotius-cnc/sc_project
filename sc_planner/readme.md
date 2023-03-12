
# sc_planner

To add waypoints to the planner :

	//! Clear previous waypoints.
    planner->sc_clear();

	//! Add 3d line.
 	planner->sc_add_line_motion(vo,ve,acs,ace,{0,0,0},{100,0,0});
 	
	//! Add 3d arc.
 	planner->sc_add_arc_motion(vo,ve,acs,ace,{0,0,0},{50,50,0},{100,0,0});

	//! Run from first program line.
 	planner->sc_set_startline(0);

    //! Run program.
 	planner->sc_set_state(sc_planner::sc_enum_program_status::program_run);

Full example :

	https://github.com/grotius-cnc/sc_project/blob/main/sc_planner/mainwindow.cpp

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_planner/screen.jpg?raw=true)



