
# sc_planner

To add waypoints to the planner :

	//! Clear previous waypoints.
    planner->sc_clear();

	//! Add 3d line.
 	planner->sc_add_line_motion(vo,ve,acs,ace,{0,0,0},{100,0,0});
 	
	//! Add 3d arc.
 	planner->sc_add_arc_motion(vo,ve,acs,ace,{0,0,0},{50,50,0},{100,0,0});
	
	//! Add motion up to 9 axis.
	planner->sc_add_general_motion(	T vo,
                            		T ve,
                            		T acs,
                            		T ace,
                            		sc_primitive_id id,
                            		sc_pnt start,
                            		sc_pnt way,
                            		sc_pnt end,
                            		sc_dir dir_start,
                            		sc_dir dir_end,
                            		sc_ext ext_start,
                            		sc_ext ext_end);

	//! Run from first program line.
 	planner->sc_set_startline(0);

    //! Run program.
 	planner->sc_set_state(sc_planner::sc_enum_program_status::program_run);

Full example :

https://github.com/grotius-cnc/sc_project/blob/main/sc_planner/mainwindow.cpp

![App Screenshot](https://github.com/grotius-cnc/sc_project/blob/main/sc_planner/screen.jpg?raw=true)



