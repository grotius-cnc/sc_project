#ifndef SC_PLANNER_H
#define SC_PLANNER_H

//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023

#include <chrono>
#include <../sc_engine/sc_engine.h>
#include <../sc_interpolate/sc_interpolate.h>

//! return 0 = error
//! return 1 = ok
class sc_planner
{
public:
    sc_planner();

    enum sc_enum_program_status {
        program_run=0,
        program_pause=1,
        program_pause_resume=2,
        program_stop=3,
        program_vm_interupt=4,
        program_wait=5,
        program_end=6,
        program_error=7
    };
    sc_enum_program_status sc_enum_program_state=
            sc_enum_program_status::program_end;

    enum sc_enum_run_status {
        run_check,
        run_init,
        run_cycle
    };
    sc_enum_run_status sc_enum_run_state=
            sc_enum_run_status::run_check;

    enum sc_enum_pause_status {
        pause_init,
        pause_cycle
    };
    sc_enum_pause_status sc_enum_pause_state=
            sc_enum_pause_status::pause_init;

    enum sc_enum_pause_resume_status {
        pause_resume_init,
        pause_resume_cycle
    };
    sc_enum_pause_resume_status sc_enum_pause_resume_state=
            sc_enum_pause_resume_status::pause_resume_init;

    enum sc_enum_stop_status {
        stop_init,
        stop_cycle
    };
    sc_enum_stop_status sc_enum_stop_state=
            sc_enum_stop_status::stop_init;

    enum sc_enum_vm_interupt_status {
        vm_interupt_init,
        vm_interupt_cycle
    };
    sc_enum_vm_interupt_status sc_enum_vm_interupt_state=
            sc_enum_vm_interupt_status::vm_interupt_init;


    V sc_set_a_dv(T acceleration, T delta_velocity);
    V sc_set_interval(T time);
    V sc_set_maxvel(T velocity_max);
    V sc_set_adaptive_feed(T adaptive_feed);
    V sc_set_startline(UI startline);
    V sc_set_position(T position);
    V sc_set_state(sc_enum_program_status command);
    B sc_set_traject_stot();
    V sc_get_motionvec_nr_from_position(
            T &motionvec_nr,
            T &motionvec_nr_progress,
            T &motionvec_nr_dtg);

    B sc_check_motionvec_loaded();
    B sc_check_motionvec_startline();
    B sc_check_servo_cycletime();
    B sc_check_vm();

    V sc_get_program_state(sc_enum_program_status &state);
    std::string sc_get_program_state();
    B sc_get_run_cycle_state();
    B sc_get_pause_cycle_state();
    B sc_get_stop_cycle_state();

    V sc_reset();
    V sc_clear();
    V sc_add_line_motion(T vo,
                         T ve,
                         T acs,
                         T ace,
                         sc_pnt start,
                         sc_pnt end);
    V sc_add_arc_motion(T vo,
                        T ve,
                        T acs,
                        T ace,
                        sc_pnt start,
                        sc_pnt way,
                        sc_pnt end);

    //! Add motion up to 9 axis.
    V sc_add_general_motion(T vo,
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

    //! Attached to thread.
    V sc_update();

    V sc_get_planner_results(T &position,
                             T &velocity,
                             T &acceleration,
                             UI &line_nr,
                             T &line_progress,
                             T &traject_progress,
                             B &finished);

    V sc_get_interpolation_results(
            sc_pnt &xyz,
            sc_dir &abc,
            sc_ext &uvw,
            T &curve_progress);

    //! State machine functions:
    V sc_run_state();
    //! Subs:
    B sc_run_flag=0; //! Check preconditions only once.
    V sc_run_check();
    V sc_run_init();
    V sc_run_cycle();
    V sc_run_finished();

    V sc_pause_state();
    //! Subs:
    V sc_pause_init();
    V sc_pause_cycle();

    V sc_pause_resume_state();
    //! Subs:
    V sc_pause_resume_init();
    V sc_pause_resume_cycle();

    V sc_stop_state();
    //! Subs:
    V sc_stop_init();
    V sc_stop_cycle();


    V sc_vm_interupt_state();
    //! Subs:
    V sc_vm_interupt_init();
    V sc_vm_interupt_cycle();

    V sc_program_end_state();
    V sc_error_state();

    T sc_performance();

private:

    sc_engine *engines = new sc_engine();
    sc_interpolate *interpolates= new sc_interpolate();
    std::vector<sc_block> blockvec; //! The gcode coordinates.
    std::vector<sc_engine::sc_period> motionvec; //! Derived lenghts from blockvec.
    std::vector<sc_engine::sc_period> pvec;

    sc_enum_program_status sc_last_program_state;

    UI sc_motionvec_nr=0;
    T sc_motionvec_nr_dtg=0;
    T sc_motionvec_nr_progress=0;

    T sc_ms=0;
    T sc_vm=0;
    T sc_adaptive_feed=1;

    T sc_stot=0;
    T sc_timer=0;
    T sc_interval=0;
    T sc_newpos=0;
    T sc_oldpos=0;
    T sc_pos=0;
    T sc_vel=0;
    T sc_acc=0;



};

#endif
