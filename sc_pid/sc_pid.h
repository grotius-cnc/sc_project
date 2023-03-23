#ifndef SC_PID_H
#define SC_PID_H

//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023

#include <../sc_engine/sc_engine.h>
#include <../sc_interpolate/sc_interpolate.h>

//! A motion follower.
//! Works nice for stand alone motion app's using
//! a scurved motion profile.
class sc_pid
{
public:
    sc_pid();

    V sc_set_a_dv_interval(T a,
                           T dv,
                           T interval);

    V sc_set_maxvel(T value);
    V sc_set_tarpos(T value);
    V sc_set_curpos(T value);
    V sc_set_curacc(T value);
    V sc_set_curvel(T value);

    //! Connect to thread.
    B sc_update();

    V sc_result(T &pos,
                T &vel,
                T &acc);

private:
    sc_engine *engine = new sc_engine();
    T interval=0;
    T maxvel=0;
    T curvel=0;
    T curacc=0;
    T curpos=0;
    T tarpos=0;

    inline V sc_update_value(T pos_add,
                      T vel,
                      T acc,
                      B reverse);
};

#endif

























