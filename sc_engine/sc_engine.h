#ifndef SC_ENGINE_H
#define SC_ENGINE_H

//! Author  : SKynet Cyberdyne
//! Licence : GPL2
//! Date    : 2023

#include <iostream>
#include <cmath>
#include <vector>

//! Make conversion's easy:
#define to_radians  M_PI/180.0
#define to_degrees  (180.0/M_PI)
#define ns_to_ms    0.000001

typedef double T;
typedef bool B;
typedef int I;
typedef uint UI;

//! Considerations :
//!
//! 1.  When pause request is near the end of the path
//!     and the pause can not be performed inside the s.
//!     The process_curve will refuse to pause and will
//!     pause at end of curve.
//!
//! 2.  If a motion is out of scope, the engine
//!     will output a minimal curve based on the input : ve, ace.
//!     The output s will overshoot the input s.
//!

//! Scurve back end.
class sc_engine {
public:
    sc_engine(){};

    //! Enum to define curve periods.
    enum sc_period_id {
        id_t1,
        id_t2,
        id_t3,
        id_t4,
        id_t5,
        id_t6,
        id_t7,
        id_pvec,
        id_none
    };

    //! A period with it's values.
    struct sc_period {
        //! Period type, t1,t2,t3, etc,
        sc_period_id id=sc_period_id::id_none;
        //! Velocity start.
        T vo=0;
        //! Velocity end.
        T ve=0;
        //! Acceleration start.
        T acs=0;
        //! Acceleration end.
        T ace=0;
        //! Netto curve displacement.
        T ncs=0;
        //! Netto curve time.
        T nct=0;
        //! Curve absolute start position.
        T startpos=0;
        //! Curve absolute end position.
        T endpos=0;
    };

    struct sc_motion {
        std::vector<sc_period> pvec;
    };

    enum sc_status {
        Error=0,
        Ok=1,
        Busy=2,
        Curve_finished=3,
        Traject_Finished=4
    };

    void sc_set_a_dv(T theA, T theDv);

    void interpolate_period(T at_time,
                            sc_period p,
                            T &pos,
                            T &vel,
                            T &acc);

    void interpolate_periods(T at_time,
                            std::vector<sc_period> pvec,
                            T &pos,
                            T &vel,
                            T &acc, bool &finished);

    T as=0;
    T a=0;
    T jm=0;
    T ct=0;

    inline int t1_t2_t3(sc_period p, std::vector<sc_period> &pvec);

    inline int t7_t1_t2_t3_t5(sc_period p, std::vector<sc_period> &pvec);

    inline int t5_t6_t7(sc_period p, std::vector<sc_period> &pvec);

    inline int t3_t5_t6_t7_t1(sc_period p, std::vector<sc_period> &pvec);

    int process_curve(sc_period p, T vm, std::vector<sc_period> &pvec);

    //! Inline is used for better time performance.

    inline int t1(T vo, T acs, T ace, sc_period &p);

    inline int t1_ve(T vo, T ve, T acs, sc_period &p);

    inline int t1_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t2(T vo, T ve, T a, sc_period &p);

    inline int t2_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t3(T vo, T acs, T ace, sc_period &p);

    inline int t3_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t4(T vo, T s, sc_period &p);

    inline int t4_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t5(T vo, T acs, T ace, sc_period &p);

    inline int t5_ve(T vo, T ve, T acs, sc_period &p);

    inline int t5_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t6(T vo, T ve, T a, sc_period &p);

    inline int t6_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline int t7(T vo, T acs, T ace, sc_period &p);

    inline int t7_i(sc_period p, T ti, T &vi, T &si, T &ai);

    inline T to_vh_acc(T vo, T ve);

    inline T to_vh_dcc(T vo, T ve);

    T to_stot_pvec(std::vector<sc_period> pvec);

    T to_ttot_pvec(std::vector<sc_period> pvec);
};

#endif





























