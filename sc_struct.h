#ifndef SC_STRUCT_H
#define SC_STRUCT_H

//! Author  : SKynet Cyberdyne
//! Licence : GPL2
//! Date    : 2023

//! Todo :
//!
//! 1. motion reverse, adaptive feed.
//! 2. scurve stepgen or pid.
//! 3. ijk gcode to arc waypoint.
//! 4. recode pause.
//! 5. mdi commands, like goto zero.
//! 6. read gcode file.
//! 7. display gcode path with opengl or opencascade.
//! 8. new science paper.

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//! Make conversion's easy:
#define to_radians  M_PI/180.0
#define to_degrees  (180.0/M_PI)
#define ns_to_ms    0.000001

typedef double T;

//! Enum to define curve periods.
enum sc_period_id {
    id_t1,
    id_t2,
    id_t3,
    id_t4,
    id_t5,
    id_t6,
    id_t7,
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
};

struct sc_motion_params {
    T maxacc=0;
    T maxdv=0;
};

struct sc_motion_in {
    T curvel=0;
    T maxvel=0;
    T endvel=0;
    T endpos=0;
    T curacc=0;
    T endacc=0;
    bool pause=0;
};

struct sc_motion_out {
    T newvel=0;
    T newpos=0;
    T newacc=0;
    bool negative=0;
};

enum sc_joint_id {
    id_J0=0,
    id_J1=1,
    id_J2=2,
    id_J3=3,
    id_J4=4,
    id_J5=5,
    id_J6=6,
    id_J7=7,
    id_J8=8,
    id_J9=9,
    id_J10=10,
    id_J11=11,
    id_J12=12,
    id_J13=13,
    id_J14=14,
    id_J15=15
};

enum sc_status {
    Error=0,
    Ok=1,
    Busy=2,
    Curve_finished=3,
    Traject_Finished=4
};

enum sc_point_id {
    LINE,
    ARC
};

struct sc_pnt {
    T x=0, y=0, z=0;

    T lenght_to_next_point(sc_pnt p){
        return sqrt(pow(p.x-x,2)+pow(p.y-y,2)+pow(p.z-z,2));
    }
};

struct sc_dir {
    T a=0, b=0, c=0;
};

struct sc_ext {
    T u=0, v=0, w=0;
};

struct sc_arc {
    std::vector<sc_pnt> pntVec;
    float radius=0;
    float diameter=0;
    float arcLenght=0;
    float arcAngleRad=0; //! Radians
    float arcCircumFence=0;
    bool arcAngleNegative=0; //! When sign of the angle < 0, set true.
    sc_pnt center;
    sc_pnt pointOnArcAxis;
};

#endif










