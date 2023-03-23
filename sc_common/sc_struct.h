#ifndef SC_STRUCT_H
#define SC_STRUCT_H

//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023

#include <vector>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

//! Make conversion's easy:
#define to_radians  M_PI/180.0
#define to_degrees  (180.0/M_PI)
#define ns_to_ms    0.000001

typedef double T ;
typedef void V;
typedef uint UI;
typedef int I;

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

enum sc_primitive_id {
    sc_line,
    sc_arc,
};

enum sc_type {
    sc_G0,
    sc_G1,
    sc_G2,
    sc_G3
};

#endif










