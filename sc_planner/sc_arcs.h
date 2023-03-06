#ifndef SC_ARCS_H
#define SC_ARCS_H

//! Author  : SKynet Cyberdyne
//! Licence : GPL2
//! Date    : 2023

#include <vector>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <../sc_struct.h>

using namespace Eigen;

class sc_arcs
{
public:
    sc_arcs();

    void sc_interpolate_arc(sc_pnt p0,
                     sc_pnt p1,     //! Waypoint.
                     sc_pnt p2,
                     T progress,    //! 0-1.
                     sc_pnt &pi);   //! Interpolated point.

    T sc_arc_lenght(sc_pnt p0,
             sc_pnt p1,  //! Waypoint.
             sc_pnt p2);

private:
    sc_pnt sc_rotate_point_around_line(sc_pnt thePointToRotate,
                                    T theta,sc_pnt theLineP1,
                                    sc_pnt theLineP2);

    sc_arc sc_arc_points(Eigen::Vector3d p1,
                      Eigen::Vector3d p2,
                      Eigen::Vector3d p3,
                      T division);
};

#endif

