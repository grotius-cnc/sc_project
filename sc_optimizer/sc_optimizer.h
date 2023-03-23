#ifndef SC_OPTIMIZER_H
#define SC_OPTIMIZER_H

//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023

#include <vector>
#include <iostream>
#include <cmath>
#include <Dense>
#include <Geometry>

#include <../sc_common/sc_struct.h>
#include <../sc_common/sc_arcs.h>
#include <../sc_interpolate/sc_interpolate.h>
#include <../sc_engine/sc_engine.h>

//! Make conversion's easy:
#define to_radians  M_PI/180.0
#define to_degrees  (180.0/M_PI)

typedef void V ;
typedef bool B ;
typedef double T ;
typedef uint UI;

class sc_optimizer
{
public:

    //! Constructor.
    sc_optimizer();

    //! Single function to optimize path planning.
    std::vector<sc_block> sc_optimize_path(
            std::vector<sc_block> blockvec,
            T velmax,
            T gforcemax, T a, T dv);

    //! Calculates rotational gforce impact in [g].
    //! We use this to set maxvel for arc's.
    V sc_get_gforce(T vel_mm_sec, T radius, T &gforce);
    V sc_set_gforce(T radius, T gforce, T &vel_mm_sec);

    //! Iterate over the blockvec to get it's follow up angles.
    //! This results in acceptable corner ve's.
    std::vector<sc_block> sc_get_blockangles(
            std::vector<sc_block> blockvec);

    //! Calculate end velocity's based on the given angles to next primitive.
    std::vector<sc_block> sc_get_corner_ve_blockangles(
            std::vector<sc_block> blockvec,
            T velmax);

    //! Set maxvel for arcs, depending on gforce.
    std::vector<sc_block> sc_get_velmax_gforce(std::vector<sc_block> blockvec,
            T velmax,
            T gforcemax);

    //! Calculate ve's iterating over gcode.
    //! Velmax is already set by the previous gforce arc function.
    std::vector<sc_block> sc_process_forward_ve(std::vector<sc_block> blockvec);

    //! p1 = common point.
    V line_line_angle(sc_pnt p0,
                      sc_pnt p1,
                      sc_pnt p2,
                      T &angle_deg);

    //! p0 = line start.
    //! p1 = arc startpoint, common point.
    //! p2 = arc waypoint.
    //! p3 = arc endpoint.
    V line_arc_angle(sc_pnt p0,
                     sc_pnt p1,
                     sc_pnt p2,
                     sc_pnt p3,
                     T &angle_deg);

    //! p0 = arc start.
    //! p1 = arc waypoint.
    //! p2 = arc endpoint, common point.
    //! p3 = line endpoint.
    V arc_line_angle(sc_pnt p0,
                     sc_pnt p1,
                     sc_pnt p2,
                     sc_pnt p3,
                     T &angle_deg);

    //! p0 = arc start.
    //! p1 = arc waypoint.
    //! p2 = arc endpoint, common point.
    //! p3 = arc waypoint.
    //! p4 = arc endpoint.
    V arc_arc_angle(sc_pnt p0,
                    sc_pnt p1,
                    sc_pnt p2,
                    sc_pnt p3,
                    sc_pnt p4,
                    T &angle_deg);

private:
    sc_engine *engine=new sc_engine();
};

#endif




















