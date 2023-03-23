#include "sc_optimizer.h"

sc_optimizer::sc_optimizer( )
{

}

std::vector<sc_block> sc_optimizer::sc_optimize_path(
        std::vector<sc_block> blockvec,
        T velmax,
        T gforcemax, T a, T dv){

    engine->sc_set_a_dv(a,dv);

    //! 1.Calculate motion block corners.
    blockvec=sc_get_blockangles(blockvec);

    //! 2.Set end velocity, based on block corners, if no angle, ve is set to vm at this stage.
    blockvec=sc_get_corner_ve_blockangles(blockvec,velmax);

    //! 3.Set the velmax for arc's using gforce value. Set the velmax for lines to program velmax.
    blockvec=sc_get_velmax_gforce(blockvec,velmax,gforcemax);

    //! 4. Check if motion is G0, or one of G1,G2,G3. If G0, ve=0, else use end ve.
    //blockvec=sc_process_forward_ve(blockvec);

    //! 5. Iterate back over gcode, to check if ve's can be realized given short blocklenghts.


    return blockvec;
}

std::vector<sc_block> sc_optimizer::sc_process_forward_ve(std::vector<sc_block> blockvec){


    T vo=0, ve=0, vm=0, acs=0, ace=0, ncs=0;
    std::vector<sc_engine::sc_period> pvec;

    T curvel=0;
    for(UI i=0; i<blockvec.size(); i++){

        if(blockvec.at(i).type==sc_G0){ //! End velocity=0 if motion is G0, rapid.
            blockvec.at(i).vo=0;
            blockvec.at(i).ve=0;

            //! Set next block vo to 0.
            if(i<blockvec.size()-1){
                blockvec.at(i+1).vo=0;
            }

            curvel=0;

        } else { //! Of type G1,G2,G3.

            acs=0, ace=0; //! To keep it simple, can be used later on to improve this library.
            vo=curvel;
            blockvec.at(i).vo=curvel;
            ncs=blockvec.at(i).blocklenght();

            //! This velmax has already a reduced vm for arc gforce.
            vm=blockvec.at(i).velmax;

            //! The ve is already reduced for corner transitions. If no corner, ve is set to vm by previous function.
            ve=blockvec.at(i).ve;

            //! Sample the ve down until it fits displacement ncs.
            for(UI i=ve; ve>0; ve-=0.1*vm){

                engine->process_curve(sc_engine::id_run,vo,i /*ve*/,acs,ace,ncs,vm,pvec);

                if(engine->to_stot_pvec(pvec)==ncs){
                    blockvec.at(i).ve=i; //! Set the end velocity.
                    curvel=i; //! Update for next iteration.
                    break;
                }
            }
        }
    }
    return blockvec;
}

std::vector<sc_block> sc_optimizer::sc_get_velmax_gforce(std::vector<sc_block> blockvec,
                                                         T velmax, T gforcemax){
    for(UI i=0; i<blockvec.size(); i++){

        if(blockvec.at(i).primitive_id==sc_primitive_id::sc_arc){

            T radius=0;
            sc_arcs().sc_arc_radius(blockvec.at(i).pnt_s,
                                    blockvec.at(i).pnt_w,
                                    blockvec.at(i).pnt_e,radius);
            std::cout<<"radius:"<<radius<<std::endl;

            //! Checks gforce using the program's velmax value.
            T gforce=0;
            sc_get_gforce(velmax,radius,gforce);
            std::cout<<"velmax:"<<velmax<<" gforce:"<<gforce<<std::endl;

            if(gforce>gforcemax){
                T maxvel_arc=0;
                sc_set_gforce(radius,gforcemax,maxvel_arc);
                blockvec.at(i).velmax=maxvel_arc;
            } else {
                blockvec.at(i).velmax=velmax;
            }
        } else { //! For a line set velmax as usual.
            blockvec.at(i).velmax=velmax;
        }
    }
    return blockvec;
}

std::vector<sc_block> sc_optimizer::sc_get_corner_ve_blockangles(std::vector<sc_block> blockvec, T velmax){

    for(UI i=0; i<blockvec.size(); i++){

        if(blockvec.at(i).angle_end_deg<=90){ //! Stop required.
            blockvec.at(i).ve=0;
            //! Set next block vo to zero.
            if(i<blockvec.size()-1){
                blockvec.at(i+1).vo=0;
            }
        }

        if(blockvec.at(i).angle_end_deg>90){ //! Percentage ve up to 180 degrees. //! 180 degrees = colinear.
            T angle_deg=blockvec.at(i).angle_end_deg;
            T factor=(angle_deg-90)/90; //! 0-1, 1=straight on. 0=90 degrees.
            T ve=velmax*factor;

            blockvec.at(i).ve=ve;
            //! Set next block vo to this ve.
            if(i<blockvec.size()-1){
                blockvec.at(i+1).vo=ve;
            }
        }
    }
    return blockvec;
}

std::vector<sc_block> sc_optimizer::sc_get_blockangles(std::vector<sc_block> blockvec){

    if(blockvec.size()>0){ //! Vector safe.
        for(UI i=0; i<blockvec.size()-1; i++){
            T angle_deg=0;
            if(blockvec.at(i).primitive_id==sc_primitive_id::sc_line &&
                    blockvec.at(i+1).primitive_id==sc_primitive_id::sc_line  ){
                line_line_angle(blockvec.at(i).pnt_s,
                                blockvec.at(i).pnt_e,
                                blockvec.at(i+1).pnt_e,angle_deg);
                blockvec.at(i).angle_end_deg=angle_deg;
            }
            if(blockvec.at(i).primitive_id==sc_primitive_id::sc_line &&
                    blockvec.at(i+1).primitive_id==sc_primitive_id::sc_arc  ){
                line_arc_angle(blockvec.at(i).pnt_s,
                               blockvec.at(i).pnt_e,
                               blockvec.at(i+1).pnt_w,
                               blockvec.at(i+1).pnt_e,angle_deg);
                blockvec.at(i).angle_end_deg=angle_deg;
            }
            if(blockvec.at(i).primitive_id==sc_primitive_id::sc_arc &&
                    blockvec.at(i+1).primitive_id==sc_primitive_id::sc_line ){
                arc_line_angle(blockvec.at(i).pnt_s,
                               blockvec.at(i).pnt_w,
                               blockvec.at(i).pnt_e,
                               blockvec.at(i+1).pnt_e,angle_deg);
                blockvec.at(i).angle_end_deg=angle_deg;
            }
            if(blockvec.at(i).primitive_id==sc_primitive_id::sc_arc &&
                    blockvec.at(i+1).primitive_id==sc_primitive_id::sc_arc ){
                arc_arc_angle(blockvec.at(i).pnt_s,
                              blockvec.at(i).pnt_w,
                              blockvec.at(i).pnt_e,
                              blockvec.at(i+1).pnt_w,
                              blockvec.at(i+1).pnt_e,angle_deg);
                blockvec.at(i).angle_end_deg=angle_deg;
            }
        }
    }
    return blockvec;
}

V sc_optimizer::sc_set_gforce(T radius, T gforce, T &vel_mm_sec){

    T circumfence=(radius*2)*M_PI;
    T a=gforce/0.0001;
    T rps= sqrt(a/(4*(M_PI*M_PI)*radius));
    vel_mm_sec=rps*circumfence;
}

V sc_optimizer::sc_get_gforce(T vel_mm_sec, T radius, T &gforce){

    T circumfence=(radius*2)*M_PI;
    T rps=vel_mm_sec/circumfence;
    T a=4*(M_PI*M_PI)*radius*(rps*rps); //! [mm/s^2]
    gforce=a*0.0001; //! [g]
}

V sc_optimizer::line_line_angle(sc_pnt p0, sc_pnt p1, sc_pnt p2, T &angle_deg){

    Eigen::Vector3d p00(p0.x,p0.y,p0.z);
    Eigen::Vector3d p11(p1.x,p1.y,p1.z); //! Common point.
    Eigen::Vector3d p22(p2.x,p2.y,p2.z);

    Eigen::Vector3d v1 = p00-p11;
    Eigen::Vector3d v2 = p22-p11;

    v1.normalize();
    v2.normalize();
    T dot = v1.dot(v2);
    T angle_rad = acos(dot);
    angle_deg = angle_rad*to_degrees;
}

V sc_optimizer::line_arc_angle(sc_pnt p0,
                               sc_pnt p1,
                               sc_pnt p2,
                               sc_pnt p3,
                               T &angle_deg){

    sc_pnt pi;
    sc_arcs().sc_interpolate_arc(p1,p2,p3,0.1,pi);
    line_line_angle(p0,p1,pi,angle_deg);
}

V sc_optimizer::arc_line_angle(sc_pnt p0,
                               sc_pnt p1,
                               sc_pnt p2,
                               sc_pnt p3,
                               T &angle_deg){

    sc_pnt pi;
    sc_arcs().sc_interpolate_arc(p0,p1,p2,0.9,pi);
    line_line_angle(pi,p2,p3,angle_deg);
}

V sc_optimizer::arc_arc_angle(sc_pnt p0,
                              sc_pnt p1,
                              sc_pnt p2,
                              sc_pnt p3,
                              sc_pnt p4,
                              T &angle_deg){

    sc_pnt pi0, pi1;
    sc_arcs().sc_interpolate_arc(p0,p1,p2,0.9,pi0);
    sc_arcs().sc_interpolate_arc(p2,p3,p4,0.1,pi1);
    line_line_angle(pi0,p2,pi1,angle_deg);
}























