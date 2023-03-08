#include "sc_interpolate.h"

sc_interpolate::sc_interpolate()
{

}

T sc_interpolate::sc_block::blocklenght(){

    if(primitive_id==id_line){
        return sc_lines().sc_line_lenght(pnt_s,pnt_e);
    }
    if(primitive_id==id_arc){
        return sc_arcs().sc_arc_lenght(pnt_s,pnt_w,pnt_e);
    }
    return 0;
}

V sc_interpolate::sc_block::interpolate(T progress,
                                        sc_pnt &pnt,
                                        sc_dir &dir,
                                        sc_ext &ext){

    if(primitive_id==id_line){
        sc_lines().sc_interpolate_lin(pnt_s,pnt_e,progress,pnt);
    }
    if(primitive_id==id_arc){
        sc_arcs().sc_interpolate_arc(pnt_s,pnt_w,pnt_e,progress,pnt);
    }

    sc_lines().sc_interpolate_dir(dir_s,dir_e,progress,dir);
    sc_lines().sc_interpolate_ext(ext_s,ext_e,progress,ext);
}

V sc_interpolate::sc_block::set_pnt(sc_pnt start, sc_pnt end){
    pnt_s=start;
    pnt_e=end;
}

V sc_interpolate::sc_block::set_pnt(sc_pnt start, sc_pnt way, sc_pnt end){
    pnt_s=start;
    pnt_w=way;
    pnt_e=end;
}

V sc_interpolate::sc_block::set_dir(sc_dir start, sc_dir end){
    dir_s=start;
    dir_e=end;
}

V sc_interpolate::sc_block::set_ext(sc_ext start, sc_ext end){
    ext_s=start;
    ext_e=end;
}
