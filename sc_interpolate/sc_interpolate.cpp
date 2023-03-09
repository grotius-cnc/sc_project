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

V sc_interpolate::interpolate_block(sc_block block,
                                              T progress,
                                              sc_pnt &pnt,
                                              sc_dir &dir,
                                              sc_ext &ext){

    if(block.primitive_id==id_line){
        sc_lines().sc_interpolate_lin(block.pnt_s,block.pnt_e,progress,pnt);
    }
    if(block.primitive_id==id_arc){
        sc_arcs().sc_interpolate_arc(block.pnt_s,block.pnt_w,block.pnt_e,progress,pnt);
    }

    sc_lines().sc_interpolate_dir(block.dir_s,block.dir_e,progress,dir);
    sc_lines().sc_interpolate_ext(block.ext_s,block.ext_e,progress,ext);
}




















