#include "sc_lines.h"

sc_lines::sc_lines()
{

}

void sc_lines::sc_interpolate_pnt(sc_pnt p0, sc_pnt p1, T progress, sc_pnt &pi){
    pi.x=sc_interpolate_line(p0.x,p1.x,progress);
    pi.y=sc_interpolate_line(p0.y,p1.y,progress);
    pi.z=sc_interpolate_line(p0.z,p1.z,progress);
}

void sc_lines::sc_interpolate_dir(sc_dir d0, sc_dir d1, T progress, sc_dir &di){
    di.a=sc_interpolate_line(d0.a,d1.a,progress);
    di.b=sc_interpolate_line(d0.b,d1.b,progress);
    di.c=sc_interpolate_line(d0.c,d1.c,progress);
}

void sc_lines::sc_interpolate_ext(sc_ext e0, sc_ext e1, T progress, sc_ext &ei){
    ei.u=sc_interpolate_line(e0.u,e1.u,progress);
    ei.v=sc_interpolate_line(e0.v,e1.v,progress);
    ei.w=sc_interpolate_line(e0.w,e1.w,progress);
}

T sc_lines::sc_interpolate_line(T start, T end, T progress){
    return start+(progress*(end-start));
}

T sc_lines::sc_line_lenght(sc_pnt p0, sc_pnt p1){
    return sqrt(pow(p1.x-p0.x,2)+pow(p1.y-p0.y,2)+pow(p1.z-p0.z,2));
}
