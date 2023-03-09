#ifndef SC_INTERPOLATE_H
#define SC_INTERPOLATE_H

//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023
//!
#include "sc_lines.h"
#include "sc_arcs.h"

class sc_interpolate
{
public:
    sc_interpolate();



    enum sc_primitive_id {
        id_line,
        id_arc,
    };

    struct sc_block {
    public:
        sc_primitive_id primitive_id=id_line;

        V set_pnt(sc_pnt start, sc_pnt end);
        V set_pnt(sc_pnt start, sc_pnt way, sc_pnt end);
        V set_dir(sc_dir start, sc_dir end);
        V set_ext(sc_ext start, sc_ext end);

        T blocklenght();

        sc_pnt pnt_s, pnt_e, pnt_w;
        sc_dir dir_s, dir_e;
        sc_ext ext_s, ext_e;
    };


    V interpolate_blockvec(std::vector<sc_block> blockvec,
                           T traject_progress, //! 0-1
                           sc_pnt &pnt,
                           sc_dir &dir,
                           sc_ext &ext,
                           T &curve_progress);
};

#endif
