#ifndef SC_LINES_H
#define SC_LINES_H

#include <../sc_struct.h>

class sc_lines
{
public:
    sc_lines();

    void sc_interpolate_pnt(sc_pnt p0,
                            sc_pnt p1,
                            T progress,
                            sc_pnt &pi);

    void sc_interpolate_dir(sc_dir d0,
                            sc_dir d1,
                            T progress,
                            sc_dir &di);

    void sc_interpolate_ext(sc_ext e0,
                            sc_ext e1,
                            T progress,
                            sc_ext &ei);

    T sc_interpolate_line(T start,
                          T end,
                          T progress);

    T sc_line_lenght(sc_pnt p0, sc_pnt p1);
};

#endif
