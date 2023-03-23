//! Author  : SKynet Cyberdyne
//! Licence : MIT
//! Date    : 2023

/* Forumula cheet sheet.

    Info    : https://www.wolframalpha.com/widgets/view.jsp?id=c778a2d8bf30ef1d3c2d6bc5696defad

    t       time.
    ct      curve time.
    dv      delta velocity, needed to calculate jm.
            time needed from 0 acc to max acc (2*a), back to 0 acc.
            dv is ve-ve.
    ht      half time.
    tf      time front.
    ts      time start.
    te      time end.

    jm      jerk max.
    as      acceleration at inflection point.

    v,vi    velocity.
    vo      velocity start.
    ve      velocity end.

    s,si    displacement.

    a,ai    acceleration.

general concave_convex:
    ct=jm/2*as;
    as=2*a;
    jm=2*as/ct;
    s=abs(ve*ve - vo*vo)/(2*a);
    ct=dv/a;
    ct=abs(ve-vo)/a;
    ht=0.5*ct.

curve t1:
    vi=vo+jm*(t*t)/2;
        vo=vi-(jm*(t*t)/2);
        t=sqrt(2)*sqrt((vi)-vo)/sqrt(jm);
        t=sqrt(-2*vf_+2*ve)/sqrt(jm);


    si=vo*t+jm*(t*t*t)/6;
        t=(cbrt(3*(jm*jm)*s+ sqrt(9*(jm*jm*jm*jm)*(s*s)+8*(jm*jm*jm)*(vo*vo*vo))) / jm ) - ( 2*vo / cbrt(3*(jm*jm)*s+ sqrt(9*(jm*jm*jm*jm)*(s*s)+8*(jm*jm*jm)*(vo*vo*vo))));
    ai=jm*t;
    ts=ai/jm;

curve t3:
    vi=vh + as*t - jm*(t*t)/2;
        t= (as -  sqrt( (as*as) + 2*(vh-vi) *jm))/jm;
        vh = -as*t+((jm*(t*t))/2)+vi

    si=vh*t + as*(t*t)/2 - jm*(t*t*t)/6;
        t= see function solve_for_t
    si+=sh;
    ai=as-jm*t;
    ts=(as-ai)/jm

curve t5:
    vi=vo-jm*(t*t)/2;
        vo=((jm*(t*t))/2)+vi;
        te=(sqrt(2)*sqrt(vf_-ve))/sqrt(jm);
    si=vo*t-jm*(t*t*t)/6;
    ts=ai/jm
    ai=jm*t;

curve t7:
    ve=vh - as*t + jm*(t*t)/2;
        vh=as*t - ((jm*(t*t))/2) + ve;
    ai=as-jm*t;
    ts=(as-acs)/jm;
    si=vh*t - as*(t*t)/2 + jm*(t*t*t)/6

linear_acceleration:
    v=vo + a*t;
    s=vo*t + 0.5*a*(t*t);
        t= abs( (-vo + sqrt(vo*vo - 2*a*s)) / a);
    s=((ve*ve) - (vo*vo))/(2*a)
    v=sqrt((vo*vo) + 2*a*s);

linear_deceleration:
    v=vo - a*t;
    s=vo*t - 0.5*a*(t*t);
        t= abs( (-vo + sqrt(vo*vo - 2*a*s)) / a);
    s=((vo*vo) - (ve*ve))/(2*a)
    v=sqrt((vo*vo) - 2*a*s);

linear_general:
    ct=abs(ve-vo)/a

steady:
    s=v*t;
    t=s/v;
    v=s/t;
*/


























