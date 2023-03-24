#include "sc_planner.h"

sc_planner::sc_planner()
{

}

V sc_planner::sc_set_a_dv(T acceleration, T delta_velocity){
    engine->sc_set_a_dv(acceleration,delta_velocity);
}

V sc_planner::sc_set_maxvel(T velocity_max){
    sc_vm=velocity_max;
}

V sc_planner::sc_set_adaptive_feed(T adaptive_feed){
    sc_adaptive_feed=adaptive_feed;
}

V sc_planner::sc_set_state(sc_enum_program_status state){
    sc_enum_program_state=state;
}

V sc_planner::sc_set_interval(T time){
    sc_interval=time;
}

V sc_planner::sc_set_startline(UI startline){
    sc_motionvec_nr=startline;
}

V sc_planner::sc_set_position(T position){
    sc_pos=position;
}

V sc_planner::sc_clear(){
    std::cerr<<"clear all."<<std::endl;
    pvec.clear();
    motionvec.clear();
    blockvec.clear();
    sc_reset();
}

V sc_planner::sc_reset(){
    std::cerr<<"reset."<<std::endl;
    sc_run_flag=0;
    sc_newpos=0;
    sc_oldpos=0;
    sc_timer=0;
}

V sc_planner::sc_add_line_motion(T vo, T ve, T acs, T ace, sc_pnt start, sc_pnt end, sc_type type){

    T ncs=0, nct=0;

    sc_block b;
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=type;
    b.set_pnt(start,end);
    ncs=b.blocklenght();
    blockvec.push_back(b);

    motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct});
}

V sc_planner::sc_add_arc_motion(T vo, T ve, T acs, T ace, sc_pnt start, sc_pnt way, sc_pnt end, sc_type type){

    T ncs=0, nct=0;

    sc_block b;
    b.primitive_id=sc_primitive_id::sc_arc;
    b.type=type;
    b.set_pnt(start,way,end);
    ncs=b.blocklenght();
    blockvec.push_back(b);

    motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct});
}

V sc_planner::sc_add_general_motion(T vo,
                                    T ve,
                                    T acs,
                                    T ace,
                                    sc_primitive_id id,
                                    sc_type type,
                                    sc_pnt start,
                                    sc_pnt way,
                                    sc_pnt end,
                                    sc_dir dir_start,
                                    sc_dir dir_end,
                                    sc_ext ext_start,
                                    sc_ext ext_end){

    T ncs=0, nct=0;

    sc_block b;
    b.primitive_id=id;
    b.type=type;
    b.set_pnt(start,way,end);
    b.set_dir(dir_start,dir_end);
    b.set_ext(ext_start,ext_end);
    ncs=b.blocklenght();
    blockvec.push_back(b);

    motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct});

}

B sc_planner::sc_set_traject_stot(){

    sc_stot=0;
    for(UI i=0; i<motionvec.size(); i++){
        std::vector<sc_engine::sc_period> tempvec;
        engine->process_curve(motionvec.at(i),sc_vm,tempvec);
        sc_stot+=engine->to_stot_pvec(tempvec);
    }
    if(sc_stot==0){
        std::cerr<<"traject stot error"<<std::endl;
        return 0;
    }
    return 1;
}

V sc_planner::sc_get_motionvec_nr_from_position(T &motionvec_nr, T &motionvec_nr_progress, T &motionvec_nr_dtg){

    T l=0, s=0;

    motionvec_nr=0;
    motionvec_nr_progress=0;
    motionvec_nr_dtg=0;

    for(UI i=0; i<motionvec.size(); i++){
        std::vector<sc_engine::sc_period> tempvec;
        engine->process_curve(motionvec.at(i),sc_vm,tempvec);
        l=engine->to_stot_pvec(tempvec);

        if(sc_pos>=s && sc_pos <=s+l){
            motionvec_nr=i;
            motionvec_nr_dtg=l-(sc_pos-s);
            motionvec_nr_progress= (sc_pos-s) /l;
            break;
        }
        s+=l;
    }
}

V sc_planner::sc_update(){

    auto start_clock = std::chrono::high_resolution_clock::now();

    switch (sc_enum_program_state) {

    case program_run:
        sc_run_state();
        break;
    case program_pause:
        sc_pause_state();
        break;
    case program_pause_resume:
        sc_pause_resume_state();
        break;
    case program_stop:
        sc_stop_state();
        break;
    case program_vm_interupt:
        sc_vm_interupt_state();
        break;
    case program_end:
        sc_program_end_state();
        break;
    case program_error:
        sc_error_state();
        break;
    case program_wait:

        break;
    }

    auto stop_clock = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_clock - start_clock);
    sc_ms=ns.count()*ns_to_ms;
}

V sc_planner::sc_run_state(){

    if(!sc_run_flag){
        sc_enum_run_state=run_check; //! Check if parameters and motionvec is ok.
        sc_run_flag=1;
    }

    switch (sc_enum_run_state) {

    case run_check:
        sc_run_check();
        break;
    case run_init:
        sc_run_init(); //! Init motionblock.
        break;
    case run_cycle:
        sc_run_cycle(); //! Run motionblock.
        break;
    }

    sc_last_program_state=program_run;
}

V sc_planner::sc_run_check(){

    if(sc_check_motionvec_loaded() &&
            sc_check_motionvec_startline() &&
            sc_check_servo_cycletime() &&
            sc_check_vm() &&
            sc_set_traject_stot()){
        sc_enum_run_state=run_init;

        sc_newpos=0;
        sc_oldpos=0;
        sc_timer=0;
    } else {
        sc_enum_program_state=program_error;
    }
}

V sc_planner::sc_run_init(){

    pvec.clear();
    engine->process_curve(motionvec.at(sc_motionvec_nr),sc_vm,pvec);
    sc_timer=0;
    sc_oldpos=0;
    sc_newpos=0;
    sc_enum_run_state=run_cycle;
}

V sc_planner::sc_run_cycle(){

    B finished=0;

    sc_oldpos=sc_newpos;
    //! Process current motionvec_nr.
    engine->interpolate_periods(sc_timer,pvec,sc_newpos,sc_vel,sc_acc,finished);

    sc_pos+=sc_newpos-sc_oldpos;

    sc_timer+=sc_interval*sc_adaptive_feed;
    if(sc_timer<0){sc_timer=0;} //! Limit negative adaptive feed.

    T motionvec_nr,motionvec_nr_dtg,motionvec_nr_progress;
    sc_get_motionvec_nr_from_position(motionvec_nr,motionvec_nr_progress,motionvec_nr_dtg);
    //! Don't overwrite the motionvec_nr in the run cycle. Only in pause or stop cycle.
    sc_motionvec_nr_dtg=motionvec_nr_dtg;
    sc_motionvec_nr_progress=motionvec_nr_progress;

    if(finished){
        if(sc_motionvec_nr<motionvec.size()-1){
            sc_motionvec_nr++;
            sc_enum_run_state=run_init;
        } else {
            sc_enum_program_state=program_end;
        }
    }
}

V sc_planner::sc_pause_state(){

    if(sc_last_program_state==program_stop){
        std::cerr<<"pause refused after program stop"<<std::endl;
        sc_enum_program_state=program_wait;
        return;
    }

    switch (sc_enum_pause_state) {
    case pause_init:
        sc_pause_init();
        break;
    case pause_cycle:
        sc_pause_cycle();
        break;
    }
    sc_last_program_state=program_pause;
}

V sc_planner::sc_pause_init(){

    T ve=0, ace=0, ncs=0;

    pvec.clear();
    engine->process_curve(sc_engine::sc_period_id::id_pause,
                          sc_vel, ve, sc_acc, ace, ncs, sc_vm, pvec);
    sc_timer=0;
    sc_oldpos=0;
    sc_newpos=0;

    sc_enum_pause_state=pause_cycle;
}

V sc_planner::sc_pause_cycle(){

    B finished=0;

    sc_oldpos=sc_newpos;

    engine->interpolate_periods(sc_timer,pvec,sc_newpos,sc_vel,sc_acc,finished);

    sc_pos+=sc_newpos-sc_oldpos;

    sc_timer+=sc_interval*sc_adaptive_feed;
    if(sc_timer<0){sc_timer=0;} //! Limit negative adaptive feed.

    //! Update current motionvec_nr if crossing a waypoint.
    T motionvec_nr,motionvec_nr_dtg,motionvec_nr_progress;
    sc_get_motionvec_nr_from_position(motionvec_nr,motionvec_nr_progress,motionvec_nr_dtg);
    sc_motionvec_nr=motionvec_nr;
    sc_motionvec_nr_dtg=motionvec_nr_dtg;
    sc_motionvec_nr_progress=motionvec_nr_progress;

    if(finished){
        sc_enum_pause_state=pause_init; //! Reset to init for next pause request

        if(sc_vel>0){ //! Pause was not finished. Perform next pause sequence.
            std::cerr<<"pause repeated to ve=0."<<std::endl;
            sc_enum_program_state=program_pause;
        } else {
            sc_enum_program_state=program_wait;
        }
    }
}

V sc_planner::sc_pause_resume_state(){

    if(sc_last_program_state==program_pause){ //! Resume only after pause.
        switch (sc_enum_pause_resume_state) {
        case pause_resume_init:
            sc_pause_resume_init();
            break;
        case pause_resume_cycle:
            sc_pause_resume_cycle();
            break;
        }
    } else {
        std::cerr<<"pause-resume sequence valid after pause."<<std::endl;
        sc_enum_program_state=program_wait;
    }
}

V sc_planner::sc_pause_resume_init(){

    T ncs=sc_motionvec_nr_dtg;

    pvec.clear();
    engine->process_curve(sc_engine::sc_period_id::id_run,
                          sc_vel, motionvec.at(sc_motionvec_nr).ve, sc_acc,
                          motionvec.at(sc_motionvec_nr).ace, ncs, sc_vm, pvec);
    sc_timer=0;
    sc_oldpos=0;
    sc_newpos=0;

    sc_enum_pause_resume_state=pause_resume_cycle;
}

V sc_planner::sc_pause_resume_cycle(){

    sc_enum_pause_resume_state=pause_resume_init; //! Reset to init for next pause_resume request
    sc_enum_program_state=program_run;
    sc_enum_run_state=run_cycle;  //! Finish pause resume in program run cycle.
}

V sc_planner::sc_stop_state(){

    if(sc_last_program_state==program_end){
        std::cerr<<"program already finished"<<std::endl;
        return;
    }

    switch (sc_enum_stop_state) {
    case stop_init:
        sc_stop_init();
        break;
    case stop_cycle:
        sc_stop_cycle();
        break;
    }
    sc_last_program_state=program_stop;
}

V sc_planner::sc_stop_init(){

    //! Create a pause curve from current speed.
    T ve=0, ace=0, ncs=0;

    pvec.clear();
    engine->process_curve(sc_engine::sc_period_id::id_pause,
                          sc_vel, ve, sc_acc, ace, ncs, sc_vm, pvec);
    sc_timer=0;
    sc_oldpos=0;
    sc_newpos=0;

    sc_enum_stop_state=stop_cycle;
}

V sc_planner::sc_stop_cycle(){

    B finished=0;

    sc_oldpos=sc_newpos;

    engine->interpolate_periods(sc_timer,pvec,sc_newpos,sc_vel,sc_acc,finished);

    sc_pos+=sc_newpos-sc_oldpos;

    sc_timer+=sc_interval*sc_adaptive_feed;
    if(sc_timer<0){sc_timer=0;} //! Limit negative adaptive feed.

    //! Update current motionvec_nr if crossing a waypoint.
    T motionvec_nr,motionvec_nr_dtg,motionvec_nr_progress;
    sc_get_motionvec_nr_from_position(motionvec_nr,motionvec_nr_progress,motionvec_nr_dtg);
    sc_motionvec_nr=motionvec_nr;
    sc_motionvec_nr_dtg=motionvec_nr_dtg;
    sc_motionvec_nr_progress=motionvec_nr_progress;

    if(finished){
        sc_enum_stop_state=stop_init; //! Reset to init for next stop request

        if(sc_vel>0){
            std::cerr<<"stop repeated to ve=0."<<std::endl;
            sc_enum_program_state=program_stop;
        } else {
            sc_enum_program_state=program_end;
        }
    }
}

V sc_planner::sc_vm_interupt_state(){

    switch (sc_enum_vm_interupt_state) {
    case vm_interupt_init:
        sc_vm_interupt_init();
        break;
    case vm_interupt_cycle:
        sc_vm_interupt_cycle();
        break;
    }
}

V sc_planner::sc_vm_interupt_init(){

    if(motionvec.size()==0){
        sc_enum_program_state=program_wait;
        return;
    }

    if(sc_last_program_state==program_run){

        T ncs=sc_motionvec_nr_dtg;

        std::vector<sc_engine::sc_period> tempvec;
        engine->process_curve(sc_engine::sc_period_id::id_run,
                              sc_vel, motionvec.at(sc_motionvec_nr).ve, sc_acc,
                              motionvec.at(sc_motionvec_nr).ace, ncs, sc_vm, tempvec);

        if(engine->to_stot_pvec(tempvec)<=ncs+0.00001){ //! Allow a vm interupt if interupt curve fits dtg.

            pvec.clear();
            pvec=tempvec;

            sc_timer=0;
            sc_oldpos=0;
            sc_newpos=0;

        }
        sc_enum_vm_interupt_state=vm_interupt_cycle;
    } else {
        sc_enum_program_state=program_wait;
    }
}

V sc_planner::sc_vm_interupt_cycle(){
    sc_enum_vm_interupt_state=vm_interupt_init;
    sc_enum_program_state=program_run;
    sc_enum_run_state=run_cycle;
}

V sc_planner::sc_program_end_state(){
    sc_last_program_state=program_end;
}

V sc_planner::sc_error_state(){

}

V sc_planner::sc_get_planner_results(T &position,
                                     T &velocity,
                                     T &acceleration,
                                     UI &line_nr,
                                     T &line_progress,
                                     T &traject_progress,
                                     B &finished){
    position=sc_pos;
    velocity=sc_vel;
    acceleration=sc_acc;

    line_nr=sc_motionvec_nr;
    line_progress=sc_motionvec_nr_progress;

    traject_progress=sc_pos/sc_stot;

    if(sc_enum_program_state==program_end){
        finished=true;
    }
}

V sc_planner::sc_get_interpolation_results(sc_pnt &xyz, sc_dir &abc, sc_ext &uvw, T &curve_progress){
    T traject_progress=sc_pos/sc_stot;
    interpolate->interpolate_blockvec(blockvec,
                                      traject_progress,
                                      xyz,abc,uvw,curve_progress);
}

V sc_planner::sc_get_program_state(sc_enum_program_status &state){
    state=sc_enum_program_state;
}

std::string sc_planner::sc_get_program_state(){
    if(sc_enum_program_state==program_run){
        return "program_run";
    }
    if(sc_enum_program_state==program_pause){
        return "program_pause";
    }
    if(sc_enum_program_state==program_pause_resume){
        return "program_pause_resume";
    }
    if(sc_enum_program_state==program_stop){
        return "program_stop";
    }
    if(sc_enum_program_state==program_vm_interupt){
        return "program_vm_interupt";
    }
    if(sc_enum_program_state==program_wait){
        return "program_wait";
    }
    if(sc_enum_program_state==program_end){
        return "program_end";
    }
    if(sc_enum_program_state==program_error){
        return "program_error";
    }
    return "no state";
}

B sc_planner::sc_get_run_cycle_state(){
    if(sc_enum_program_state==program_run){
        return 1;
    }
    return 0;
}

B sc_planner::sc_get_pause_cycle_state(){
    if(sc_enum_program_state==program_pause){
        return 1;
    }
    return 0;
}

B sc_planner::sc_get_stop_cycle_state(){
    if(sc_enum_program_state==program_stop || sc_enum_program_state==program_end){
        return 1;
    }
    return 0;
}

T sc_planner::sc_performance(){
    return sc_ms;
}

B sc_planner::sc_check_motionvec_loaded(){
    if(motionvec.size()==0){
        std::cerr<<"no motion loaded error."<<std::endl;
        return 0;
    }
    return 1;
}

B sc_planner::sc_check_motionvec_startline(){
    if(sc_motionvec_nr>motionvec.size()-1){
        std::cerr<<"startline input error."<<std::endl;
        return 0;
    }

    //! Check if sc_position matches the program start line.
    T l=0, s=0;

    for(UI i=0; i<motionvec.size(); i++){
        std::vector<sc_engine::sc_period> tempvec;
        engine->process_curve(motionvec.at(i),sc_vm,tempvec);
        l=engine->to_stot_pvec(tempvec);

        if(sc_motionvec_nr==i){
            sc_set_position(s);
            break;
        }
        s+=l;
    }
    return 1;
}

B sc_planner::sc_check_servo_cycletime(){
    if(sc_interval<=0){
        std::cerr<<"servo cycletime input error."<<std::endl;
        return 0;
    }
    return 1;
}

B sc_planner::sc_check_vm(){
    if(sc_vm<=0){
        std::cerr<<"velocity max input error."<<std::endl;
        return 0;
    }
    return 1;
}

V sc_planner::sc_optimize(UI range_begin, UI range_end){

    //! Use the sc_optimizer to optimize the given gcode input.
    T gforcemax=0.022; //! Gforce for : circle 6mm diameter, 1400mm/min.

    optimizer->sc_set_a_dv_gforce_velmax(engine->a,engine->dv,gforcemax,sc_vm);

    blockvec=optimizer->sc_optimize_all(blockvec);

    //! Update the motionvec.
    for(UI i=std::max(0.0,T(range_begin)); i<std::min(T(range_end),T(blockvec.size())); i++){

        T vo=blockvec.at(i).vo;
        T ve=blockvec.at(i).ve;
        T velmax=blockvec.at(i).velmax;
        T acs=0, ace=0;
        T ncs=blockvec.at(i).blocklenght();
        T nct=0;

        motionvec.at(i)={sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct};
    }

    //! optimizer->sc_print_blockvec(blockvec);
}







































