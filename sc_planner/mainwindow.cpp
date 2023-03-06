#include "mainwindow.h"
#include "./ui_mainwindow.h"

//! If user reqeust a pause, motion will do a controlled stop. For now test on the steady stage.
//! A pause resume will go back to pause interupt position, then will continue path.

//! Gui items.
std::vector<double> vvec,svec,avec;
int gui_delay=0; //! To keep a good gui performance using opengl.

//! Vectors to store motion data.
std::vector<sc_engine::sc_period>  motionvec, runvec, pausevec, resumevec, vmvec;

UI motionvec_nr=0;      //! Current motionvec counter.

B run_finished=0;       //! Flags.
B pause_finished=0;
B pause_resume_finished=0;
B run=0;
B pause=0;
B pause_resume=0;
B run_init=0;
B pause_init=0;
B pause_resume_init=0;
B motion_reverse=0;
B vm_interupt=0;

T position=0;        //! Overall absolute position.
T velocity=0;
T acceleration=0;
T store_velocity=0;
T store_displacement=0;
T store_acceleration=0;
T stored_ace=0;
T stored_ve=0;
T store_startpos=0;
T stored_endpos=0;
T timer_run=0;
T timer_pause=0;
T timer_resume=0;
T displacement_run=0;
T displacement_run_old=0;
T displacement_pause=0;
T displacement_pause_old=0;
T displacement_resume=0;
T displacement_resume_old=0;

T millisecond=0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(25,25);
    myOpenGl->setInterval(0.01);
    myOpenGl->set2VecShift(100);
    myOpenGl->set1VecScale(0.1);

    //! Timer to simulate servo cycle.
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);

    engine->sc_set_a_dv(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value());
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::clear_opengl(){

    vvec.clear();
    svec.clear();
    avec.clear();
}

void MainWindow::set_opengl(T vel, T pos, T acc){

    vvec.push_back(vel);
    svec.push_back(pos);
    avec.push_back(acc);

    myOpenGl->setj0vec(vvec);
    myOpenGl->setj1vec(svec);
    myOpenGl->setj2vec(avec);
}

//! This function simulates the servo cycle. And is called every 1 millisecond.
void MainWindow::thread(){

    auto start = std::chrono::high_resolution_clock::now();

    if(vm_interupt && !pause && !pause_resume){

        T ncs=stored_endpos-position;

        vmvec.clear();
        engine->process_curve({sc_engine::sc_period_id::id_none,
                               velocity,
                               stored_ve,
                               acceleration,
                               stored_ace,
                               ncs,
                               position,
                               stored_endpos}, ui->doubleSpinBox_vm->value(), vmvec);

        if(engine->to_stot_pvec(vmvec)==ncs){ //! Apply interupt if it fits in motion s.

            displacement_run=0;
            displacement_run_old=0;
            timer_run=0;

            runvec.clear();
            runvec=vmvec;
        }

        vm_interupt=0;
    }

    if(run && !pause && !pause_resume && motionvec.size()>0){

        if(!run_init){
            motionvec_nr=0;
            runvec.clear();
            run_finished=1;
            timer_run=0;
            displacement_run_old=0;
            displacement_run=0;
            velocity=0;
            acceleration=0;
            run_init=1;
        }
        if(run_finished){
            if(motionvec_nr<motionvec.size()){

                sc_engine::sc_period p=motionvec.at(motionvec_nr);
                p.ncs=p.endpos-p.startpos;
                stored_ace=p.ace;
                stored_ve=p.ve;
                store_startpos=p.startpos;
                stored_endpos=p.endpos;

                engine->process_curve(p,ui->doubleSpinBox_vm->value(),runvec);
                if(p.endpos<p.startpos){
                    motion_reverse=1;
                } else {
                    motion_reverse=0;
                }
                motionvec_nr++;
                run_finished=0;
            }
        }

        engine->interpolate_periods(timer_run,runvec,displacement_run,velocity,acceleration,run_finished);

        if(!run_finished){
            if(!motion_reverse){
                position+=displacement_run-displacement_run_old;
            } else {
                position-=displacement_run-displacement_run_old;
            }
            displacement_run_old=displacement_run;

            set_opengl(velocity,position,acceleration);
        }
        timer_run+=0.01;
    }

    if(pause){

        if(!pause_init){
            pausevec.clear();
            pause_finished=0;

            store_velocity=velocity;
            store_displacement=position;
            store_acceleration=acceleration;

            T ve=0, ace=0;
            engine->process_curve({sc_engine::sc_period_id::id_none,
                                   velocity,
                                   ve,
                                   acceleration,
                                   ace}, ui->doubleSpinBox_vm->value(), pausevec);
            timer_pause=0;
            displacement_pause=0;
            displacement_pause_old=0;
            pause_init=1;
        }

        engine->interpolate_periods(timer_pause,pausevec,displacement_pause,velocity,acceleration,pause_finished);

        if(std::isnan(displacement_pause)){
            displacement_pause=displacement_pause_old;
        }

        if(!pause_finished){
            if(!motion_reverse){
                position+=displacement_pause-displacement_pause_old;
            } else {
                position-=displacement_pause-displacement_pause_old;
            }

            displacement_pause_old=displacement_pause;

            set_opengl(velocity,position,acceleration);
        }
        timer_pause+=0.01;
    }

    if(pause_resume){

        if(!pause_resume_init){
            resumevec.clear();

            T vo=0, acs=0, tarpos=0;

            tarpos=std::abs(position-store_displacement);

            engine->process_curve({sc_engine::sc_period_id::id_none,
                                   vo,
                                   store_velocity,
                                   acs,
                                   store_acceleration,
                                   tarpos}, ui->doubleSpinBox_vm->value(), resumevec);
            timer_resume=0;

            displacement_resume=0;
            displacement_resume_old=0;
            pause_resume_finished=0;
            pause_resume_init=1;
        }

        engine->interpolate_periods(timer_resume,resumevec,displacement_resume,velocity,acceleration,pause_resume_finished);

        if(!pause_resume_finished){

            if(!motion_reverse){
                position-=displacement_resume-displacement_resume_old;
            } else {
                position+=displacement_resume-displacement_resume_old;
            }

            displacement_resume_old=displacement_resume;

            set_opengl(velocity,position,acceleration);

        }
        if(pause_resume_finished){
            pause_resume=0;
        }
        timer_resume+=0.01;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    millisecond=ns.count()*ns_to_ms;

    gui_delay++;
    if(gui_delay>10){
        ui->label_planner_pos->setText(QString::number(position,'f',3));

        if(motion_reverse){
            ui->checkBox_motion_reverse->setChecked(1);
        } else {
            ui->checkBox_motion_reverse->setChecked(0);
        }

        ui->label_performance->setText(QString::number(millisecond,'f',6));

        gui_delay=0;
    }
}

void MainWindow::on_pushButton_planner_pressed()
{
    clear_opengl();

    T vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=0;

    motionvec.clear();

    vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=200;
    motionvec.push_back({sc_engine::sc_period_id::id_none,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    vo=0, ve=0, acs=0, ace=0, ncs=200, nct=0, startpos=200, endpos=-50;
    motionvec.push_back({sc_engine::sc_period_id::id_none,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    run=1;
    run_init=0;
    pause=0;
    pause_init=0;
    pause_resume=0;
    pause_resume_init=0;

    position=0; //! To avoid a position reset, out-comment this one.
}

void MainWindow::on_pushButton_planner_pause_pressed()
{
    if(pause==1){
        return;
    }
    pause=1;
    pause_init=0;
    pause_resume=0;
}

void MainWindow::on_pushButton_resume_pressed()
{
    if(!pause || !pause_finished){
        std::cerr<<"no pause or pause busy."<<std::endl;
        return;
    }
    if(pause_resume==1){
        return;
    }
    pause_resume=1;
    pause_resume_init=0;
    pause=0;
}


void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    arg1=0;
    vm_interupt=1;
}
