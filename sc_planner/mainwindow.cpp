#include "mainwindow.h"
#include "./ui_mainwindow.h"

std::vector<double> vvec,svec,avec;
int gui_delay=0;
std::vector<double> block;
double position=0;
double t_planner=0, t_pause=0, t_resume=0, s=0, old_s=0, v=0, a=0, spause=0, spause_old=0;
bool finished=0, pause_finished=0, resume_init=0, resume_finished;
std::vector<sc_engine::sc_period> pvec, pausevec, resumevec, testvec;
bool run=0, pause=0, pause_init=0, run_init=0;
std::vector<sc_engine::sc_period> motionvec;
uint motionvec_nr=0;
bool motion_reverse=0, pause_ready=0, resume=0;
bool vm_interupt=0;
double interupt_v=0,interupt_s=0,interupt_a=0;
double ace=0, ve=0, startpos=0, endpos=0;

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

//! This function simulates the servo cycle. And is called every 1 millisecond.
void MainWindow::thread(){


    if(vm_interupt){

        T ncs=endpos-position;

        testvec.clear();
        engine->process_curve({sc_engine::sc_period_id::id_none,v,ve,a,ace,ncs,position,endpos},ui->doubleSpinBox_vm->value(),testvec);


        if(engine->to_stot_pvec(testvec)==ncs){ //! Apply interupt if it fits in motion s.

            s=0;
            old_s=0;
            t_planner=0;

            pvec.clear();
            pvec=testvec;
        }

        vm_interupt=0;
    }


    if(run && !pause && !resume && motionvec.size()>0){

       // std::cout<<"run"<<std::endl;

        if(!run_init){
            motionvec_nr=0;
            pvec.clear();
            finished=1;
            t_planner=0;
            run_init=1;
            old_s=0;
            s=0;
            v=0;
            a=0;
        }

        if(finished){
            if(motionvec_nr<motionvec.size()){

                sc_engine::sc_period p=motionvec.at(motionvec_nr);
                p.ncs=p.endpos-p.startpos;
                ace=p.ace;
                ve=p.ve;
                startpos=p.startpos;
                endpos=p.endpos;

                engine->process_curve(p,ui->doubleSpinBox_vm->value(),pvec);
                if(p.endpos<p.startpos){
                    motion_reverse=1;
                } else {
                    motion_reverse=0;
                }
                motionvec_nr++;
                finished=0;
            }
        }

        engine->interpolate_periods(t_planner,pvec,s,v,a,finished);
        if(!motion_reverse){
            position+=s-old_s;
        } else {
            position-=s-old_s;
        }
        old_s=s;

        if(!finished){
            vvec.push_back(v);
            svec.push_back(position);
            avec.push_back(a);

            myOpenGl->setj0vec(vvec);
            myOpenGl->setj1vec(svec);
            myOpenGl->setj2vec(avec);

            t_planner+=0.01;
        }
    }

    if(pause){

       // std::cout<<"pause"<<std::endl;

        if(!pause_init){
            pausevec.clear();
            pause_finished=0;

            interupt_v=v;
            interupt_s=position;
            interupt_a=a;

            T ve=0, ace=0;
            engine->process_curve({sc_engine::sc_period_id::id_none,v,ve,a,ace},ui->doubleSpinBox_vm->value(),pausevec);
            t_pause=0;
            spause=0;
            spause_old=0;
            pause_init=1;
        }

        engine->interpolate_periods(t_pause,pausevec,spause,v,a,pause_finished);

        if(!pause_finished){
            if(!motion_reverse){
                position+=spause-spause_old;
            } else {
                position-=spause-spause_old;
            }
            spause_old=spause;

            vvec.push_back(v);
            svec.push_back(position);
            avec.push_back(a);

            myOpenGl->setj0vec(vvec);
            myOpenGl->setj1vec(svec);
            myOpenGl->setj2vec(avec);
        }
        t_pause+=0.01;
    }

    if(resume){ //! Pause resume.

       // std::cout<<"resume"<<std::endl;

        if(!resume_init){
            resumevec.clear();

            T vo=0, acs=0, tarpos=0;

            tarpos=std::abs(position-interupt_s);

            engine->process_curve({sc_engine::sc_period_id::id_none,vo,interupt_v,acs,interupt_a,tarpos},ui->doubleSpinBox_vm->value(),resumevec);
            t_resume=0;

            spause=0;
            spause_old=0;
            resume_finished=0;
            resume_init=1;
        }

        engine->interpolate_periods(t_resume,resumevec,spause,v,a,resume_finished);

        if(!resume_finished){

            if(!motion_reverse){
                position-=spause-spause_old;
            } else {
                position+=spause-spause_old;
            }

            spause_old=spause;

            vvec.push_back(v);
            svec.push_back(position);
            avec.push_back(a);

            myOpenGl->setj0vec(vvec);
            myOpenGl->setj1vec(svec);
            myOpenGl->setj2vec(avec);

        }
        if(resume_finished){
            resume=0;
        }
        t_resume+=0.01;
    }

    gui_delay++;
    if(gui_delay>10){
        ui->label_planner_pos->setText(QString::number(position,'f',3));

        if(motion_reverse){
            ui->checkBox_motion_reverse->setChecked(1);
        } else {
            ui->checkBox_motion_reverse->setChecked(0);
        }
        gui_delay=0;
    }
}

void MainWindow::on_pushButton_planner_pressed()
{
   // std::cout<<"planner pressed"<<std::endl;

    vvec.clear();
    svec.clear();
    avec.clear();

    T vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=0;

    motionvec.clear();

    vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=200;
    motionvec.push_back({sc_engine::sc_period_id::id_none,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    // vo=0, ve=0, acs=0, ace=0, ncs=200, nct=0, startpos=100, endpos=200;
    //  motionvec.push_back({sc_engine::sc_period_id::id_none,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    run=1;
    run_init=0;
    pause=0;
    pause_init=0;
    resume=0;
    resume_init=0;

    position=0;
}

void MainWindow::on_pushButton_planner_pause_pressed()
{
    if(pause==1){
        return;
    }
    pause=1;
    pause_init=0;
    resume=0;
}

void MainWindow::on_pushButton_resume_pressed()
{
    if(resume==1){
        return;
    }
    resume=1;
    resume_init=0;
    pause=0;
}


void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    arg1=0;
    vm_interupt=1;
}
