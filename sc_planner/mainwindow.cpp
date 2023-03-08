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
UI start_line_nr=0;

B run_finished=0;       //! Flags.
B pause_finished=0;
B pause_resume_finished=0;
B run=0;
B stop=0;
B pause=0;
B pause_resume=0;
B run_init=0;
B pause_init=0;
B pause_resume_init=0;
B motion_reverse=0;
B vm_interupt=0;

T position=0;        //! Overall absolute position.
T velocity=0;
T velocity_max=0;
T acceleration=0;
T progress=0;
T adaptive_feed=0;
T stored_velocity=0;
T stored_velocity_max=0;
T stored_position=0;
T stored_acceleration=0;
T stored_ace=0;
T stored_ve=0;
T stored_startpos=0;
T stored_endpos=0;
T timer_run=0;
T timer_apaptive_feed=0;
T servo_cycle=0.01;

T displacement_run=0;
T displacement_run_old=0;

T millisecond=0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(15,15);
    myOpenGl->setInterval(servo_cycle);
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

    auto start_clock = std::chrono::high_resolution_clock::now();

    if(pause){
        if(!pause_init){
            stored_velocity_max=velocity_max;
            velocity_max=0;
            vm_interupt=1;
            pause_init=1;
        }
    }

    if(pause_resume){
        pause=false;
        if(!pause_resume_init){
            velocity_max=stored_velocity_max;
            vm_interupt=1;
            pause_resume_init=1;
        }
        pause_resume=0;
    }

    if(vm_interupt){

        timer_apaptive_feed=0;
        T dtg=engine->netto_difference_of_2_values(stored_endpos,position);

        vmvec.clear();

        engine->process_curve({sc_engine::sc_period_id::id_run,
                               velocity,
                               stored_ve,
                               acceleration,
                               stored_ace,
                               dtg,
                               position,
                               stored_endpos}, velocity_max, vmvec);

        if(dtg>5){ //! Execute pause sequence away from waypoints.

            displacement_run=0;
            displacement_run_old=0;
            timer_run=0;

            runvec.clear();
            runvec=vmvec;
            vm_interupt=0;
        }
    }

    if(run && motionvec.size()>0){

        if(!run_init){
            motionvec_nr=-1;
            runvec.clear();
            run_finished=1;
            timer_run=0;
            timer_apaptive_feed=0;
            displacement_run_old=0;
            displacement_run=0;
            velocity=0;
            acceleration=0;
            run_init=1;
        }
        if(run_finished){

            motionvec_nr++;

            if(start_line_nr>motionvec_nr){ //! Set the start line nr.
                motionvec_nr=start_line_nr;
            }

            timer_apaptive_feed=0;

            if(motionvec_nr>motionvec.size()-1){
                motionvec_nr=motionvec.size()-1;
                stop=1;
            }

            motionvec.at(motionvec_nr).ncs=engine->netto_difference_of_2_values(motionvec.at(motionvec_nr).endpos,
                                                                                motionvec.at(motionvec_nr).startpos);

            sc_engine::sc_period p=motionvec.at(motionvec_nr);

            stored_ace=p.ace;
            stored_ve=p.ve;
            stored_startpos=p.startpos;
            stored_endpos=p.endpos;

            engine->process_curve(p,velocity_max,runvec);
            if(p.endpos<p.startpos){
                motion_reverse=1;
            } else {
                motion_reverse=0;
            }

            run_finished=0;
        }

        engine->interpolate_periods(timer_run,runvec,displacement_run,velocity,acceleration,run_finished);

        if(!run_finished){

            if(!motion_reverse){
                position+=displacement_run-displacement_run_old;
            } else {
                position-=displacement_run-displacement_run_old;
            }
            displacement_run_old=displacement_run;

            timer_run+=servo_cycle*adaptive_feed;
            timer_apaptive_feed+=servo_cycle*adaptive_feed;

            if(timer_apaptive_feed<0){
                timer_apaptive_feed=0;
                adaptive_feed=0;
            }

            progress=displacement_run/engine->to_stot_pvec(runvec);

            //! Todo interpolate overall, just like timer_run,
            // blockvec.at(motionvec_nr).interpolate(progress,xyz,abc,uvw);

            set_opengl(velocity,position,acceleration);
        }
    }

    if(stop){
        run=false;
        run_init=0;
        pause=0;
        pause_init=0;
        pause_resume=0;
        pause_resume_init=0;
    }

    auto stop_clock = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(stop_clock - start_clock);
    millisecond=ns.count()*ns_to_ms;

    gui_delay++;
    if(gui_delay>20){
        ui->label_planner_pos->setText(QString::number(position,'f',3));
        ui->label_planner_vel->setText(QString::number(velocity,'f',3));
        ui->label_planner_acc->setText(QString::number(acceleration,'f',3));

        ui->label_planner_block_nr->setText(QString::number(motionvec_nr,'f',3));
        ui->label_progress->setText(QString::number(progress,'f',3));

        ui->label_x->setText(QString::number(xyz.x,'f',3));
        ui->label_y->setText(QString::number(xyz.y,'f',3));
        ui->label_z->setText(QString::number(xyz.z,'f',3));
        ui->label_a->setText(QString::number(abc.a,'f',3));
        ui->label_b->setText(QString::number(abc.b,'f',3));
        ui->label_c->setText(QString::number(abc.c,'f',3));
        ui->label_u->setText(QString::number(uvw.u,'f',3));
        ui->label_v->setText(QString::number(uvw.v,'f',3));
        ui->label_w->setText(QString::number(uvw.w,'f',3));

        if(pause){
            ui->pushButton_pause->setStyleSheet(orange);
        } else {
            ui->pushButton_pause->setStyleSheet(original);
        }

        if(pause_resume){
            ui->pushButton_resume->setStyleSheet(green);
        } else {
            ui->pushButton_resume->setStyleSheet(original);
        }

        if(run){
            ui->pushButton_start->setStyleSheet(green);
        } else {
            ui->pushButton_start->setStyleSheet(original);
        }

        if(stop){
            ui->pushButton_stop->setStyleSheet(red);
        } else {
            ui->pushButton_stop->setStyleSheet(original);
        }

        if(motion_reverse){
            ui->checkBox_motion_reverse->setChecked(1);
        } else {
            ui->checkBox_motion_reverse->setChecked(0);
        }

        ui->label_performance->setText(QString::number(millisecond,'f',6));

        gui_delay=0;
    }
}

void MainWindow::on_pushButton_start_pressed()
{
    clear_opengl();
    motionvec.clear();
    T vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, start=0, end=0;

    //! Choose one:
    B example_0=1;
    B example_1=0;


    if(example_0){
        //! Load the waypoints, the ncs is mainly used in by the engine class, this for info.
        vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, start=0, end=100;
        motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct,start,end});

        vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, start=100, end=-200;
        motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct,start,end});
    }

    if(example_1){
        start=0;
        sc_interpolate::sc_block block;
        block.primitive_id=sc_interpolate::sc_primitive_id::id_line;
        block.set_pnt({0,0,0},{200,00,0});
        end=block.blocklenght();
        motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct,start,end});
        blockvec.push_back(block);
    }

    //! Execute.
    velocity_max=ui->doubleSpinBox_vm->value();
    adaptive_feed=ui->doubleSpinBox_adaptive_feed->value();
    start_line_nr=ui->spinBox_start_line->value();
    run=1;
    stop=0;
    run_init=0;
    // position=0; //! To avoid a position reset, out-comment this one.

    std::cerr<<"start."<<std::endl;
}

void MainWindow::on_pushButton_pause_pressed()
{
    pause=1;
    pause_init=0;
    std::cerr<<"pause."<<std::endl;
}

void MainWindow::on_pushButton_resume_pressed()
{
    pause_resume=1;
    pause_resume_init=0;
    std::cerr<<"pause resume."<<std::endl;
}


void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    arg1=0;
    vm_interupt=1;
    velocity_max=ui->doubleSpinBox_vm->value();
    std::cerr<<"vm interupt."<<std::endl;
}

void MainWindow::on_pushButton_stop_pressed()
{
    stop=true;
    std::cerr<<"stop."<<std::endl;
}

void MainWindow::on_doubleSpinBox_adaptive_feed_valueChanged(double arg1)
{
    adaptive_feed=arg1;
}

void MainWindow::on_pushButton_released()
{
    position=ui->lineEdit_set_position->text().toDouble();
}

void MainWindow::on_pushButton_test_pressed()
{

}
