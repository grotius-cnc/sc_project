#include "mainwindow.h"
#include "./ui_mainwindow.h"

//! If user reqeust a pause, motion will do a controlled stop. For now test on the steady stage.
//! A pause resume will go back to pause interupt position, then will continue path.

//! Gui items.
std::vector<double> vvec,svec,avec;
int gui_delay=0; //! To keep a good gui performance using opengl.

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(15,15);
    myOpenGl->setInterval(0.01);
    myOpenGl->set2VecShift(100);
    myOpenGl->set1VecScale(0.1);

    //! Timer to simulate servo cycle.
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);

    planner->sc_set_a_dv(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value());
    planner->sc_set_servo_cycle_time(0.01);
    planner->sc_set_vm(ui->doubleSpinBox_vm->value());
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

    planner->sc_update();

    T position=0, velocity=0, acceleration=0, line_progress=0, traject_progress=0;
    UI line_nr=0;
    B finished=0;

    planner->sc_get_planner_results(position,
                                    velocity,
                                    acceleration,
                                    line_nr,
                                    line_progress,
                                    traject_progress,
                                    finished);

    set_opengl(velocity,position,acceleration);


    gui_delay++;
    if(gui_delay>20){



        ui->label_planner_pos->setText(QString::number(position,'f',3));
        ui->label_planner_vel->setText(QString::number(velocity,'f',3));
        ui->label_planner_acc->setText(QString::number(acceleration,'f',3));
        ui->label_line_nr->setText(QString::number(line_nr,'f',1));
        ui->label_line_nr_progress->setText(QString::number(line_progress,'f',3));
        ui->label_traject_progress->setText(QString::number(traject_progress,'f',3));
        ui->label_finished->setText(QString::number(finished,'f',1));

        sc_pnt xyz;
        sc_dir abc;
        sc_ext uvw;
        T interpolation_progress=0;
        planner->sc_get_interpolation_results(xyz,abc,uvw,interpolation_progress);

        ui->label_x->setText(QString::number(xyz.x,'f',3));
        ui->label_y->setText(QString::number(xyz.y,'f',3));
        ui->label_z->setText(QString::number(xyz.z,'f',3));
        ui->label_a->setText(QString::number(abc.a,'f',3));
        ui->label_b->setText(QString::number(abc.b,'f',3));
        ui->label_c->setText(QString::number(abc.c,'f',3));
        ui->label_u->setText(QString::number(uvw.u,'f',3));
        ui->label_v->setText(QString::number(uvw.v,'f',3));
        ui->label_w->setText(QString::number(uvw.w,'f',3));
        ui->label_interpolation_progress->setText( QString::number(interpolation_progress,'f',2));

        enum sc_planner::sc_enum_program_status state;
        planner->sc_get_program_state(state);

        if(state==sc_planner::sc_enum_program_status::program_pause){
            ui->pushButton_run->setStyleSheet(original);
            ui->pushButton_pause->setStyleSheet(green);
            resume_orange=1;
            ui->pushButton_stop->setStyleSheet(original);
        }

        if(state==sc_planner::sc_enum_program_status::program_pause_resume){
            ui->pushButton_run->setStyleSheet(original);
            ui->pushButton_pause->setStyleSheet(original);
            ui->pushButton_resume->setStyleSheet(green);
            ui->pushButton_stop->setStyleSheet(original);
        }

        if(state==sc_planner::sc_enum_program_status::program_run){
            ui->pushButton_run->setStyleSheet(green);
            ui->pushButton_pause->setStyleSheet(original);
            resume_orange=0;
            ui->pushButton_stop->setStyleSheet(original);
        }

        if(state==sc_planner::sc_enum_program_status::program_stop ||
                state==sc_planner::sc_enum_program_status::program_end ){
            ui->pushButton_run->setStyleSheet(original);
            ui->pushButton_pause->setStyleSheet(original);
            resume_orange=0;
            ui->pushButton_stop->setStyleSheet(red);
        }

        blink_delay++;
        if( resume_orange && blink_delay>10){
            ui->pushButton_resume->setStyleSheet(orange);
        }
        if(resume_orange && blink_delay>20){
            ui->pushButton_resume->setStyleSheet(original);
            blink_delay=0;
        }
        if(!resume_orange){
            ui->pushButton_resume->setStyleSheet(original);
        }


        ui->label_program_status_string->setText(QString::fromStdString(planner->sc_get_program_state()));

        ui->label_performance->setText(QString::number(planner->sc_performance(),'f',6));

        gui_delay=0;
    }
}

void MainWindow::on_pushButton_run_pressed()
{
    if(planner->sc_get_stop_cycle_state()){

        clear_opengl();

        //! Add some motions.
        T vo=0, ve=0, acs=0, ace=0;

        planner->sc_clear();
        vo=0, ve=5;
        planner->sc_add_line_motion(vo,ve,acs,ace,{0,0,0},{100,0,0});
        vo=ve; ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{100,0,0},{200,0,0});
        vo=ve; ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{200,0,0},{250,250,250});

        planner->sc_set_startline(ui->spinBox_start_line->value());
        planner->sc_set_state(sc_planner::sc_enum_program_status::program_run);

    }
}

void MainWindow::on_pushButton_pause_pressed()
{   
    planner->sc_set_state(sc_planner::sc_enum_program_status::program_pause);
}

void MainWindow::on_pushButton_resume_pressed()
{
    planner->sc_set_state(sc_planner::sc_enum_program_status::program_pause_resume);
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    planner->sc_set_vm(arg1);
    planner->sc_set_state(sc_planner::sc_enum_program_status::program_vm_interupt);
}

void MainWindow::on_pushButton_stop_pressed()
{
    planner->sc_set_state(sc_planner::sc_enum_program_status::program_stop);
}

void MainWindow::on_doubleSpinBox_adaptive_feed_valueChanged(double arg1)
{
    planner->sc_set_adaptive_feed(arg1);
}

void MainWindow::on_pushButton_pressed()
{
    planner->sc_set_position(ui->lineEdit_set_position->text().toDouble());
}









