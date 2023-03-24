#include "mainwindow.h"
#include "./ui_mainwindow.h"

//! Gui items.
std::vector<double> vvec,svec,avec,pid_vec;
//! To keep a good gui performance using opengl.
int gui_delay=0;
int pid_delay=0;
UI optimizer_trigger=0;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(15,15);
    myOpenGl->setInterval(0.001);
    myOpenGl->set2VecShift(100);
    myOpenGl->set1VecScale(0.1);

    //! Timer to simulate servo cycle.
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);

    planner->sc_set_a_dv(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value());
    planner->sc_set_interval(0.001);
    planner->sc_set_maxvel(ui->doubleSpinBox_vm->value());
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

    if(line_progress>0 && line_progress<0.5 && !optimizer_trigger){
        planner->sc_optimize(line_nr,line_nr+20);
        // std::cout<<"optimizing ahead, from line:"<<line_nr<<" to line nr:"<<line_nr+20<<std::endl;
        optimizer_trigger=1;
    }
    if(line_progress>0.5){
        optimizer_trigger=0;
    }

    planner->sc_get_interpolation_results(xyz,abc,uvw,interpolation_progress);

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
        vo=0, ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{0,0,0},{10,0,0},sc_type::sc_G1);

        vo=0, ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{10,0,0},{20,0,0},sc_type::sc_G1);

        vo=0, ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{20,0,0},{100,0,0},sc_type::sc_G1);

        vo=0, ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{100,0,0},{110,0,0},sc_type::sc_G1);

        vo=0, ve=0;
        planner->sc_add_line_motion(vo,ve,acs,ace,{110,0,0},{200,50,0},sc_type::sc_G1);

        //! Optimize gcode's first 20 lines at program start.
        planner->sc_optimize(0,20);
        //! Or optimize full gcode.
        // planner->sc_optimize(0,Infinity);

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
    planner->sc_set_maxvel(arg1);
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



