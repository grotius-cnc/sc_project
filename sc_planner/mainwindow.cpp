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
T acceleration=0;
T stored_velocity=0;
T stored_position=0;
T stored_acceleration=0;
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
        engine->process_curve({sc_engine::sc_period_id::id_run, //! Id run works ok for interupts.
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

                std::cout<<"run finished:"<<std::endl;

                sc_engine::sc_period p=motionvec.at(motionvec_nr);
                p.ncs=engine->netto_difference_of_2_values(p.endpos,p.startpos);

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
            timer_run+=0.01;
            set_opengl(velocity,position,acceleration);
        }
    }

    if(stop){
        run=0;
        run_init=0;
        pause=0;
        pause_init=0;
        pause_resume=0;
        pause_resume_init=0;
    }

    if(pause){

        if(!pause_init){
            pausevec.clear();
            pause_finished=0;

            stored_velocity=velocity;
            stored_position=position;
            stored_acceleration=acceleration;

            T ve=0, ace=0;
            engine->process_curve({sc_engine::sc_period_id::id_pause, //! Id_pause produces a short curve without processing vm.
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

        if(!pause_finished){
            if(!motion_reverse){
                position+=displacement_pause-displacement_pause_old;
            } else {
                position-=displacement_pause-displacement_pause_old;
            }

            displacement_pause_old=displacement_pause;
            timer_pause+=0.01;

            set_opengl(velocity,position,acceleration);
        }
    }

    if(pause_resume){

        // Set up motion resume. Find current position in the motionvec nr's.
        UI restart_nr=0;
        UI reverse=0;

        for(uint i=0; i<motionvec.size(); i++){ //! Find current position in the motionvec.

            //! The position can be everywhere. Find another solution tomorrow to find out in
            //! wich motionvec nr we are after machine is stopped in pause.

           std::cout<<"starpos: "<<motionvec.at(i).startpos<<std::endl;
           std::cout<<"endpos: "<<motionvec.at(i).endpos<<std::endl;

           if(motionvec.at(i).startpos<motionvec.at(i).endpos){
               std::cout<<"mption fwd"<<std::endl;
               reverse=0;
           } else {
               std::cout<<"mption reverse"<<std::endl;
               reverse=1;
           }

           if(engine->is_inbetween_2_values(motionvec.at(i).startpos,motionvec.at(i).endpos,position)){
               std::cout<<"position is at i:"<<i<<std::endl;
           }

//            if(!reverse &&motionvec.at(i).endpos>position){
//                restart_nr=i;

//                std::cout<<"fwd found .. position:"<<position<<std::endl;
//                std::cout<<"fwd found .. restart nr:"<<i<<std::endl;
//                break;
//            }

//            if(reverse &&motionvec.at(i).endpos<position){
//                restart_nr=i;

//                std::cout<<"rev found .. position:"<<position<<std::endl;
//                std::cout<<"rev found .. restart nr:"<<i<<std::endl;
//                break;
//            }

       }

        std::cout<<"reverse flag:"<<reverse<<std::endl;
        std::cout<<"restart nr:"<<restart_nr<<std::endl;
        std::cout<<"cur motionvec nr:"<<motionvec_nr<<std::endl;

        T dtg=engine->netto_difference_of_2_values(motionvec.at(restart_nr).endpos,position);

        std::cout<<"dtg:"<<dtg<<std::endl;

        //! Edit the current run motionblock to finish current motion.
        T vo=0, ve=motionvec.at(restart_nr).ve, acs=0, ace=motionvec.at(restart_nr).ace;
        runvec.clear();
        engine->process_curve({sc_engine::sc_period_id::id_run, //! Id_pause produces a short curve without processing vm.
                               vo,
                               ve,
                               acs,
                               ace,
                               dtg}, ui->doubleSpinBox_vm->value(), runvec);

        //! Reset some run values.
        run_init=1;
        run_finished=0;
        timer_run=0;
        displacement_run_old=0;

        if(motionvec.at(restart_nr).endpos<motionvec.at(restart_nr).startpos){
            motion_reverse=true;
        } else {
            motion_reverse=false;
        }

        motionvec_nr=restart_nr+1;

        //! Go, do your best.
        pause_resume=0;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    millisecond=ns.count()*ns_to_ms;

    gui_delay++;
    if(gui_delay>20){
        ui->label_planner_pos->setText(QString::number(position,'f',3));
        ui->label_planner_vel->setText(QString::number(velocity,'f',3));
        ui->label_planner_acc->setText(QString::number(acceleration,'f',3));

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

    T vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=0;


    motionvec.clear();

    //! Load the waypoints, the ncs is mainly used in by the engine class.
    vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=0, endpos=100;
    motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    vo=0, ve=0, acs=0, ace=0, ncs=0, nct=0, startpos=100, endpos=-200;
    motionvec.push_back({sc_engine::sc_period_id::id_run,vo,ve,acs,ace,ncs,nct,startpos,endpos});

    //! Execute.
    run=1;
    stop=0;
    run_init=0;
    pause=0;
    pause_init=0;
    pause_resume=0;
    pause_resume_init=0;

    position=0; //! To avoid a position reset, out-comment this one.

    std::cerr<<"start."<<std::endl;
}

void MainWindow::on_pushButton_pause_pressed()
{
    if(pause==1){
        return;
    }
    pause=1;
    pause_init=0;
    pause_resume=0;

    std::cerr<<"pause."<<std::endl;
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

    std::cerr<<"pause resume."<<std::endl;
}


void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    arg1=0;
    vm_interupt=1;
    std::cerr<<"vm interupt."<<std::endl;
}

void MainWindow::on_pushButton_stop_pressed()
{
    stop=true;
    std::cerr<<"stop."<<std::endl;
}






















