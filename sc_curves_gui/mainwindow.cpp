#include "mainwindow.h"
#include "./ui_mainwindow.h"

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

    process(); //! Initial process.
}

MainWindow::~MainWindow()
{
    delete ui;
}

//! This function simulates the servo cycle. And is called every 1 millisecond.
void MainWindow::thread(){
    //process();
}

void MainWindow::process(){

    std::vector<double> vvec, svec, avec;
    T vo=0, vm=0, ve=0, acs=0, ace=0, v=0, s=0, a=0, dv=0, vi=0, si=0, ai=0, dummy=0, nct=0;

    a=ui->doubleSpinBox_a->value();
    dv=ui->doubleSpinBox_dv->value();

    engine->sc_set_a_dv(a,dv);

    vo=ui->doubleSpinBox_vo->value();
    vm=ui->doubleSpinBox_vm->value();
    ve=ui->doubleSpinBox_ve->value();
    acs=ui->doubleSpinBox_acs->value();
    ace=ui->doubleSpinBox_ace->value();
    s=ui->doubleSpinBox_s->value();

    sc_engine::sc_period p;
    p={sc_engine::sc_period_id::id_pvec,vo,ve,acs,ace,s};

    std::vector<sc_engine::sc_period> pvec;

    auto start = std::chrono::high_resolution_clock::now();

   // engine->t3_t5_t6_t7_t1(p,pvec);

    engine->process_curve(p,vm,pvec);

    auto end = std::chrono::high_resolution_clock::now();
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start);
    std::cout<<"ms:"<<ns.count()*ns_to_ms<<std::endl;

    T sf=0, st=0;;
    for(uint i=0; i<pvec.size(); i++){

        for(T j=0; j<pvec.at(i).nct; j+=0.01){
            engine->interpolate_period(j,pvec.at(i),si,vi,ai);
            vvec.push_back(vi);
            st=si+sf;
            svec.push_back(si+sf);
            avec.push_back(ai);

            //if(st>s){
            //    break;
            //}
        }
        sf+=pvec.at(i).ncs;

        //if(st>s){
        //    break;
        //}
    }



    //    engine->t2_s(vo,ve,a,p);

    //    for(T i=0; i<p.nct; i+=0.01){
    //            engine->interpolate_period(i,p,si,vi,ai);
    //            vvec.push_back(vi);
    //            svec.push_back(si);
    //            avec.push_back(ai);
    //    }

    // engine->process_scurve(p,vm,pvec);

    myOpenGl->setj0vec(vvec);
    myOpenGl->setj1vec(svec);
    myOpenGl->setj2vec(avec);
}

void MainWindow::on_doubleSpinBox_a_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_dv_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_vo_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_ve_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_acs_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_ace_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_s_valueChanged(double arg1)
{
    process();
}

