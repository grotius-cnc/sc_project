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

}

void MainWindow::process(){

    std::vector<double> vvec, svec, avec;
    T vo=0, acs=0, ace=0, v=0, s=0, a=0, dv=0, vi=0, si=0, ai=0;

    a=ui->doubleSpinBox_a->value();
    dv=ui->doubleSpinBox_dv->value();

    engine->sc_set_a_dv(a,dv);

    vo=ui->doubleSpinBox_vo->value();
    acs=ui->doubleSpinBox_acs->value();
    ace=ui->doubleSpinBox_ace->value();

    sc_engine::sc_period p;

    if(ui->spinBox_type->value()==1){
        engine->t1(vo,acs,ace,p);
    }
    if(ui->spinBox_type->value()==2){

    }
    if(ui->spinBox_type->value()==3){
        engine->t3(vo,acs,ace,p);
    }
    if(ui->spinBox_type->value()==4){

    }
    if(ui->spinBox_type->value()==5){
        engine->t5(vo,acs,ace,p);
    }
    if(ui->spinBox_type->value()==6){

    }
    if(ui->spinBox_type->value()==7){
        engine->t7(vo,acs,ace,p);
    }



    for(double i=0; i<p.nct; i+=0.01){
        engine->interpolate_period(i,p,si,vi,ai);
        vvec.push_back(vi);
        svec.push_back(si);
        avec.push_back(ai);
    }

    myOpenGl->setj0vec(vvec);
    myOpenGl->setj1vec(svec);
    myOpenGl->setj2vec(avec);


    engine->interpolate_period(p.nct,p,s,v,a);

    ui->label_ve->setText(QString::number(v,'f',3));
    ui->label_s->setText(QString::number(s,'f',3));
    ui->label_t->setText(QString::number(s,'t',3));
}

void MainWindow::on_spinBox_type_valueChanged(int arg1)
{
    process();
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

void MainWindow::on_doubleSpinBox_acs_valueChanged(double arg1)
{
    process();
}

void MainWindow::on_doubleSpinBox_ace_valueChanged(double arg1)
{
    process();
}


