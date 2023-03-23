#include "mainwindow.h"
#include "./ui_mainwindow.h"

T interv=0.001;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //! OpenGl output to verify.
    myOpenGl = new opengl();
    //! Graph scale.
    myOpenGl->setScale(25,25);
    myOpenGl->setInterval(interv);
    myOpenGl->set2VecShift(100);
    myOpenGl->set1VecScale(0.1);

    //! Timer to simulate servo cycle.
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(thread()));
    timer->start(1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

std::vector<T> vvec, svec, avec;
bool run_pid=0;
uint delay=0;
T velocity=0, position=0, acceleration=0;


//! This function simulates the servo cycle. And is called every 1 millisecond.
void MainWindow::thread(){


    if(run_pid){

        B finished=pid->sc_update();

        if(!finished){

            pid->sc_result(position,velocity,acceleration);

            vvec.push_back(velocity);
            svec.push_back(position);
            avec.push_back(acceleration);

            myOpenGl->setj0vec(vvec);
            myOpenGl->setj1vec(svec);
            myOpenGl->setj2vec(avec);
        }
    }

    delay++;
    if(delay>20){ //! Gui update delay.
        ui->label_curpos->setText(QString::number(position,'f',3));
        ui->label_curvel->setText(QString::number(velocity,'f',3));
        ui->label_curacc->setText(QString::number(acceleration,'f',3));
        delay=0;
    }
}

void MainWindow::on_pushButton_run_pressed()
{
    pid->sc_set_a_dv_interval(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value(),interv);
    pid->sc_set_maxvel(ui->doubleSpinBox_vm->value());
    pid->sc_set_tarpos(ui->doubleSpinBox_tarpos->value());
    run_pid=1;
}

void MainWindow::on_pushButton_stop_pressed()
{
    run_pid=0;
    vvec.clear();
    svec.clear();
    avec.clear();
    myOpenGl->setj0vec(vvec);
    myOpenGl->setj1vec(svec);
    myOpenGl->setj2vec(avec);
}

void MainWindow::on_doubleSpinBox_tarpos_valueChanged(double arg1)
{
    pid->sc_set_tarpos(arg1);
}

void MainWindow::on_doubleSpinBox_vm_valueChanged(double arg1)
{
    pid->sc_set_maxvel(arg1);
}

void MainWindow::on_doubleSpinBox_dv_valueChanged(double arg1)
{
    pid->sc_set_a_dv_interval(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value(),interv);
}

void MainWindow::on_doubleSpinBox_a_valueChanged(double arg1)
{
    pid->sc_set_a_dv_interval(ui->doubleSpinBox_a->value(),ui->doubleSpinBox_dv->value(),interv);
}
