#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_optimize_pressed()
{
    std::vector<sc_block> blockvec;
    T velmax=9000/60; //! [mm/s]
    T gforcemax=0.01;
    T a=2;
    T dv=10;


    sc_block b;

    //! 2 colinear lines. Ve of first line should be velmax.
    b.set_pnt({0,0,0},{0,50,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    b.set_pnt({0,50,0},{0,90,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    //! 45 degrees chamfer. Ve should be ~ 0.5 * velmax.
    b.set_pnt({0,90,0},{10,100,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({10,100,0},{50,100,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    //! arc, starting and ending with a corner.
    b.set_pnt({50,100,0},{100,100,0},{100,50,0});
    b.primitive_id=sc_primitive_id::sc_arc;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({100,50,0},{100,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({100,0,0},{0,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    blockvec.push_back(b);

    blockvec=optimizer->sc_optimize_path(blockvec,
                                         velmax,
                                         gforcemax,
                                         a,
                                         dv);

    //! 1.Print calculated block angles.
    for(UI i=0; i<blockvec.size(); i++){
        std::cout<<"block nr:"<<i<<" transfer angle to next block:"<<blockvec.at(i).angle_end_deg<<std::endl;
    }

    //! 2.Print block vo's & ve's.
    for(UI i=0; i<blockvec.size(); i++){
        if(blockvec.at(i).primitive_id==sc_primitive_id::sc_line){
            std::cout<<"line"<<std::endl;
            std::cout<<"velmax:"<<blockvec.at(i).velmax<<std::endl;
        }
        if(blockvec.at(i).primitive_id==sc_primitive_id::sc_arc){
            std::cout<<"arc"<<std::endl;
            std::cout<<"given max gforce:"<<gforcemax<<std::endl;
            std::cout<<"velmax:"<<blockvec.at(i).velmax<<std::endl;
        }
        std::cout<<"block nr:"<<i<<" vo:"<<blockvec.at(i).vo<<std::endl;
        std::cout<<"block nr:"<<i<<" ve:"<<blockvec.at(i).ve<<std::endl;
    }
}















