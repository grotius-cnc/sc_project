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


void MainWindow::on_pushButton_optimize_arc_gforce_pressed()
{
    std::vector<sc_block> blockvec;
    sc_block b;
    T velmax=9000/60; //! [mm/s]
    T gforcemax=0.001;
    T a=2;
    T dv=10;

    std::cout<<"Intro:"<<std::endl;
    std::cout<<"velmax:"<<velmax<<std::endl;
    std::cout<<"gforcemax:"<<gforcemax<<std::endl;
    std::cout<<"a:"<<a<<std::endl;
    std::cout<<"dv:"<<dv<<std::endl;
    std::cout<<""<<std::endl;

    //! 2 colinear lines. Ve of first line should be velmax.
    b.set_pnt({0,0,0},{0,50,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    b.set_pnt({0,50,0},{0,90,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    //! 45 degrees chamfer. Ve should be ~ 0.5 * velmax.
    b.set_pnt({0,90,0},{10,100,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({10,100,0},{50,100,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    //! arc, starting and ending with a corner.
    b.set_pnt({50,100,0},{100,100,0},{100,50,0});
    b.primitive_id=sc_primitive_id::sc_arc;
    b.type=sc_type::sc_G2;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({100,50,0},{100,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    //! line.
    b.set_pnt({100,0,0},{0,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    optimizer->sc_set_a_dv_gforce_velmax(a,dv,gforcemax,velmax);

    //! The following function call's are path rules:
    blockvec=optimizer->sc_optimize_block_angles_ve(blockvec);
    blockvec=optimizer->sc_optimize_gforce_arcs(blockvec);
    blockvec=optimizer->sc_optimize_G0_ve(blockvec);
    blockvec=optimizer->sc_optimize_G123_ve_backward(blockvec);
    blockvec=optimizer->sc_optimize_G123_ve_forward(blockvec);

    optimizer->sc_print_blockvec(blockvec);
}

void MainWindow::on_pushButton_optimize_short_lines_pressed()
{
    std::vector<sc_block> blockvec;
    sc_block b;
    T velmax=10; //! [mm/s]
    T gforcemax=0.01;
    T a=2;
    T dv=10;

    std::cout<<"Intro:"<<std::endl;
    std::cout<<"velmax:"<<velmax<<std::endl;
    std::cout<<"gforcemax:"<<gforcemax<<std::endl;
    std::cout<<"a:"<<a<<std::endl;
    std::cout<<"dv:"<<dv<<std::endl;
    std::cout<<""<<std::endl;

    b.set_pnt({0,0,0},{10,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    b.set_pnt({10,0,0},{11,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    b.set_pnt({11,0,0},{100,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    b.set_pnt({100,0,0},{1000,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    b.set_pnt({1000,0,0},{1001,0,0});
    b.primitive_id=sc_primitive_id::sc_line;
    b.type=sc_type::sc_G1;
    blockvec.push_back(b);

    optimizer->sc_set_a_dv_gforce_velmax(a,dv,gforcemax,velmax);

    //! The following function call's are path rules:

    blockvec=optimizer->sc_optimize_block_angles_ve(blockvec);
    blockvec=optimizer->sc_optimize_gforce_arcs(blockvec);
    blockvec=optimizer->sc_optimize_G0_ve(blockvec);
    blockvec=optimizer->sc_optimize_G123_ve_backward(blockvec);
    blockvec=optimizer->sc_optimize_G123_ve_forward(blockvec);

    optimizer->sc_print_blockvec(blockvec);

}

void MainWindow::on_pushButton_calculate_gforce_pressed()
{
    T gforce=0;
    sc_optimizer().sc_get_gforce(ui->doubleSpinBox_velocity->value()/60,
                                 ui->doubleSpinBox_diameter->value()/2,
                                 gforce);
    ui->label_gforce->setText(QString::number(gforce,'f',3));
}




















