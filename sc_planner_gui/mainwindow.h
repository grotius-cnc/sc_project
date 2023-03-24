#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <chrono>
#include <opengl.h>
#include <../sc_planner/sc_planner.h>
#include <../sc_optimizer/sc_optimizer.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void thread();

    void on_pushButton_run_pressed();

    void on_pushButton_pause_pressed();

    void on_pushButton_resume_pressed();

    void on_doubleSpinBox_vm_valueChanged(double arg1);

    void clear_opengl();

    void set_opengl(T vel, T pos, T acc);

    void on_pushButton_stop_pressed();

    void on_doubleSpinBox_adaptive_feed_valueChanged(double arg1);

    void on_pushButton_pressed();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    opengl *myOpenGl;
    B resume_orange=0; UI blink_delay=0;

    sc_planner *planner=new sc_planner();
    sc_pnt xyz;
    sc_dir abc;
    sc_ext uvw;
    T interpolation_progress=0;

    QString original="background-color: rgb(51, 57, 59);\ncolor: rgb(255, 255, 255);\n";
    QString orange="background-color: rgb(170, 85, 0);\ncolor: rgb(255, 255, 255);\n";
    QString green="background-color: rgb(85, 170, 0);\ncolor: rgb(255, 255, 255);\n";
    QString red="background-color: rgb(255, 0, 0);\ncolor: rgb(255, 255, 255);\n";
};
#endif
