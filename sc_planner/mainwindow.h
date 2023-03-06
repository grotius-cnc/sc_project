#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <opengl.h>
#include <../sc_engine/sc_engine.h>

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

    void on_pushButton_planner_pressed();

    void on_pushButton_planner_pause_pressed();

    void on_pushButton_resume_pressed();

    void on_doubleSpinBox_vm_valueChanged(double arg1);

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    opengl *myOpenGl;

    sc_engine *engine = new sc_engine();
};
#endif
