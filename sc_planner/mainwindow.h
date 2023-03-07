#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <iostream>
#include <chrono>
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

    void on_pushButton_start_pressed();

    void on_pushButton_pause_pressed();

    void on_pushButton_resume_pressed();

    void on_doubleSpinBox_vm_valueChanged(double arg1);

    void clear_opengl();

    void set_opengl(T vel, T pos, T acc);

    void on_pushButton_stop_pressed();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    opengl *myOpenGl;

    sc_engine *engine = new sc_engine();
};
#endif
