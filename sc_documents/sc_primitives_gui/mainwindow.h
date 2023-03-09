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

    void process();

    void on_spinBox_type_valueChanged(int arg1);

    void on_doubleSpinBox_a_valueChanged(double arg1);

    void on_doubleSpinBox_dv_valueChanged(double arg1);

    void on_doubleSpinBox_vo_valueChanged(double arg1);

    void on_doubleSpinBox_acs_valueChanged(double arg1);

    void on_doubleSpinBox_ace_valueChanged(double arg1);


private:
    Ui::MainWindow *ui;
    QTimer *timer;
    opengl *myOpenGl;

    sc_engine *engine=new sc_engine();
};
#endif
