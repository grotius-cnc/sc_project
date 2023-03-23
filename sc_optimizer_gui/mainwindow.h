#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

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

    void on_pushButton_optimize_short_lines_pressed();

    void on_pushButton_optimize_arc_gforce_pressed();

    void on_pushButton_calculate_gforce_pressed();

private:
    Ui::MainWindow *ui;

    sc_optimizer *optimizer=new sc_optimizer();
};
#endif
