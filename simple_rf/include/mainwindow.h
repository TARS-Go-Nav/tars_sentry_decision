#pragma once

#include <QMainWindow>
#include <iostream>
#include "qnode.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void updateInfo();
    virtual void timerEvent(QTimerEvent *event);

private:
    Ui::MainWindow *ui;
    QNode *qnode;
    int timerId;
    int time = 301;
};
