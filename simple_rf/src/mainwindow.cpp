#include "mainwindow.h"
#include "./ui_main_win.h"

#include <cstdlib>
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->rcv_game_progress->setText("0");
    ui->rcv_remain_time->setText("350");
    ui->rcv_current_hp->setText("400");
    ui->rcv_projectile->setText("300");
    ui->rcv_red_outpost_hp->setText("1500");
    ui->rcv_red_base_hp->setText("5000");
    ui->rcv_blue_outpost_hp->setText("1500");
    ui->rcv_blue_base_hp->setText("5000");
    ui->rcv_be_attacked->setText("0");
    ui->rcv_out_war->setText("1");
    ui->rcv_low_energy->setText("0");
    ui->rcv_baolei_rfid->setText("0");
    ui->rcv_support1_rfid->setText("0");
    ui->rcv_support2_rfid->setText("0");

    this->timerId = startTimer(1000); 
    qnode = new QNode();
    ui->tab_manager->setCurrentIndex(0);
    connect(qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
}

void MainWindow::updateInfo() {
    bool ok;
    
    int game_progress = ui->rcv_game_progress->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Game Progress! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.game_progress = game_progress;

    if (game_progress == 4) {
        time--;
        ui->rcv_remain_time->setText(std::to_string(time).c_str());
        qnode->rcv_msg.remain_time = time;
        if (time == 0) {
            time = 351;
        }
    }

    int current_hp = ui->rcv_current_hp->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Current HP! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.current_hp = current_hp;

    int projectile = ui->rcv_projectile->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Projectile! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.projectile = projectile;

    int red_outpost_hp = ui->rcv_red_outpost_hp->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Red Outpost HP! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.red_outpost_hp = red_outpost_hp;

    int red_base_hp = ui->rcv_red_base_hp->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Red Base HP! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.red_base_hp = red_base_hp;

    int blue_outpost_hp = ui->rcv_blue_outpost_hp->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for Blue Outpost HP! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.blue_outpost_hp = blue_outpost_hp;

    int blue_base_hp = ui->rcv_blue_base_hp->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }
    qnode->rcv_msg.blue_base_hp = blue_base_hp;

    std::vector<uint8_t> bit_array(6);

    bit_array[0] = ui->rcv_be_attacked->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    bit_array[1] = ui->rcv_out_war->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    bit_array[5] = ui->rcv_low_energy->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    bit_array[2] = ui->rcv_baolei_rfid->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    bit_array[3] = ui->rcv_support1_rfid->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    bit_array[4] = ui->rcv_support2_rfid->text().toInt(&ok);
    if (!ok) {
        QMessageBox::warning(this, "Input Error", "Invalid input for blue Base HP! Please enter a valid number.");
        return;
    }

    int sentry_info=0;
    for (int i = 0; i < 6; ++i) {
        sentry_info |= (bit_array[i] & 0x01) << i;
    }

    qnode->rcv_msg.sentry_info = sentry_info;
    std::cout<<"detect_enemy="<<static_cast<int>(qnode->rcv_msg.sentry_info)<<std::endl;

    // QMessageBox::information(this, "Update Successful", "All values have been successfully updated.");
}

void MainWindow::timerEvent(QTimerEvent *e) {
    updateInfo();
}

MainWindow::~MainWindow() {
    delete ui;
}

