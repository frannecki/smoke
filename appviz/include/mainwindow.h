#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include "ui_mainwindow.h"
#include <QCloseEvent>
#include <QSettings>
#include <QMessageBox>

#include "subnode.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char **argv, QWidget *parent = 0);
    ~MainWindow();
    void ReadSettings();
    void WriteSettings();
    void closeEvent(QCloseEvent *event);
    void showNoMasterMessage();

private slots:
    void on_actionAbout_triggered();
    void on_button_connect_clicked(bool check);
    void on_checkbox_use_environment_stateChanged(int state);
    void on_button_play_clicked();
    void updateLoggingView();
    void updateFrame();

private:
    // Qt components
    Ui::MainWindow *ui;
    bool showImage;
    QNode qnode;
    QPixmap pmap;
};

#endif // MAINWINDOW_H
