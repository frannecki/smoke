#include "mainwindow.h"

MainWindow::MainWindow(int argc, char **argv, QWidget *parent):
    qnode(argc, argv), 
    QMainWindow(parent), 
    ui(new Ui::MainWindow), 
    showImage(false)
{
    // Qt related
    ui->setupUi(this);
    setWindowIcon(QIcon(":/images/icon.png"));
	ui->tab_manager->setCurrentIndex(0);
    connect(ui->actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
    connect(ui->play_button, SIGNAL(clicked()), this, SLOT(on_playbutton_clicked()));
    connect(ui->button_connect, SIGNAL(clicked()), this, SLOT(on_button_connect_clicked(bool)));
    connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));
    connect(&qnode, SIGNAL(imgUpdated()), this, SLOT(updateFrame()));
    ui->view_logging->setModel(qnode.loggingModel());
    QImage qimg(":/images/blank.jpg");
    pmap = QPixmap::fromImage(qimg);
    ui->view_image->setPixmap(pmap);

    if ( ui->checkbox_remember_settings->isChecked() ) {
        ReadSettings();
    }
}

MainWindow::~MainWindow(){
    delete ui;
}

void MainWindow::on_playbutton_clicked(){
    showImage = !showImage;
    if(showImage)  ui->play_button->setText("Stop");
    else{
        ui->play_button->setText("Play");
        ui->view_image->setPixmap(pmap);
    }
}

void MainWindow::updateLoggingView(){
    ui->view_logging->scrollToBottom();
}

void MainWindow::updateFrame(){
    if(!showImage)  return;
    ui->view_image->setPixmap(*(qnode.pixMap()));
}

void MainWindow::on_button_connect_clicked(bool check ) {
    if ( ui->checkbox_use_environment->isChecked() ) {
    	if ( !qnode.init() ) {
    	    showNoMasterMessage();
    	}
        else {
            ui->button_connect->setEnabled(false);
    	}
    } 
    else {
    	if ( ! qnode.init(ui->line_edit_master->text().toStdString(),
    			   ui->line_edit_host->text().toStdString()) ) 
        {
    	    showNoMasterMessage();
    	} 
        else {
    	    ui->button_connect->setEnabled(false);
    	    ui->line_edit_master->setReadOnly(true);
    	    ui->line_edit_host->setReadOnly(true);
    	    ui->line_edit_topic->setReadOnly(true);
    	}
    }
}

void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
    bool enabled;
    if ( state == 0 ) {
    	enabled = true;
    } 
    else {
    	enabled = false;
    }
    ui->line_edit_master->setEnabled(enabled);
    ui->line_edit_host->setEnabled(enabled);
}

void MainWindow::showNoMasterMessage() {
    QMessageBox msgBox;
    msgBox.setText("Couldn't find the ros master.");
    msgBox.exec();
    close();
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Frannecki</p><p>This package needs an about description.</p>"));
}

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qtapp");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://localhost:11311/")).toString();
    QString host_url = settings.value("host_url", QString("localhost")).toString();
    ui->line_edit_master->setText(master_url);
    ui->line_edit_host->setText(host_url);
    bool remember = settings.value("remember_settings", false).toBool();
    ui->checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui->checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui->line_edit_master->setEnabled(false);
    	ui->line_edit_host->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qtapp");
    settings.setValue("master_url",ui->line_edit_master->text());
    settings.setValue("host_url",ui->line_edit_host->text());
    settings.setValue("use_environment_variables",QVariant(ui->checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui->checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
    QMainWindow::closeEvent(event);
}