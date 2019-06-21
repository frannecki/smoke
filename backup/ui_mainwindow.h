/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_alarm;
    QGridLayout *gridLayout_3;
    QListView *view_logging;
    QWidget *tab;
    QHBoxLayout *horizontalLayout;
    QGroupBox *groupBox_image;
    QVBoxLayout *verticalLayout_4;
    QLabel *view_image;
    QSpacerItem *verticalSpacer_image;
    QPushButton *button_play;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;
    QDockWidget *dock_status;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *verticalLayout;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
    QLabel *label;
    QLineEdit *line_edit_master;
    QLabel *label_2;
    QLineEdit *line_edit_host;
    QLabel *label_3;
    QLineEdit *line_edit_topic;
    QCheckBox *checkbox_use_environment;
    QCheckBox *checkbox_remember_settings;
    QSpacerItem *horizontalSpacer;
    QPushButton *button_connect;
    QSpacerItem *verticalSpacer_3;
    QPushButton *quit_button;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1273, 886);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        MainWindow->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        action_Quit = new QAction(MainWindow);
        action_Quit->setObjectName(QStringLiteral("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindow);
        action_Preferences->setObjectName(QStringLiteral("action_Preferences"));
        actionAbout = new QAction(MainWindow);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout_Qt = new QAction(MainWindow);
        actionAbout_Qt->setObjectName(QStringLiteral("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::UnitedStates));
        tab_status = new QWidget();
        tab_status->setObjectName(QStringLiteral("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        groupBox_alarm = new QGroupBox(tab_status);
        groupBox_alarm->setObjectName(QStringLiteral("groupBox_alarm"));
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_alarm->sizePolicy().hasHeightForWidth());
        groupBox_alarm->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(groupBox_alarm);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        view_logging = new QListView(groupBox_alarm);
        view_logging->setObjectName(QStringLiteral("view_logging"));

        gridLayout_3->addWidget(view_logging, 0, 0, 1, 1);


        verticalLayout_2->addWidget(groupBox_alarm);

        tab_manager->addTab(tab_status, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        horizontalLayout = new QHBoxLayout(tab);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        groupBox_image = new QGroupBox(tab);
        groupBox_image->setObjectName(QStringLiteral("groupBox_image"));
        verticalLayout_4 = new QVBoxLayout(groupBox_image);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        view_image = new QLabel(groupBox_image);
        view_image->setObjectName(QStringLiteral("view_image"));

        verticalLayout_4->addWidget(view_image);

        verticalSpacer_image = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_4->addItem(verticalSpacer_image);

        button_play = new QPushButton(groupBox_image);
        button_play->setObjectName(QStringLiteral("button_play"));

        verticalLayout_4->addWidget(button_play);


        horizontalLayout->addWidget(groupBox_image);

        tab_manager->addTab(tab, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1273, 36));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);
        dock_status = new QDockWidget(MainWindow);
        dock_status->setObjectName(QStringLiteral("dock_status"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(dock_status->sizePolicy().hasHeightForWidth());
        dock_status->setSizePolicy(sizePolicy1);
        dock_status->setMinimumSize(QSize(480, 584));
        dock_status->setAllowedAreas(Qt::RightDockWidgetArea);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QStringLiteral("dockWidgetContents_2"));
        verticalLayout = new QVBoxLayout(dockWidgetContents_2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        frame = new QFrame(dockWidgetContents_2);
        frame->setObjectName(QStringLiteral("frame"));
        sizePolicy1.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy1);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setFrameShape(QFrame::StyledPanel);
        label->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label, 0, 0, 1, 1);

        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QStringLiteral("line_edit_master"));

        gridLayout->addWidget(line_edit_master, 1, 0, 1, 2);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setFrameShape(QFrame::StyledPanel);
        label_2->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label_2, 2, 0, 1, 1);

        line_edit_host = new QLineEdit(groupBox);
        line_edit_host->setObjectName(QStringLiteral("line_edit_host"));

        gridLayout->addWidget(line_edit_host, 3, 0, 1, 2);

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setFrameShape(QFrame::StyledPanel);
        label_3->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label_3, 4, 0, 1, 1);

        line_edit_topic = new QLineEdit(groupBox);
        line_edit_topic->setObjectName(QStringLiteral("line_edit_topic"));
        line_edit_topic->setEnabled(false);

        gridLayout->addWidget(line_edit_topic, 5, 0, 1, 2);

        checkbox_use_environment = new QCheckBox(groupBox);
        checkbox_use_environment->setObjectName(QStringLiteral("checkbox_use_environment"));
        checkbox_use_environment->setLayoutDirection(Qt::RightToLeft);

        gridLayout->addWidget(checkbox_use_environment, 6, 0, 1, 2);

        checkbox_remember_settings = new QCheckBox(groupBox);
        checkbox_remember_settings->setObjectName(QStringLiteral("checkbox_remember_settings"));
        checkbox_remember_settings->setLayoutDirection(Qt::RightToLeft);

        gridLayout->addWidget(checkbox_remember_settings, 7, 0, 1, 2);

        horizontalSpacer = new QSpacerItem(170, 21, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 8, 0, 1, 1);

        button_connect = new QPushButton(groupBox);
        button_connect->setObjectName(QStringLiteral("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy2);

        gridLayout->addWidget(button_connect, 8, 1, 1, 1);


        verticalLayout_3->addWidget(groupBox);

        verticalSpacer_3 = new QSpacerItem(20, 233, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_3);


        verticalLayout->addWidget(frame);

        quit_button = new QPushButton(dockWidgetContents_2);
        quit_button->setObjectName(QStringLiteral("quit_button"));
        sizePolicy2.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy2);

        verticalLayout->addWidget(quit_button);

        dock_status->setWidget(dockWidgetContents_2);
        MainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dock_status);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindow);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindow, SLOT(close()));

        tab_manager->setCurrentIndex(1);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Appviz", 0));
        action_Quit->setText(QApplication::translate("MainWindow", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindow", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindow", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindow", "About &Qt", 0));
        groupBox_alarm->setTitle(QApplication::translate("MainWindow", "logging view", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindow", "View Alarms", 0));
        groupBox_image->setTitle(QApplication::translate("MainWindow", "detection image", 0));
        view_image->setText(QApplication::translate("MainWindow", "TextLabel (shows images)", 0));
        button_play->setText(QApplication::translate("MainWindow", "Play", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab), QApplication::translate("MainWindow", "View Images", 0));
        menu_File->setTitle(QApplication::translate("MainWindow", "&Appviz", 0));
        dock_status->setWindowTitle(QApplication::translate("MainWindow", "Command Panel", 0));
        groupBox->setTitle(QApplication::translate("MainWindow", "Ros Master", 0));
        label->setText(QApplication::translate("MainWindow", "Ros Master Url", 0));
        line_edit_master->setText(QApplication::translate("MainWindow", "http://localhost:11311/", 0));
        label_2->setText(QApplication::translate("MainWindow", "Ros IP", 0));
        line_edit_host->setText(QApplication::translate("MainWindow", "localhost", 0));
        label_3->setText(QApplication::translate("MainWindow", "Ros Hostname", 0));
        line_edit_topic->setText(QApplication::translate("MainWindow", "unused", 0));
        checkbox_use_environment->setText(QApplication::translate("MainWindow", "Use environment variables", 0));
        checkbox_remember_settings->setText(QApplication::translate("MainWindow", "Remember settings on startup", 0));
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindow", "Set the target to the current joint trajectory state.", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QApplication::translate("MainWindow", "Clear all waypoints and set the target to the current joint trajectory state.", 0));
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindow", "Connect", 0));
        quit_button->setText(QApplication::translate("MainWindow", "Quit", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
