#include "mainwindow.h"
#include "ui_mainwindow.h"
//#include "../Mesh.h"
#include <QTimer>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    (void) event;
    ui->visualizer->exit();
}
