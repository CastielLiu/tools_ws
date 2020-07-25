#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <thread>
#include <mutex>
#include "utils.h"
#include <cstdio>
#include <QDebug>
#include <QStringListModel>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QString appDir, QWidget *parent = nullptr);
    ~MainWindow();
    void addDataToLogListView(const QString &data);
    void processThread(taskType task);

private slots:
    void on_pushButton_video2gif_select_clicked();

    void on_pushButton_video2gif_start_clicked();

    void on_pushButton_test_clicked();

    void on_pushButton_stop_clicked();

    void on_tabWidget_currentChanged(int index);

private:
    Ui::MainWindow *ui;
    QString m_appDir;
    QString m_toolScriptsDir;
    bool m_isProcessing;
    bool m_forceExit;

    Video2gif* m_video2gif;

    QStringListModel m_logModel;
};
#endif // MAINWINDOW_H
