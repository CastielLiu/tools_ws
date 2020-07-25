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
#include <QMessageBox>
#include <QThread>
#include <QTimer>

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

    void on_tabWidget_currentChanged(int index);

    void on_pushButton_forceQuit_clicked();
    void on_pushButton_video2images_select_clicked();

    void on_pushButton_video2images_start_clicked();

private:
    void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QString m_appDir;
    QString m_toolScriptsDir;
    bool m_isProcessing;
    bool m_forceQuitCurrentTool;

    Video2gif* m_video2gif;
    Video2images *m_video2images;
    int m_lastTabIndex;

    QStringListModel m_logModel;
};
#endif // MAINWINDOW_H
