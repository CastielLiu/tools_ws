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
#include <QCloseEvent>
#include <QDesktopWidget>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QString appDir, QWidget *parent = nullptr);
    ~MainWindow();
    void configToolBar();
    void processThread(taskType task);
    void showInfo();

private slots:
    void onAddDataToLogListView(const QString &data);
    void on_pushButton_video2gif_select_clicked();
    void on_pushButton_video2gif_start_clicked();
    void on_pushButton_forceQuit_clicked();
    void on_pushButton_video2images_select_clicked();
    void on_pushButton_video2images_start_clicked();
    void on_pushButton_images2gif_select_clicked();
    void on_pushButton_images2gif_start_clicked();
    void on_pushButton_help_clicked();
    void on_pushButton_imagesCutter_select_clicked();
    void on_pushButton_imagesCutter_start_clicked();
    void on_pushButton_videoCutter_select_clicked();
    void on_pushButton_videoCutter_start_clicked();
    void on_pushButton_imagesAddLogo_selectImageDir_clicked();
    void on_pushButton_imagesAddLogo_selectLogo_clicked();
    void on_pushButton_imagesAddLogo_start_clicked();
    void onAction_author_trigger();

    void on_pushButton_imagesRename_select_clicked();
    void on_pushButton_imagesRename_start_clicked();
    void on_tabWidget_imageTools_currentChanged(int index);

    void on_tabWidget_videoTools_currentChanged(int index);

    void on_pushButton_videoAudio_selectVideo_clicked();

    void on_pushButton_videoAudio_selectAudio_clicked();

    void on_pushButton_videoAudio_start_clicked();

    void on_comboBox_videoAudio_operateType_currentIndexChanged(int index);

signals:
    void addDataToLogListView(const QString& data);

private:
    void closeEvent(QCloseEvent *event) override;

private:
    Ui::MainWindow *ui;
    QString m_appDir;
    QString m_toolScriptsDir;
    bool m_isProcessing;
    bool m_forceQuitCurrentTool;

    Video2gif     *m_video2gif;
    Video2images  *m_video2images;
    Images2gif    *m_images2gif;
    ImagesCutter  *m_imagesCutter;
    VideoCutter   *m_videoCutter;
    ImagesAddLogo *m_imagesAddLogo;
    ImagesRename  *m_imagesRename;

    int m_lastTabIndex;

    QStringListModel m_logModel;
    std::mutex m_logListViewMutex;
};
#endif // MAINWINDOW_H
