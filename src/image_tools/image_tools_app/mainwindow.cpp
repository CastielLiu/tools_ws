#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QString appDir, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_appDir(appDir)
{
    m_toolScriptsDir = m_appDir + "/..";
    m_video2gif = new Video2gif(m_toolScriptsDir.toStdString());
    m_video2images = new Video2images(m_toolScriptsDir.toStdString());
    m_images2gif = new Images2gif(m_toolScriptsDir.toStdString());
    m_imagesCutter = new ImagesCutter(m_toolScriptsDir.toStdString());
    m_isProcessing = false;

    ui->setupUi(this);
    this->setWindowTitle(tr("图像/视频处理工具"));
    ui->listView_log->setModel(&m_logModel);

    ui->tabWidget->setCurrentIndex(tabWidgetTab_video2gif);
    //手动发送信号，以更新内容
    emit ui->tabWidget->currentChanged(tabWidgetTab_video2gif);

    connect(this, SIGNAL(addDataToLogListView(const QString&)), this, SLOT(onAddDataToLogListView(const QString&)));
}

MainWindow::~MainWindow()
{
    m_forceQuitCurrentTool = true;
    QThread::sleep(1);
    delete ui;
    delete m_video2gif;
    delete m_video2images;
    delete m_images2gif;
    delete m_imagesCutter;
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    int button = QMessageBox::question(this,"退出确认",
                                       "确定要退出应用程序?",
                                       QMessageBox::Yes|QMessageBox::No,
                                       QMessageBox::Yes);
    if(button == QMessageBox::Yes)
    {
        event->accept();
        QMainWindow::closeEvent(event);
    }
    else
        event->ignore();
}

void MainWindow::onAddDataToLogListView(const QString& data)
{
    m_logModel.insertRows(m_logModel.rowCount(),1);
    QVariant newRow(data);
    m_logModel.setData(m_logModel.index(m_logModel.rowCount()-1),newRow);
    ui->listView_log->setCurrentIndex(m_logModel.index(m_logModel.rowCount()-1));
}

void MainWindow::on_pushButton_video2gif_select_clicked()
{
    QString videoName = QFileDialog::getOpenFileName(this,"select video","/home");
    if(videoName.isEmpty())
        return;

    ui->lineEdit_video2gif_video->setText(videoName);
}

void MainWindow::on_pushButton_video2gif_start_clicked()
{
    if(m_isProcessing)
    {
        onAddDataToLogListView("system is processing now, please try again a later.");
        return;
    }
    if(ui->comboBox_video2gif_interval->currentText().isEmpty())
    {
        onAddDataToLogListView(tr("图片间隔为空!"));
        return;
    }
    if(ui->lineEdit_video2gif_video->text().isEmpty())
    {
        onAddDataToLogListView(tr("请选择视频文件!"));
        return;
    }
    if(ui->lineEdit_video2gif_expectTime->text().isEmpty())
    {
        onAddDataToLogListView(tr("期望时长为空!"));
        return;
    }

    QString imageInterval = ui->comboBox_video2gif_interval->currentText();
    QString totalTime = ui->lineEdit_video2gif_expectTime->text();

    m_video2gif->video_name = ui->lineEdit_video2gif_video->text().toStdString();
    m_video2gif->image_interval = imageInterval.toStdString();
    m_video2gif->total_time = totalTime.toStdString();
    std::thread t(&MainWindow::processThread,this,taskType_video2gif);
    t.detach();
}

void MainWindow::processThread(taskType task)
{
    m_isProcessing = true;
    std::string cmd;

    if(taskType_video2gif == task)
        cmd = m_video2gif->cmd();
    else if(taskType_video2images == task)
        cmd = m_video2images->cmd();
    else if(taskType_images2gif == task)
        cmd = m_images2gif->cmd();
    else if(taskType_imagesCutter == task)
        cmd = m_imagesCutter->cmd();

    emit addDataToLogListView(QString::fromStdString(cmd));
    FILE* fp = popen(cmd.c_str(),"r");
    if(fp == nullptr)
    {
        //info
        m_isProcessing = false;
        return;
    }
    m_forceQuitCurrentTool = false;
    char buf[100] ;
    while(fgets(buf,100,fp) != nullptr)
    {
        //std::cout << std::string(buf) << std::endl;
        QString data = QString::fromLocal8Bit(buf);
        emit addDataToLogListView(data);

        if(m_forceQuitCurrentTool)
        {
            std::cout << "m_forceExit" << std::endl;
            emit addDataToLogListView("force quit!");
            system("pkill python"); //将关闭所有python进程,dangerous!
            break;
        }
    }
    if(fp) //关闭前先判断是否为空，防止释放空内存
    {
        std::cout << "close pip"<< std::endl;
        pclose(fp);
    }

    std::cout << "processing ok." << std::endl;
    m_isProcessing = false;
}

void MainWindow::on_pushButton_forceQuit_clicked()
{
    QMessageBox msgBox(QMessageBox::Question, tr("退出"), tr("退出当前应用程序？"),
                       QMessageBox::Yes|QMessageBox::No|QMessageBox::Cancel,this);
    //添加按键并重命名
    msgBox.button(QMessageBox::Yes)->setText(tr("退出应用程序"));
    msgBox.button(QMessageBox::No)->setText(tr("中断当前任务"));
    msgBox.button(QMessageBox::Cancel)->setText(tr("取消"));

    //设置默认按键
    if(m_isProcessing)
        msgBox.setDefaultButton(QMessageBox::No);
    else
        msgBox.setDefaultButton(QMessageBox::Yes);
    //启动msgBox事件循环
    int button = msgBox.exec();

    //点击叉号或者Cancel均返回QMessageBox::Cancel
    if(button == QMessageBox::Cancel)
    {
        return;
    }
    else if(button == QMessageBox::Yes)
    {
        this->close();
    }
    else if(button == QMessageBox::No)
    {
        m_forceQuitCurrentTool = true;
    }
}

void MainWindow::on_pushButton_video2images_select_clicked()
{
    QString videoName = QFileDialog::getOpenFileName(this,"select video","/home");
    if(videoName.isEmpty())
        return;

    ui->lineEdit_video2images_video->setText(videoName);
}

void MainWindow::on_pushButton_video2images_start_clicked()
{
    if(m_isProcessing)
    {
        onAddDataToLogListView("system is processing now, please try again a later.");
        return;
    }
    if(ui->lineEdit_video2images_video->text().isEmpty())
    {
        onAddDataToLogListView(tr("请选择视频文件!"));
        return;
    }
    m_video2images->video_name = ui->lineEdit_video2images_video->text().toStdString();
    std::thread t(&MainWindow::processThread,this,taskType_video2images);
    t.detach();
}

void MainWindow::on_pushButton_images2gif_select_clicked()
{
    QString imagesDir = QFileDialog::getExistingDirectory(this,tr("选择图片路径"),"/home",QFileDialog::ReadOnly);
    if(imagesDir.isEmpty())
        return;
    ui->lineEdit_images2gif_imagePath->setText(imagesDir);
}

void MainWindow::on_pushButton_images2gif_start_clicked()
{
    if(m_isProcessing)
    {
        onAddDataToLogListView("system is processing now, please try again a later.");
        return;
    }
    m_images2gif->img_dir = ui->lineEdit_images2gif_imagePath->text().toStdString();
    m_images2gif->start_seq = ui->lineEdit_images2gif_startSeq->text().toStdString();
    m_images2gif->end_seq = ui->lineEdit_images2gif_endSeq->text().toStdString();
    m_images2gif->img_suffix = ui->comboBox_images2gif_imageType->currentText().toStdString();
    m_images2gif->total_time = ui->lineEdit_images2gif_expectTime->text().toStdString();
    m_images2gif->image_interval = ui->comboBox_images2gif_interval->currentText().toStdString();
    m_images2gif->scale = ui->lineEdit_images2gif_scale->text().toStdString();
    std::thread t(&MainWindow::processThread,this,taskType_images2gif);
    t.detach();
}

void MainWindow::on_pushButton_help_clicked()
{
    QString title;
    QString content;
    if(ui->tabWidget->currentIndex()==tabWidgetTab_video2gif)
    {
        title = tr("视频转gif");
        content = g_toolDescription_video2gif;
    }
    else if(ui->tabWidget->currentIndex()==tabWidgetTab_images2gif)
    {
        title = tr("图片转gif");
        content = g_toolDescription_images2gif;
    }
    else if(ui->tabWidget->currentIndex()==tabWidgetTab_video2images)
    {
        title = tr("视频拆分图片");
        content = g_toolDescription_video2images;
    }
    else if(ui->tabWidget->currentIndex()==tabWidgetTab_imagesCutter)
    {
        title = tr("图片批量裁剪");
        content = g_toolDescription_imagesCutter;
    }



    QMessageBox::information(this,title,content);
}

void MainWindow::on_pushButton_imagesCutter_select_clicked()
{
    QString imagesDir = QFileDialog::getExistingDirectory(this,tr("选择图片路径"),"/home",QFileDialog::ReadOnly);
    if(imagesDir.isEmpty())
        return;
    ui->lineEdit_imagesCutter_imagePath->setText(imagesDir);
}

void MainWindow::on_pushButton_imagesCutter_start_clicked()
{
    if(m_isProcessing)
    {
        onAddDataToLogListView("system is processing now, please try again a later.");
        return;
    }
    m_imagesCutter->images_dir = ui->lineEdit_imagesCutter_imagePath->text().toStdString();
    m_imagesCutter->image_suffix = ui->comboBox_imagesCutter_imageType->currentText().toStdString();
    m_imagesCutter->expect_w = ui->lineEdit_imagesCutter_w->text().toStdString();
    m_imagesCutter->expect_h = ui->lineEdit_imagesCutter_h->text().toStdString();
    m_imagesCutter->use_w_h_as_target_size = ui->checkBox_imagesCutter_asTargetSize->isChecked();
    std::thread t(&MainWindow::processThread,this,taskType_imagesCutter);
    t.detach();
}
