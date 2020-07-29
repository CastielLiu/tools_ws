#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QString appDir, QWidget *parent):
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_appDir(appDir)
{
    ui->setupUi(this);

    m_toolScriptsDir = m_appDir + "/../..";
    m_video2gif = new Video2gif(m_toolScriptsDir.toStdString());
    m_video2images = new Video2images(m_toolScriptsDir.toStdString());
    m_images2gif = new Images2gif(m_toolScriptsDir.toStdString());
    m_imagesCutter = new ImagesCutter(m_toolScriptsDir.toStdString());
    m_videoCutter = new VideoCutter(m_toolScriptsDir.toStdString());
    m_imagesAddLogo = new ImagesAddLogo(m_toolScriptsDir.toStdString());
    m_imagesRename = new ImagesRename(m_toolScriptsDir.toStdString());
    m_videoAudio = nullptr;
    m_isProcessing = false;

    //移动到屏幕中间
    QDesktopWidget *deskdop = QApplication::desktop();
    move((deskdop->width() - this->width())/2, (deskdop->height() - this->height())/2);

    setWindowTitle(tr("图像/视频处理工具"));
    setWindowIcon(QIcon(":/images/resouces/icon.jpg"));

    ui->listView_log->setModel(&m_logModel);
    ui->stackedWidget->setCurrentIndex(0);
    ui->tabWidget_imageTools->setCurrentIndex(0);
    ui->tabWidget_videoTools->setCurrentIndex(0);
    //手动发送信号，以更新内容
    //emit ui->tabWidget_imageTools->currentChanged(0);

    connect(this, SIGNAL(addDataToLogListView(const QString&)), this, SLOT(onAddDataToLogListView(const QString&)));
    connect(ui->action_author, SIGNAL(triggered()), this, SLOT(onAction_author_trigger()));
    QMessageBox::information(this,tr("温馨提示"),tr("文件或路径中请勿包含中文，否则可能出现异常错误！"));
    configToolBar();
}

void MainWindow::configToolBar()
{
    //QAction *videoToolAction = ui->toolBar->addAction(tr("视频处理工具"));
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
    delete m_videoCutter;
    delete m_imagesAddLogo;
    delete m_imagesRename;
    if(m_videoAudio!=nullptr)
        delete m_videoAudio;
}

void MainWindow::onAction_author_trigger()
{
    QMessageBox::information(this,tr("作者信息"),tr("作者: 文刀\n单位: 东南大学\n邮箱: castiel_liu@outlook.com"));
}

//关于屏幕尺寸以及界面位置
void MainWindow::showInfo()
{
    QDesktopWidget * desktop = QApplication::desktop();

    //获取程序所在屏幕是第几个屏幕
    int current_screen = desktop->screenNumber(this);
    //获取程序所在屏幕的尺寸
    QRect rect = desktop->screenGeometry(current_screen);
    //获取所有屏幕总大小
    QRect rectA = desktop->geometry();
    //获取所有屏幕的个数
    int screen_count = desktop->screenCount();
    //获取主屏幕是第几个
    int prim_screen = desktop->primaryScreen();

    QString temp = "total screen size = " + QString::number(screen_count);
    temp = temp + "\ncurrent screen num = " + QString::number(current_screen);
    temp = temp + "\ncurrent screen rect " + QString::number(rect.width()) + "*" + QString::number(rect.height());
    temp = temp + "\nwhole screen rect " + QString::number(rectA.width()) + "*" + QString::number(rectA.height());
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
    if(m_isProcessing)
    {
        emit addDataToLogListView("system is processing now, please try again a later.");
        return;
    }

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
    else if(taskType_videoCutter == task)
        cmd = m_videoCutter->cmd();
    else if(taskType_imagesAddLogo == task)
        cmd = m_imagesAddLogo->cmd();
    else if(taskType_imagesRename == task)
        cmd = m_imagesRename->cmd();
    else if(taskType_videoAddAudio == task ||
            taskType_videoExtractAudio == task ||
            taskType_videoTransFormat == task ||
            taskType_audioTransFormat == task)
    {
        cmd = m_videoAudio->cmd();
        runCmdInShell(cmd);
        emit addDataToLogListView(tr("工具正在终端中运行……"));
        m_isProcessing = false;
        return;
    }

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

        if(data[data.length()-1] == '\n')
            data = data.remove(data.length()-1,1);

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
        //std::cout << "close pip"<< std::endl;
        pclose(fp);
    }
    emit addDataToLogListView("current image tool exit.\n");
    //std::cout << "processing ok." << std::endl;
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
    if(ui->lineEdit_images2gif_imagePath->text().isEmpty())
    {
        addDataToLogListView(tr("输入错误: 图片路径为空"));
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
    QString title = tr("帮助");
    QString content;
    if(ui->stackedWidget->currentIndex() == stackWidget_imageTools)
    {
        size_t index = ui->tabWidget_imageTools->currentIndex();
        if(index >= g_imageToolsDiscription.size())
            content = "无描述";
        else
            content = g_imageToolsDiscription[index];
    }
    else if(ui->stackedWidget->currentIndex() == stackWidget_videoTools)
    {
        size_t index = ui->tabWidget_videoTools->currentIndex();
        if(index >= g_videoToolsDiscription.size())
            content = "无描述";
        else
            content = g_videoToolsDiscription[index];
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
    if(ui->lineEdit_imagesCutter_imagePath->text().isEmpty())
    {
        addDataToLogListView(tr("输入错误: 图片路径为空"));
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


void MainWindow::on_pushButton_videoCutter_select_clicked()
{
    QString video = QFileDialog::getOpenFileName(this,tr("选择视频"),"/home");
    if(video.isEmpty())
        return;
    ui->lineEdit_videoCutter_video ->setText(video);
}

void MainWindow::on_pushButton_videoCutter_start_clicked()
{
    if(ui->lineEdit_videoCutter_video->text().isEmpty())
    {
        addDataToLogListView(tr("输入错误: 视频文件为空"));
        return;
    }
    m_videoCutter->video_name = ui->lineEdit_videoCutter_video->text().toStdString();

    std::thread t(&MainWindow::processThread,this,taskType_videoCutter);
    t.detach();
}

void MainWindow::on_pushButton_imagesAddLogo_selectImageDir_clicked()
{
    QString imagesDir = QFileDialog::getExistingDirectory(this,tr("选择图片路径"),"/home",QFileDialog::ReadOnly);
    if(imagesDir.isEmpty())
        return;
    ui->lineEdit_imagesAddLogo_imagePath->setText(imagesDir);
}

void MainWindow::on_pushButton_imagesAddLogo_selectLogo_clicked()
{
    QString logo = QFileDialog::getOpenFileName(this,tr("选择logo"),ui->lineEdit_imagesAddLogo_imagePath->text());
    if(logo.isEmpty())
        return;
    ui->lineEdit_imagesAddLogo_logo ->setText(logo);
}

void MainWindow::on_pushButton_imagesAddLogo_start_clicked()
{
    if(ui->lineEdit_imagesAddLogo_imagePath->text().isEmpty())
    {
        addDataToLogListView(tr("错误: 图片路径为空"));
        return;
    }
    if(ui->lineEdit_imagesAddLogo_logo->text().isEmpty())
    {
        addDataToLogListView(tr("错误: logo文件为空"));
        return;
    }
    m_imagesAddLogo->images_dir = ui->lineEdit_imagesAddLogo_imagePath->text().toStdString();
    m_imagesAddLogo->logo = ui->lineEdit_imagesAddLogo_logo->text().toStdString();
    m_imagesAddLogo->image_suffix = ui->comboBox_imageAddLogo_imgType->currentText().toStdString();
    m_imagesAddLogo->add_mode = std::to_string(ui->comboBox_imagesAddLogo_mode->currentIndex());

    std::thread t(&MainWindow::processThread,this,taskType_imagesAddLogo);
    t.detach();
}

void MainWindow::on_pushButton_imagesRename_select_clicked()
{
    QString imageDir = QFileDialog::getExistingDirectory(this,tr("选择图片路径"),"/home",QFileDialog::ReadOnly);
    ui->lineEdit_imagesRename_imageDir->setText(imageDir);
}

void MainWindow::on_pushButton_imagesRename_start_clicked()
{
    if(ui->lineEdit_imagesRename_imageDir->text().isEmpty())
    {
        addDataToLogListView(tr("输入错误: 图片路径为空"));
        return;
    }
    bool ok ;
    ui->lineEdit_imagesRename_startSeq->text().toInt(&ok);
    if(!ok)
    {
        ui->lineEdit_imagesRename_startSeq->setText("1");
        addDataToLogListView(tr("输入错误: 起始编号应为数字, 自动修改为默认值1, 请重新运行."));
        return;
    }

    m_imagesRename->images_dir = ui->lineEdit_imagesRename_imageDir->text().toStdString();
    m_imagesRename->image_suffix = ui->comboBox_imagesRename_imgType->currentText().toStdString();
    m_imagesRename->start_seq = ui->lineEdit_imagesRename_startSeq->text().toStdString();

    std::thread t(&MainWindow::processThread,this,taskType_imagesRename);
    t.detach();
}

void MainWindow::on_tabWidget_imageTools_currentChanged(int index)
{
    if(ui->tabWidget_imageTools->currentWidget()->objectName() == "tab_changeToVideoTools")
    {
        ui->stackedWidget->setCurrentIndex(stackWidget_videoTools);
        ui->tabWidget_videoTools->setCurrentIndex(0);
    }
}

void MainWindow::on_tabWidget_videoTools_currentChanged(int index)
{
    if(ui->tabWidget_videoTools->currentWidget()->objectName() == "tab_changeToImageTools")
    {
        ui->stackedWidget->setCurrentIndex(stackWidget_imageTools);
        ui->tabWidget_imageTools->setCurrentIndex(0);
        //qDebug() << ui->tabWidget_videoTools->currentIndex();
    }
    else if(ui->tabWidget_videoTools->currentWidget()->objectName() == "tab_videoAudio")
    {
        ui->comboBox_videoAudio_operateType->setCurrentIndex(0);
        emit ui->comboBox_videoAudio_operateType->currentIndexChanged(0);
    }
}

void MainWindow::on_pushButton_videoAudio_selectVideo_clicked()
{
    QString video = QFileDialog::getOpenFileName(this,tr("视频文件"),"/home");
    if(video.isEmpty())
        return;
    ui->lineEdit_videoAudio_video->setText(video);
    ui->lineEdit_videoAudio_video->setAlignment(Qt::AlignRight);
}

void MainWindow::on_pushButton_videoAudio_selectAudio_clicked()
{
    QString audio = QFileDialog::getOpenFileName(this,tr("音频文件"),"/home","*.mp3 *.wav");
    if(audio.isEmpty())
        return;
    ui->lineEdit_videoAudio_audio->setText(audio);
    ui->lineEdit_videoAudio_audio->setAlignment(Qt::AlignRight);
}

void MainWindow::on_pushButton_videoAudio_start_clicked()
{
    if(m_isProcessing)
    {
        onAddDataToLogListView("system is processing now, please try again a later.");
        return;
    }

    if(m_videoAudio != nullptr)
        delete m_videoAudio;

    int toolsType = ui->comboBox_videoAudio_operateType->currentIndex();

    if(toolsType == videoAudioToolsType_addAudio)
    {
        m_videoAudio = new VideoAudioTool(taskType_videoAddAudio);
        m_videoAudio->inputVideo = ui->lineEdit_videoAudio_video->text().toStdString();
        m_videoAudio->inputAudio = ui->lineEdit_videoAudio_audio->text().toStdString();

        if(m_videoAudio->inputVideo.empty() || m_videoAudio->inputAudio.empty())
        {
            addDataToLogListView(tr("输入错误: 视频/音频文件为空"));
            return;
        }
    }
    else if(toolsType == videoAudioToolsType_extractAudio)
    {
        m_videoAudio = new VideoAudioTool(taskType_videoExtractAudio);
        m_videoAudio->inputVideo = ui->lineEdit_videoAudio_video->text().toStdString();
        m_videoAudio->outputFormat = ui->comboBox_videoAudio_extractAudioFormat->currentText().toStdString();
        if(m_videoAudio->inputVideo.empty())
        {
            addDataToLogListView(tr("输入错误: 视频文件为空"));
            return;
        }
    }
    else if(toolsType == videoAudioToolsType_videoTransFormat)
    {
        m_videoAudio = new VideoAudioTool(taskType_videoTransFormat);
        m_videoAudio->inputVideo = ui->lineEdit_videoAudio_video->text().toStdString();
        m_videoAudio->outputFormat = ui->comboBox_videoAudio_transVideoFormat->currentText().toStdString();
        if(m_videoAudio->inputVideo.empty())
        {
            addDataToLogListView(tr("输入错误: 视频文件为空"));
            return;
        }
    }
    else if(toolsType == videoAudioToolsType_audioTransFormat)
    {
        m_videoAudio = new VideoAudioTool(taskType_audioTransFormat);
        m_videoAudio->inputAudio = ui->lineEdit_videoAudio_audio->text().toStdString();
        m_videoAudio->outputFormat = ui->comboBox_videoAudio_transAudioFormat->currentText().toStdString();
        if(m_videoAudio->inputAudio.empty())
        {
            addDataToLogListView(tr("输入错误: 视频音频为空"));
            return;
        }
    }
    std::thread t(&MainWindow::processThread,this,m_videoAudio->type);
    t.detach();
}

//使得stackWidget随comboBox切换
void MainWindow::on_comboBox_videoAudio_operateType_currentIndexChanged(int index)
{
    ui->stackedWidget_videoAudioTools->setCurrentIndex(index);
}
