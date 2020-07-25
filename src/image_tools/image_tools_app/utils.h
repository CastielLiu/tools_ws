#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <QString>

const QString g_toolDescription_video2gif =
        QString("视频转gif动图\n图片间隔：视频帧间隔采样\n期望时长：生成gif的时长，默认与视频时长一致");

const QString g_toolDescription_videoCutter =
        QString("视频裁剪\n等待开发");

const QString g_toolDescription_video2images =
        QString("视频转图片\n等待开发");


enum taskType
{
    taskType_test,
    taskType_video2gif,
    taskType_video2images,
    taskType_videoCutter,
};

enum tabWidgetTab
{
    tabWidgetTab_video2gif = 0,
    tabWidgetTab_video2images = 1,
    tabWidgetTab_videoCutter = 2,
};

class Video2gif
{
public:
    Video2gif(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/video2gif.py";
    }
    std::string tool_script;
    std::string video_name;
    std::string image_interval;
    std::string total_time; //optional

    std::string cmd()
    {
        if(total_time=="0") total_time = "";
        std::string _cmd = std::string("python ")+tool_script+" "+video_name+" "+image_interval+" "+total_time;
        return _cmd;
    }

};


#endif // UTILS_H
