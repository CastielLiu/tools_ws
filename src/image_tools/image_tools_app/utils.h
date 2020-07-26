#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <QString>

const QString g_toolDescription_video2gif =
        QString("图片间隔: 视频帧间隔采样\n"
                "期望时长: 生成gif的时长，默认与视频时长一致.");

const QString g_toolDescription_images2gif =
        QString("工具要求图片按照数字顺序进行命名,(1.jpg 2.jpg)\n"
                "图片名称数字可以不连续\n\n"
                "图片类型: 图像文件后缀名\n"
                "起始编号: 图像名称的起始编号\n"
                "终止编号: 图像名称的终止编号"
                "期望时长: 生成gif的期望总时长"
                "图片间隔: 图片的采样间隔"
                "图片缩放: 图片缩放比例");

const QString g_toolDescription_videoCutter =
        QString("视频裁剪\n等待开发");

const QString g_toolDescription_video2images =
        QString("将视频拆分为图片并保存在独立文件夹\n"
                "支持MP4 / AVI / GIF");

const QString g_toolDescription_imagesCutter =
        QString("批量裁剪目标文件夹下所有指定后缀的图片,裁剪后的图片保存在独立文件夹，不删除原文件\n"
                "点击运行后,将显示一张目标图片,使用鼠标左键框选裁剪区域,\n"
                "完成后点击鼠标中键或键盘's'键开始裁剪\n"
                "按'q'键放弃裁剪\n\n"
                "图片类型  : 图片后缀名\n"
                "锁定宽高比 : 裁剪时是否锁定宽高比\n"
                "复选框    : 是否使用输入的宽高作为目标尺寸");

enum taskType
{
    taskType_video2gif,
    taskType_images2gif,
    taskType_video2images,
    taskType_videoCutter,
    taskType_imagesCutter,
};

enum tabWidgetTab
{
    tabWidgetTab_video2gif ,
    tabWidgetTab_images2gif,
    tabWidgetTab_video2images ,
    tabWidgetTab_videoCutter ,
    tabWidgetTab_imagesCutter,
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
        if(total_time=="-1") total_time = "";
        image_interval = std::to_string(atoi(image_interval.c_str()) + 1);
        std::string _cmd = std::string("python ")+tool_script+" "+video_name+" "+image_interval+" "+total_time;
        return _cmd;
    }
};

class Images2gif
{
public:
    Images2gif(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/images2gif.py";
    }
    std::string tool_script;
    std::string img_dir;
    std::string start_seq;
    std::string end_seq;
    std::string img_suffix;
    std::string total_time;
    std::string image_interval; //optional
    std::string scale; //optional

    std::string cmd()
    {
        image_interval = std::to_string(atoi(image_interval.c_str()) + 1);

        std::string _cmd = std::string("python ")+tool_script+" "+img_dir+" "
                                                 +start_seq + " "
                                                 +end_seq + " "
                                                 +img_suffix + " "
                                                 +total_time + " "
                                                 +image_interval+ " "
                                                 +scale;
        return _cmd;
    }
};

class Video2images
{
public:
    Video2images(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/video2images.py";
    }
    std::string tool_script;
    std::string video_name;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+video_name;
        return _cmd;
    }
};

class VideoCutter
{
public:
    VideoCutter(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/video_cutter2.py";
    }
    std::string tool_script;
    std::string video_name;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+video_name;
        return _cmd;
    }
};


class ImagesCutter
{
public:
    ImagesCutter(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/images_cutter.py";
    }
    std::string tool_script;
    std::string images_dir;
    std::string image_suffix;
    std::string expect_w;
    std::string expect_h;
    bool use_w_h_as_target_size;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+images_dir + " "
                                                 +image_suffix + " ";
        if(expect_w != "-1" && expect_h != "-1")
        {
            _cmd += expect_w + ":" + expect_h + " ";
            if(use_w_h_as_target_size)
                _cmd += "1";
        }
        return _cmd;
    }
};


#endif // UTILS_H
