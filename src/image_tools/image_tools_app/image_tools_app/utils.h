#ifndef UTILS_H
#define UTILS_H
#include <iostream>
#include <QString>
#include <vector>
#include <cstdio>


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

const QString g_toolDescription_images2video =
        QString("d");

const QString g_toolDescription_video2images =
        QString("将视频拆分为图片并保存在独立文件夹\n"
                "支持MP4 / AVI / GIF");

const QString g_toolDescription_imagesCutter =
        QString("裁剪目标文件夹下所有指定后缀的图片,裁剪后的图片保存在独立文件夹，不删除原文件\n"
                "点击运行后,将显示一张目标图片, 可使用 < , > 切换图片\n"
                "使用鼠标左键框选裁剪区域,按键盘's'键开始裁剪\n"
                "按'q'键放弃裁剪并退出\n\n"
                "图片类型    : 图片后缀名\n"
                "锁定宽高比   : 裁剪时是否锁定宽高比\n"
                "是否目标尺寸 : 是否使用输入的宽高作为目标尺寸\n"
                "批量裁剪    :所有图片使用相同的裁剪区域\n"
                "逐张裁剪    :每张图片使用不同的裁剪区域");

const QString g_toolDescription_videoCutter =
        QString("可视化视频裁剪工具,裁剪后的视频与原视频位于相同路径\n"
                "点击运行后,视频将自动播放,点击[空格]可暂停或继续播放视频,\n\n"
                "功能1: 裁剪视频片段\n"
                "通过拖动Beg和End滚动条选择裁剪片段，拖动Pos滚动条可实时更新视频位置;\n\n"
                "功能2: 选择裁剪区域, 使用鼠标选择裁剪区域\n\n"
                "点击‘d’可以单独播放裁剪片段, 点击'q'键退出裁剪\n"
                "点击 '<' 和 '>' 可逐帧切换视频\n"
                "裁剪片段选定后按's'键保存视频.");

const QString g_toolDescription_imagesAddLogo =
        QString("图像可视化批量添加logo,添加后的图像保存在新的文件夹中\n"
                "图片类型: 图片后缀名\n"
                "添加模式: 批量添加,所有图像在相同的位置添加logo\n"
                "添加模式: 逐张添加,每张图片在不同的位置添加logo\n"
                "点击's'保存图像，点击'q'退出工具\n"
                "滑动Alpha滚动条可更改logo透明度");

const QString g_toolDescription_imagesRename =
        QString("图片批量重命名,直接修改文件名,不保留副本\n"
                "输出文件顺序与原文件数字顺序相同, 内置转存机制, 不会导致文件丢失\n\n"
                "图片路径: 需要重命名的图像所在位置\n"
                "图片类型: 图片后缀名\n"
                "起始编号: 输出文件的最小编号值, 文件名称依次递增\n");


//任务类型，用于选择对应的工具
//顺序可更改
enum taskType
{
    taskType_images2gif,
    taskType_images2video,
    taskType_imagesCutter,
    taskType_imagesAddLogo,
    taskType_imagesRename,

    taskType_video2gif,
    taskType_video2images,
    taskType_videoCutter,
    taskType_videoAddAudio,
    taskType_videoExtractAudio,
    taskType_videoTransFormat,
    taskType_audioTransFormat,
};

//应与实际索引一致
enum stackWidget
{
    stackWidget_imageTools = 0,
    stackWidget_videoTools = 1,
};

//图片工具tabWidget索引，应与实际索引一致
enum imageToolsWidget_Tab
{
    imageToolsWidget_Tab_images2gif,
    imageToolsWidget_Tab_images2video,
    imageToolsWidget_Tab_imagesCutter,
    imageToolsWidget_Tab_imagesAddLogo,
    imageToolsWidget_Tab_imagesRename,
};

//图片工具描述，顺序应与imageToolsWidget_Tab对应
static std::vector<QString> g_imageToolsDiscription = {
    g_toolDescription_images2gif,
    g_toolDescription_images2video,
    g_toolDescription_imagesCutter,
    g_toolDescription_imagesAddLogo,
    g_toolDescription_imagesRename
};

//视频工具tabWidget索引，应与实际索引一致
enum videoToolsWidget_Tab
{
    videoToolsWidget_Tab_video2gif ,
    videoToolsWidget_Tab_video2images ,
    videoToolsWidget_Tab_videoCutter ,
};

//视频工具描述，顺序应与videoToolsWidget_Tab对应
static std::vector<QString> g_videoToolsDiscription = {
    g_toolDescription_video2gif,
    g_toolDescription_video2images,
    g_toolDescription_videoCutter
};

//视频音频stackWidget索引，应与实际索引一致
enum videoAudioToolsType
{
    videoAudioToolsType_addAudio,
    videoAudioToolsType_extractAudio,
    videoAudioToolsType_videoTransFormat,
    videoAudioToolsType_audioTransFormat,
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

class Images2video
{
public:
    Images2video(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/images2video.py";
    }
    std::string tool_script;
    std::string img_dir;
    std::string start_seq;
    std::string end_seq;
    std::string img_suffix;
    std::string output_hz;
    std::string image_interval; //optional
    std::string scale; //optional

    std::string cmd()
    {
        image_interval = std::to_string(atoi(image_interval.c_str()) + 1);

        std::string _cmd = std::string("python ")+tool_script+" "+img_dir+" "
                                                 +start_seq + " "
                                                 +end_seq + " "
                                                 +img_suffix + " "
                                                 +output_hz + " "
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
    int mode;
    std::string tool_script;
    std::string images_dir;
    std::string image_suffix;
    std::string expect_w;
    std::string expect_h;
    bool use_w_h_as_target_size;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+images_dir + " "
                                                 +image_suffix + " " + std::to_string(mode) + " ";
        if(expect_w != "-1" && expect_h != "-1")
        {
            _cmd += expect_w + ":" + expect_h + " ";
            if(use_w_h_as_target_size)
                _cmd += "1";
        }
        return _cmd;
    }
};

class ImagesAddLogo
{
public:
    ImagesAddLogo(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/images_add_mask.py";
    }
    std::string tool_script;
    std::string images_dir;
    std::string image_suffix;
    std::string logo;
    std::string add_mode;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+images_dir + " "
                                                 +image_suffix + " "
                                                 +logo + " " + add_mode;
        return _cmd;
    }
};

class ImagesRename
{
public:
    ImagesRename(const std::string& scriptsDir)
    {
        tool_script = scriptsDir + "/images_rename.py";
    }
    std::string tool_script;
    std::string images_dir;
    std::string image_suffix;
    std::string start_seq;

    std::string cmd()
    {
        std::string _cmd = std::string("python ")+tool_script+" "+images_dir + " "
                                                 +image_suffix + " "
                                                 +start_seq;
        return _cmd;
    }
};

class VideoAudioTool
{
public:
    VideoAudioTool(taskType _type)
    {
        type = _type;
    }
    std::string cmd()
    {
        std::string out_cmd;
        if(type == taskType_videoAddAudio)
        {
            int dotIndex = inputVideo.find_last_of('.');
            std::string prefix = inputVideo.substr(0,dotIndex);
            std::string suffix = inputVideo.substr(dotIndex,inputVideo.length());
            std::string outName = prefix + "_out." + suffix;

            out_cmd =  std::string("ffmpeg -i ") + inputVideo + " -i " + inputAudio +
                        " -strict -2 -vn " + outName;
        }
        else if(type == taskType_videoExtractAudio)
        {
            int dotIndex = inputVideo.find_last_of('.');
            std::string prefix = inputVideo.substr(0,dotIndex);
            //std::string suffix = inputVideo.substr(dotIndex,inputVideo.length());
            std::string outName = prefix + "_out." + outputFormat;
            out_cmd =  std::string("ffmpeg -i ") + inputVideo + " -vn " + outName;
        }
        else if(type == taskType_videoTransFormat)
        {
            int dotIndex = inputVideo.find_last_of('.');
            std::string prefix = inputVideo.substr(0,dotIndex);
            std::string outName = prefix + "_out." + outputFormat;
            out_cmd =  std::string("ffmpeg -i ") + inputVideo + " -strict -2 " + outName;
        }
        else if(type == taskType_audioTransFormat)
        {
            int dotIndex = inputAudio.find_last_of('.');
            std::string prefix = inputAudio.substr(0,dotIndex);
            std::string outName = prefix + "_out." + outputFormat;
            out_cmd =  std::string("ffmpeg -i ") + inputAudio + " -strict -2 " + outName;
        }

        return out_cmd + " -hide_banner";
    }

    taskType type;
    std::string inputAudio;
    std::string inputVideo;
    std::string output;
    std::string outputFormat;
    bool audioCyclePlay;

    std::string _cmd;
};

static void runCmdInShell(const std::string& cmd)
{
    std::string res = std::string("gnome-terminal -x bash -c ") + "\"" + cmd + "; read \"";
    system(res.c_str());
}


#endif // UTILS_H
