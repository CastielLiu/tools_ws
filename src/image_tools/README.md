#图片处理工具

##tools

### video2gif.py
    将视频转换为gif动图
* **`argvs`**
    1. srcVideo 视频路径
    2. resolution gif精度，每reslotion个视频帧取一帧存入gif
    3. totalTime gif总时长，可选参数，默认为视频总时长

* **`run`**
    python video2gif.py srcVideo resolution [totalTime]


### video_cutter.py
    视频裁剪
* **`argvs`**
    1. srcVideo   视频路径
    2. startTime  开始裁剪时刻
    3. endTime    停止裁剪时刻
    4. resolution 裁剪精度，可选参数，默认为1 

* **`run`**
    python video_cutter.py srcVideo startTime endTime [resolution]

### video_cutter2.py
    视频裁剪,根据显示结果手动选择开始裁剪和停止裁剪
    程序启动后，按任意键播放视频帧，需要开始裁剪时按下s键开始裁剪，按q键停止裁剪

* **`argvs`**
    1. srcVideo   视频路径

* **`run`**
    python video_cutter.py srcVideo 