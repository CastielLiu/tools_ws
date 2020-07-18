#图片处理工具

##tools

### video2gif.py
    将视频转换为gif动图,可在图片上添加mask,　将mask.jpg与图片放在同级目录
* **`argvs`**
    1. srcVideo 视频路径
    2. resolution gif精度，每reslotion个视频帧取一帧存入gif
    3. totalTime gif总时长，可选参数，默认为视频总时长

* **`run`**
    python video2gif.py srcVideo resolution [totalTime]

### images2gif.py
    将一系列图片转换为gif动图
* **`argvs`**
    1. path 图片路径(绝对路径/相对路径)
    2. startSeq 图片起始序号
    3. endSeq 图片终止序号
    4. suffix 图片后缀、拓展名(png,jpg...)
    5. duration 生成gif图的总时长
    6. resolution 取图精度，隔resolution张取一张　(默认为1)
    7. scale   缩放比例(默认为1)
    
* **`run`**
    python images2gif.py path,startSeq,endSeq,suffix,duration,(resolution,scale)

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

### video2images.py
    将视频拆分为图片，保存于在当前目录自动创建图像文件夹
* **`argvs`**
    1. srcVideo   视频路径

* **`run`**
    python video_cutter.py srcVideo 

### video_post.py 
    视频合成，将一张图片合成到另一张图片上