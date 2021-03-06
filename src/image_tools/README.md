#　图片处理工具
** 运行./install.sh 安装工具所依赖的python模块

##　tools

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
    6. resolution 取图精度，每resolution张取一张　(默认为1)
    7. scale   缩放比例(默认为1)
    
* **`run`**
    python images2gif.py path,startSeq,endSeq,suffix,duration,(resolution,scale)

### images2video.py
    将一系列图片转换为视频
* **`argvs`**
    1. path 图片路径(绝对路径/相对路径)
    2. startSeq 图片起始序号
    3. endSeq 图片终止序号
    4. suffix 图片后缀、拓展名(png,jpg...)
    5. hz 帧率
    6. resolution 取图精度，每resolution张取一张　(默认为1)
    7. scale   缩放比例(默认为1)
    
* **`run`**
    python images2video.py path,startSeq,endSeq,suffix,hz,(resolution,scale)


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
    程序启动后，按空格键播放或暂停视频，拖动Beg和End滚动条选择裁剪片段
    可使用鼠标选择视频裁剪区域，

    按键说明：
    [q] 退出程序
    [ ] 暂停或播放视频
    [d] 循环播放选定的视频区域
    [s] 保存视频片段
    [<] 查看上一帧图像
    [>] 查看下一帧图像

* **`argvs`**
    1. srcVideo   视频路径

* **`run`**
    python video_cutter.py srcVideo 

### video2images.py
    将视频拆分为图片，保存于在当前目录自动创建图像文件夹
* **`argvs`**
    1. srcVideo   视频路径

* **`run`**
    python video2images.py srcVideo 

### video_post.py 
    视频合成，将一张图片合成到另一张图片上

### images_add_mask.py
    图像批量添加面具(logo),两种工作模式, 0:所有图像在同一位置添加mask 1:每张图片自由选择位置
    保留原图像，自动新建文件夹保存处理后的图像

* **`argvs`**
    1. path           图片路径   image/
    2. suffix         图片后缀名 jpg
    3. mask_image     需要添加的mask文件
    4. mode           添加模式
        0:所有图像在同一位置添加mask 1:每张图片自由选择位置

* **`run`**
    python images_add_mask.py path suffix mask_image　mode

　　程序启动后，默认显示第一张图像，使用鼠标点击mask添加的位置，位置可多次调整，位置确定后点击's'保存图像，若为模式0,所有图像均被处理保存;　若为模式1,点击's'后切换到下一张,mask位置确定后再次点击‘s’，切换到下一张图片上，将保留上一次的mask位置，若当前图像不需要mask,只需点击右侧边缘无法添加mask的位置即可清除原有mask，然后点击's'保存即可
　　

### images_cutter.py
    图片裁剪工具，批量裁剪图片
* **`argvs`**
    1. path     图片路径   image/
    2. suffix   图片后缀名 jpg
    3. mode     裁剪模式(可选参数)　默认为0:批量裁剪　 1:逐张裁剪
    4. w:h      裁剪宽高比(可选参数)　640:480
    5. asTargetScale 是否将按宽高比尺寸输出图像(可选参数)
        0: 默认裁剪后直接输出 1: 按照宽高尺寸缩放后输出

* **`run`**
    python images_cutter.py path suffix (mode w:h asTargetScale)

    程序运行后默认为第一张图片，使用鼠标按住左键拖动选择裁剪区域，按滚轮确定裁剪
    若输入参数中包含宽高比，选择框自动按比例显示，若输入asTargetScale参数为1，图像将缩放为w:h输出
    批量裁剪：裁剪区域选定后，按下's'键，所有图片按照框选区域进行自动裁剪
    逐张裁剪：裁剪区域选定后按's'键保存当前裁剪后的图片，并跳转到下一张，可选择新的裁剪范围

    裁剪过程中可使用< > 切换图片


### images_rename.py
    批量图片重命名, 用于多个文件夹图像合并预处理，前提是图像文件名为有序数字　
    首先列出文件夹中所有指定类型的图片，根据其数字名称进行排序，
    将排序后的图像文件，按照startSeq++的顺序重命名为中间文件，最后再将中间文件命名为目标文件
    自动过滤文件名不是数字的图片文件

* **`argvs`**
    1. path     　　　图片路径   image/
    2. suffix   　　　图片后缀名 jpg
    3. startSeq      输出图像起始编号

* **`run`**
    python images_rename.py path suffix startSeq



