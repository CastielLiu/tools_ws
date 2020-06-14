#PCD工具

##tools

### pcd2ros 
	将pcd文件转换为ros消息类型并发布出去
* **`argvs`**
    1. pcdFile  pcd文件
    2. hz 发布频率

* **`run`**
    rosrun pcd_tools pcd2ros_node 1.pcd 10

### ros2pcd 
	订阅ros雷达点云消息并保存为pcd
* **`argvs`**
    1. savePath  保存路径
    2. topicName     点云话题名称

* **`run`**
    rosrun pcd_tools ros2pcd_node savePath topicName

### register_demo 
	点云配准demo, 传入pcd文件并将起平移旋转，利用配准算法匹配旋转前后的点云，实时显示配准结果，按空格键进入下一次迭代
* **`argvs`**
    1. pcdFile  pcd文件

* **`run`**
    rosrun pcd_tools ros2pcd_node savePath topicName
