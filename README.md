
# ALOHA采集
## 源文件编译

如果没有修改源文件，编译只需要执行一次：

```shell
conda deactivate
[follow1] catkin_make
[follow2] catkin_make
```
## 启动 roscore
新建一个终端执行:

```shell
roscore
```
遥控
## 打开can通信、机械臂和摄像机
在remote control中 如果部署则是follow_control
```shell
./can.sh
./start.sh
./start_camera.sh
```
在master中点击i键开启机械臂示教


## 设置路径
sample.py设置Max_step，directory_path，video_path

## 校准摄像头
将callback中的save改为False，调整摄像头的位置，然后将save改为True

## 开始采集

```shell
[follow1] cd follow1 && source devel/setup.bash
[follow1] rosrun arm_control sample.py
```

## 部署
```shell
[follow1] cd follow1 && source devel/setup.bash 
rosrun arm_control shit_moutain.py
```


