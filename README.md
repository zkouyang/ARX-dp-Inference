# 方舟无限Diffusion Policy部署框架

## 源文件编译

如果没有修改源文件，编译只需要执行一次：

```shell
conda deactivate
[follow1] catkin_make
[follow2] catkin_make
```
## 1.启动 roscore

```shell
# 打开一个新的终端
roscore
```

## 2.打开can通信

```shell
./can.sh
```

## 3.启动机械臂

根据需求打开指定的机械臂文件夹：

```shell
# 打开一个新的终端
[follow2] cd follow2 && source devel/setup.bash && roslaunch arm_control arx5.launch
```

## 4.启动摄像机

```shell
# 打开一个新的终端
[follow1] cd follow1 && source devel/setup.bash && roslaunch arm_control camera.launch
```
# 校正摄像头的位置
```shell
# 打开一个新的终端
conda activate robodiff
python /media/dc/CLEAR/arx/dp_inference/follow1/src/arm_control/scripts/camera_calibration.py
```

## 5.真机部署

新建一个终端执行插值程序：

```shell
[follow1] cd follow1 && source devel/setup.bash && rosrun arm_control linear_interpolation.py
```

另外建一个终端执行推理程序：

```shell
[dp](cd follow1 && source devel/setup.bash && cd ..) && conda activate robodiff && cd dp

python eval_real_ros.py --input_path "/media/dc/CLEAR/dataset/eval_data/real_drawer_320_240_10Hz/epoch=00500-val_action_mse_error=0.0251.ckpt"

```

机械臂归位：

归位之前需要先终止差值进程，然后执行如下命令：

```shell
[follow1]
cd follow1 && source devel/setup.bash
rosrun arm_control back_to_origin.py
```
