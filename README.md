1. 在follow1文件夹的上一级完成下列操作：
./can.sh
./start_arx5.sh
./start_camera.sh

2. 进入follow1文件夹, 添加ROS包路径：
source devel/setup.bash

3. 启动
conda activate robodiff
python dp/eval_real_ros.py -ip="ckpt所在路径" -op="视频存放的文件夹"


output_path(-op)中保存的文件：
- episode_N
    - mid
        - mid.mp4
    - right
        - right.mp4
    - display.mp4
