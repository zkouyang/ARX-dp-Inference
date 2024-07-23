import cv2
import pyrealsense2 as rs
import numpy as np

# 初始化Realsense相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # 设置颜色流的分辨率和格式

pipeline.start(config)

try:
    while True:
        # 等待一帧数据
        frames = pipeline.wait_for_frames()

        # 获取颜色帧
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # 将颜色帧转换为numpy数组
        color_image = np.asanyarray(color_frame.get_data())

        # 显示图像
        cv2.imshow('Realsense Color Image', color_image)

        # 检测键盘输入，按下ESC键退出循环
        key = cv2.waitKey(1)
        if key == 27:
            break

finally:
    # 关闭Realsense管道
    pipeline.stop()
    cv2.destroyAllWindows()

