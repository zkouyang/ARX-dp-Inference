import h5py
import numpy as np

# 指定HDF5文件路径
file_path = "/home/dc/Desktop/arx-follow-V2/arx-follow/follow1/src/arm_control/scripts/test.hdf5"

# 打开HDF5文件以读写模式
with h5py.File(file_path, 'r+') as file:
    # 检查文件中是否存在 '/observations/qpos' 和 '/action'
    if '/observations/qpos' in file and '/action' in file:
        # 获取数据集
        qpos_dataset = file['/observations/qpos']
        action_dataset = file['/action']

        print(len(qpos_dataset))
        for i in range(len(qpos_dataset)):
            qpos_data = qpos_dataset[i]
            action_data = action_dataset[i]

            print(action_data[6]/qpos_data[6],action_data[13]/qpos_data[13])
            new_rmse = np.sqrt(np.mean((qpos_data - action_data) ** 2))

            print("-----------------------------------")
    else:
        print("Dataset '/observations/qpos' or '/action' not found in the file.")
