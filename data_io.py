import numpy as np


def read_txt(path):
    line = {}
    data = np.loadtxt(path, delimiter=";")
    pointclouds = data[:, 3:]
    z_max=np.max(data[:,-1])
    z_min=np.min(data[:,-1])
    temp = 0
    line_idx = data[0, 0]
    for row in range(data.shape[0]):
        if line_idx != data[row, 0]:
            point = dict(zip(map(int, data[temp:row, 1]), data[temp:row, :]))
            # point = {}
            # for i in range(data[temp:row, :].shape[0]):
            #     point.update({f"{int(data[temp + i, 1])}": data[temp + i, :]})
            line.update({f"{int(line_idx)}": point})
            line_idx = data[row, 0]
            temp = row

    print(len(line))

    return line


txt_path = "data/xiaomi/1/short1_ijkxyz.txt"
a = read_txt(txt_path)
