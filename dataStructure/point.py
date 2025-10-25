import numpy as np


class PointCloud:
    def __init__(self):
        self.indexicalCloud = None #嵌套字典，ijkxyz
        self.zMax = None
        self.zMin = None
        self.lineNum = None
        self.lineEndKey = None  # 最后一个line对于的index
        self.currentLinePoints = LinePoints()  # 正在处理的line，比如在界面中显示的line
        self.cloudData = None #xyz


    def readIndexicalFromIjkxyzTxt(self, filePath):
        lines = {}
        data = np.loadtxt(filePath, delimiter=";")
        temp = 0
        lineKey = data[0, 0]
        for row in range(data.shape[0]):
            if lineKey != data[row, 0]:  # 发现新的行号
                point = dict(zip(map(int, data[temp:row, 1]), data[temp:row, :]))
                lines.update({f"{int(lineKey)}": point})
                lineKey = data[row, 0]
                temp = row

        #print(len(lines))
        self.indexicalCloud = lines
        self.zMin = np.min(data[:, -1])
        self.zMax = np.max(data[:, -1])
        self.lineEndKey = list(self.indexicalCloud.keys())[-1]
        self.cloudData = data[:, 3:]


class LinePoints:
    def __init__(self):
        self.lineNo = None  # 该line在点云中的序号，从0起，连续计数
        self.lineKey = None  # 该line在点云中的索引号
        self.lineData = None


class TrajectoryPoints:
    def __init__(self):
        self.trajectory = None

