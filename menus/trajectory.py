from mdiSubWindow import MdiSubWindow
from PyQt5 import QtWidgets, QtCore, QtGui
from drawing import DrawingWidget
from utils.utils import timeNow2Str, printLog
import open3d as o3d
import numpy as np
import logging


class ActionTrajectory():
    def __init__(self, parent=None):
        self.parent = parent
        self.windowTrajectorySetting = None


#################模板点胶轨迹标注#################################################
    def slotTrajectoryLabeling(self):
        if self.windowTrajectorySetting is not None:
            if not self.windowTrajectorySetting.isHidden():
                self.parent.textBrowser.append(printLog("窗口已打开"))
                return

        self.parent.pointCloud.__init__()  # pointcloud初始化为none

        self.windowTrajectorySetting = MdiSubWindow()
        self.windowTrajectorySetting.setWindowTitle('点胶轨迹标注')
        self.windowTrajectorySetting.setFixedSize(1100, 550)

        self.windowTrajectorySetting.setWindowFlags(QtCore.Qt.WindowCloseButtonHint)



        self.dWidget = DrawingWidget(self.parent)
        self.dWidget.setFixedSize(920, 450)
        self.buttonSubOne = QtWidgets.QPushButton('-1(A)')
        self.buttonSubOne.setShortcut('A')
        self.buttonAddOne = QtWidgets.QPushButton('+1(S)')
        self.buttonAddOne.setShortcut('S')
        self.buttonSubTen = QtWidgets.QPushButton('-10(Z)')
        self.buttonSubTen.setShortcut('Z')
        self.buttonAddTen = QtWidgets.QPushButton('+10(X)')
        self.buttonAddTen.setShortcut('X')
        self.buttonSubHundred = QtWidgets.QPushButton('-100(Q)')
        self.buttonSubHundred.setShortcut('Q')
        self.buttonAddHundred = QtWidgets.QPushButton('+100(W)')
        self.buttonAddHundred.setShortcut('W')
        self.checkAssist=QtWidgets.QCheckBox('辅助标记模式')
        self.lblLabelingStep=QtWidgets.QLabel('连续标记长度')
        self.cmbLabelingStep=QtWidgets.QComboBox()
        self.cmbLabelingStep.addItem("1")
        self.cmbLabelingStep.addItem("10")
        self.cmbLabelingStep.addItem("100")
        self.cmbLabelingStep.addItem("200")
        self.cmbLabelingStep.addItem("500")
        self.cmbLabelingStep.addItem("all")

        hLayout1 = QtWidgets.QHBoxLayout()
        vLayout1_2 = QtWidgets.QVBoxLayout()

        hLayout1_2_1 = QtWidgets.QHBoxLayout()
        hLayout1_2_2 = QtWidgets.QHBoxLayout()
        hLayout1_2_3 = QtWidgets.QHBoxLayout()
        vLayout1_2_4 = QtWidgets.QVBoxLayout()

        vLayout1_2.addStretch(20)
        hLayout1_2_1.addWidget(self.buttonSubOne)
        hLayout1_2_1.addWidget(self.buttonAddOne)
        hLayout1_2_2.addWidget(self.buttonSubTen)
        hLayout1_2_2.addWidget(self.buttonAddTen)
        hLayout1_2_3.addWidget(self.buttonSubHundred)
        hLayout1_2_3.addWidget(self.buttonAddHundred)
        vLayout1_2_4.addWidget(self.lblLabelingStep , alignment=QtCore.Qt.AlignHCenter)
        vLayout1_2_4.addWidget(self.cmbLabelingStep)#, alignment=QtCore.Qt.AlignHCenter)

        vLayout1_2.addLayout(hLayout1_2_1)
        vLayout1_2.addStretch(4)
        vLayout1_2.addLayout(hLayout1_2_2)
        vLayout1_2.addStretch(4)
        vLayout1_2.addLayout(hLayout1_2_3)
        vLayout1_2.addStretch(4)
        vLayout1_2.addLayout(vLayout1_2_4)
        vLayout1_2.addStretch(100)
        hLayout1.addWidget(self.dWidget)
        hLayout1.addLayout(vLayout1_2)

        self.buttonAddOne.clicked.connect(lambda:self._lineNoAdd(1))
        self.buttonAddTen.clicked.connect(lambda:self._lineNoAdd(10))
        self.buttonAddHundred.clicked.connect(lambda:self._lineNoAdd(100))
        self.buttonSubOne.clicked.connect(lambda:self._lineNoAdd(-1))
        self.buttonSubTen.clicked.connect(lambda:self._lineNoAdd(-10))
        self.buttonSubHundred.clicked.connect(lambda:self._lineNoAdd(-100))
        #self.checkAssist.clicked.connect(self._setIsAssist)
        self.cmbLabelingStep.activated[str].connect(self._setLabelStep)



        self.buttonLoadData = QtWidgets.QPushButton('加载数据')
        self.buttonLoadTrajectory = QtWidgets.QPushButton('加载轨迹')
        self.button3DShow = QtWidgets.QPushButton('3D显示')
        self.buttonSaveTrajectory = QtWidgets.QPushButton('存储轨迹')

        hLayout2 = QtWidgets.QHBoxLayout()
        hLayout2.addStretch(100)
        hLayout2.addWidget(self.buttonLoadData)
        hLayout2.addStretch(10)
        hLayout2.addWidget(self.buttonLoadTrajectory)
        hLayout2.addStretch(10)
        hLayout2.addWidget(self.button3DShow)
        hLayout2.addStretch(10)
        hLayout2.addWidget(self.buttonSaveTrajectory)
        hLayout2.addStretch(100)

        self.buttonLoadData.clicked.connect(self._loadData)
        self.buttonLoadTrajectory.clicked.connect(self._loadTrajectory)
        self.button3DShow.clicked.connect(self._show3d)
        self.buttonSaveTrajectory.clicked.connect(self._storeTrajectory)

        widget = QtWidgets.QWidget()
        vLayout = QtWidgets.QVBoxLayout()
        vLayout.addLayout(hLayout1)
        vLayout.addLayout(hLayout2)
        vLayout.addStretch(100)

        widget.setLayout(vLayout)
        self.windowTrajectorySetting.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowTrajectorySetting)
        self.windowTrajectorySetting.show()
        self.dWidget.update()

    def _setLabelStep(self, value):
        self.parent.settings.labelStep=value

        self.parent.textBrowser.append(printLog('【轨迹标注】设置连续标记长度为: '+value))
        logging.getLogger('mainLog').info('【轨迹标注】设置连续标记长度为: '+value)


    # def _setIsAssist(self):
    #     if self.checkAssist.isChecked():
    #         self.dWidget.isAssistedLabel=True
    #         self.parent.textBrowser.append(printLog('【轨迹标注】开启辅助标记模式.'))
    #         logging.getLogger('mainLog').info('【轨迹标注】开启辅助标记模式.')
    #     else:
    #         self.dWidget.isAssistedLabel=False
    #         self.parent.textBrowser.append(printLog('【轨迹标注】关闭辅助标记模式.'))
    #         logging.getLogger('mainLog').info('【轨迹标注】关闭辅助标记模式.')


    def _loadData(self):
        filePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTrajectorySetting, "Open File", "",
                                                            "All Files (*);;Text Files (*.txt)")
        if filePath:
            self.parent.pointCloud.readIndexicalFromIjkxyzTxt(filePath)

            self.parent.pointCloud.lineNum = len(self.parent.pointCloud.indexicalCloud)

            for i, key in enumerate(self.parent.pointCloud.indexicalCloud.keys()):
                if len(self.parent.pointCloud.indexicalCloud[key]) > 1:
                    self.parent.pointCloud.currentLinePoints.lineNo = i
                    self.parent.pointCloud.currentLinePoints.lineKey = key
                    self.parent.pointCloud.currentLinePoints.lineData = self.parent.pointCloud.indexicalCloud[key]
                    break

            self.dWidget.paintMode = 'DrawLine'
            if self.parent.pointCloud.zMax is not None:
                self.dWidget.yMin = self.parent.pointCloud.zMin
            if self.parent.pointCloud.zMin is not None:
                self.dWidget.yMax = self.parent.pointCloud.zMax
            self.parent.textBrowser.append(printLog('【轨迹标注】读取点云成功: ' + filePath + '.'))
            logging.getLogger('mainLog').info('【轨迹标注】读取点云成功:： ' + filePath + ' .')

    def _storeTrajectory(self):
        if self.parent.pointCloud.lineNum is None:
            QtWidgets.QMessageBox.warning(self.windowTrajectorySetting, "警告", "请先加载手机边框点云！")
            return
        file_path, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTrajectorySetting, "存储点云轨迹数据", "",
                                                             "Text Files (*.txt)")
        if file_path == '':
            return
        self.parent.textBrowser.append(printLog("【轨迹标注】边框点云轨迹保存中..."))
        logging.getLogger('mainLog').info('【轨迹标注】边框点云轨迹保存中...')

        trajectoryPoints = []
        with open(file_path, 'w') as f:
            for key in self.parent.pointCloud.indexicalCloud:
                if "labeled" in self.parent.pointCloud.indexicalCloud[key]:
                    temp = np.expand_dims(self.parent.pointCloud.indexicalCloud[key]["labeled"], axis=0)
                    np.savetxt(f, temp, delimiter=";", fmt='%.5f')
                    trajectoryPoints.append(temp[0, 3:])

        # save ply
        save_path = file_path.split('.')[0]+".ply"
        trajectoryPoints = np.array(trajectoryPoints)
        pcdTraj = o3d.geometry.PointCloud()
        pcdTraj.points = o3d.utility.Vector3dVector(trajectoryPoints)
        o3d.io.write_point_cloud(save_path, pcdTraj)

        self.parent.textBrowser.append(printLog('【轨迹标注】边框点云轨迹文件存储（ijkxyz格式）于： '+file_path+'.' ))
        self.parent.textBrowser.append(printLog('【轨迹标注】边框点云轨迹文件存储（xyz格式）于： ' + save_path+'.'))
        logging.getLogger('mainLog').info('【轨迹标注】边框点云轨迹文件存储（ijkxyz格式）于： '+file_path+'.')
        logging.getLogger('mainLog').info('【轨迹标注】边框点云轨迹文件存储（xyz格式）于： ' + save_path+'.')

    def _loadTrajectory(self):
        if self.parent.pointCloud.lineNum is None:
            QtWidgets.QMessageBox.warning(self.windowTrajectorySetting, "警告", "请先加载手机边框点云！")
            return
        filePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTrajectorySetting, "加载点云轨迹数据", "",
                                                             "Text Files (*.txt)")
        if filePath == '':
            return

        with open(filePath, 'r') as f:
            lines=np.loadtxt(f,delimiter=';')
            for line in lines:
                key=str(int(line[0]))
                self.parent.pointCloud.indexicalCloud[key]['labeled']=line



            # for key in self.parent.pointCloud.indexicalCloud:
            #     if "labeled" in self.parent.pointCloud.indexicalCloud[key]:
            #         temp = np.expand_dims(self.parent.pointCloud.indexicalCloud[key]["labeled"], axis=0)
            #         np.savetxt(f, temp, delimiter=";", fmt='%.5f')
            #         trajectoryPoints.append(temp[0, 3:])

        self.parent.textBrowser.append(printLog('【轨迹标注】加载边框点云轨迹文件： ' + filePath + '.'))
        logging.getLogger('mainLog').info('【轨迹标注】加载边框点云轨迹文件： ' + filePath + '.')

    def _show3d(self):
        if self.parent.pointCloud.cloudData is not None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(self.parent.pointCloud.cloudData)
            pcd = pcd.voxel_down_sample(0.15)
            pcd.paint_uniform_color([1, 0, 0])

            trajectoryPoints = []
            for key in self.parent.pointCloud.indexicalCloud:
                if "labeled" in self.parent.pointCloud.indexicalCloud[key]:
                    temp = np.expand_dims(self.parent.pointCloud.indexicalCloud[key]["labeled"], axis=0)[0, 3:]
                    trajectoryPoints.append(temp)

            if len(trajectoryPoints) > 0:
                trajectoryPoints = np.array(trajectoryPoints)
                pcdTraj = o3d.geometry.PointCloud()
                pcdTraj.points = o3d.utility.Vector3dVector(trajectoryPoints)
                pcdTraj.paint_uniform_color([0, 0, 1])

                allPcd = [pcd, pcdTraj]
            else:
                allPcd = [pcd]

            o3d.visualization.draw_geometries(allPcd, window_name='手机边框点云', width=800, height=600, top=200, left=100)

    def _lineNoAdd(self, step):
        if self.parent.pointCloud.lineNum is not None:
            self.parent.pointCloud.currentLinePoints.lineNo = self.parent.pointCloud.currentLinePoints.lineNo + step
            # 如果不在范围内，截断在[0, lineNum - 1]
            if self.parent.pointCloud.currentLinePoints.lineNo not in range(self.parent.pointCloud.lineNum):
                self.parent.pointCloud.currentLinePoints.lineNo = \
                    np.clip(np.array([self.parent.pointCloud.currentLinePoints.lineNo]), 0,
                            self.parent.pointCloud.lineNum - 1)[0]

            self.parent.pointCloud.currentLinePoints.lineKey = list(self.parent.pointCloud.indexicalCloud.keys())[
                self.parent.pointCloud.currentLinePoints.lineNo]

            self.parent.pointCloud.currentLinePoints.lineData = self.parent.pointCloud.indexicalCloud[
                self.parent.pointCloud.currentLinePoints.lineKey]

            self.dWidget.update()


