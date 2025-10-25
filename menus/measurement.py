from utils.utils import timeNow2Str, printLog
from mdiSubWindow import MdiSubWindow
from PyQt5 import QtWidgets, QtCore, QtGui
import logging
import ctypes
import open3d as o3d
from dataStructure import DeformationGraph
from autoDispenserDll import generateTrajectoryDll, generateTrajectoryDll2,loadTemplateMeshDll, readO3dFromFileDll
import time


logger = logging.getLogger('mainLog')


class ActionMeasurement():
    def __init__(self, parent=None):
        self.parent = parent

    #################模板文件路径设置#################################################
    def slotTemplatePathSetting(self):
        self.windowTemplatePathSetting = MdiSubWindow()
        self.windowTemplatePathSetting.setWindowTitle('模板文件路径设置')

        widget = QtWidgets.QWidget()
        vLayout = QtWidgets.QVBoxLayout()

        hLayout1 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateMeshPath = QtWidgets.QPushButton('设置模板Mesh数据文件路径')
        self.buttonSetTemplateMeshPath.clicked.connect(lambda: self._setTemplateFilesPath('模板Mesh'))
        self.labelTemplateMeshPath = QtWidgets.QLabel()
        hLayout1.addWidget(self.buttonSetTemplateMeshPath)
        hLayout1.addWidget(self.labelTemplateMeshPath)
        hLayout1.addStretch(100)

        hLayout2 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateTracjectoryPath = QtWidgets.QPushButton('设置' + '模板轨迹' + '数据文件路径')
        self.buttonSetTemplateTracjectoryPath.clicked.connect(lambda: self._setTemplateFilesPath('模板轨迹'))
        self.labelTemplateTracjectoryPath = QtWidgets.QLabel()
        hLayout2.addWidget(self.buttonSetTemplateTracjectoryPath)
        hLayout2.addWidget(self.labelTemplateTracjectoryPath)
        hLayout2.addStretch(100)

        hLayout3 = QtWidgets.QHBoxLayout()
        self.buttonSetTrajectoryIndicesPath = QtWidgets.QPushButton('设置' + '轨迹索引' + '数据文件路径')
        self.buttonSetTrajectoryIndicesPath.clicked.connect(lambda: self._setTemplateFilesPath('轨迹索引'))
        self.labelTrajectoryIndicesPath = QtWidgets.QLabel()
        hLayout3.addWidget(self.buttonSetTrajectoryIndicesPath)
        hLayout3.addWidget(self.labelTrajectoryIndicesPath)
        hLayout3.addStretch(100)

        hLayout4 = QtWidgets.QHBoxLayout()
        self.buttonSetDeformationGraphPath = QtWidgets.QPushButton('设置' + '变形图' + '数据文件路径')
        self.buttonSetDeformationGraphPath.clicked.connect(lambda: self._setTemplateFilesPath('变形图'))
        self.labelDeformationGraphPath = QtWidgets.QLabel()
        hLayout4.addWidget(self.buttonSetDeformationGraphPath)
        hLayout4.addWidget(self.labelDeformationGraphPath)
        hLayout4.addStretch(100)

        hLayout5 = QtWidgets.QHBoxLayout()
        self.buttonSetTransMatPath = QtWidgets.QPushButton('设置' + '拼接矩阵' + '数据文件路径')
        self.buttonSetTransMatPath.clicked.connect(lambda: self._setTemplateFilesPath('拼接矩阵'))
        self.labelTransMatPath = QtWidgets.QLabel()
        hLayout5.addWidget(self.buttonSetTransMatPath)
        hLayout5.addWidget(self.labelTransMatPath)
        hLayout5.addStretch(100)

        hLayout6 = QtWidgets.QHBoxLayout()
        self.buttonSetTargetShort1Path = QtWidgets.QPushButton('设置' + '目标点云短边1' + '数据文件路径')
        self.buttonSetTargetShort1Path.clicked.connect(lambda: self._setTemplateFilesPath('目标点云短边1'))
        self.labelTargetShort1Path = QtWidgets.QLabel()
        hLayout6.addWidget(self.buttonSetTargetShort1Path)
        hLayout6.addWidget(self.labelTargetShort1Path)
        hLayout6.addStretch(100)

        hLayout7 = QtWidgets.QHBoxLayout()
        self.buttonSetTargetLong2Path = QtWidgets.QPushButton('设置' + '目标点云长边2' + '数据文件路径')
        self.buttonSetTargetLong2Path.clicked.connect(lambda: self._setTemplateFilesPath('目标点云长边2'))
        self.labelTargetLong2Path = QtWidgets.QLabel()
        hLayout7.addWidget(self.buttonSetTargetLong2Path)
        hLayout7.addWidget(self.labelTargetLong2Path)
        hLayout7.addStretch(100)

        hLayout8 = QtWidgets.QHBoxLayout()
        self.buttonSetTargetShort3Path = QtWidgets.QPushButton('设置' + '目标点云短边3' + '数据文件路径')
        self.buttonSetTargetShort3Path.clicked.connect(lambda: self._setTemplateFilesPath('目标点云短边3'))
        self.labelTargetShort3Path = QtWidgets.QLabel()
        hLayout8.addWidget(self.buttonSetTargetShort3Path)
        hLayout8.addWidget(self.labelTargetShort3Path)
        hLayout8.addStretch(100)

        hLayout9 = QtWidgets.QHBoxLayout()
        self.buttonSetTargetLong4Path = QtWidgets.QPushButton('设置' + '目标点云长边4' + '数据文件路径')
        self.buttonSetTargetLong4Path.clicked.connect(lambda: self._setTemplateFilesPath('目标点云长边4'))
        self.labelTargetLong4Path = QtWidgets.QLabel()
        hLayout9.addWidget(self.buttonSetTargetLong4Path)
        hLayout9.addWidget(self.labelTargetLong4Path)
        hLayout9.addStretch(100)

        hLayout10 = QtWidgets.QHBoxLayout()
        self.buttonSetTargetMeshPath = QtWidgets.QPushButton('设置' + '目标Mesh' + '数据存储文件路径')
        self.buttonSetTargetMeshPath.clicked.connect(lambda: self._setTemplateFilesPath('目标Mesh'))
        self.labelTargetMeshPath = QtWidgets.QLabel()
        hLayout10.addWidget(self.buttonSetTargetMeshPath)
        hLayout10.addWidget(self.labelTargetMeshPath)
        hLayout10.addStretch(100)
        vLayout.addLayout(hLayout1)
        vLayout.addLayout(hLayout2)
        vLayout.addLayout(hLayout3)
        vLayout.addLayout(hLayout4)
        vLayout.addLayout(hLayout5)
        vLayout.addLayout(hLayout6)
        vLayout.addLayout(hLayout7)
        vLayout.addLayout(hLayout8)
        vLayout.addLayout(hLayout9)
        vLayout.addLayout(hLayout10)
        vLayout.addStretch(100)

        widget.setLayout(vLayout)
        self.windowTemplatePathSetting.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowTemplatePathSetting)
        self.windowTemplatePathSetting.show()

    def _setTemplateFilesPath(self, mode):
        if mode == '目标Mesh':
            filePath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplatePathSetting, '设置' + mode + '存储文件路径',
                                                                "",
                                                                "All Files (*);;Text Files (*.ply)")
            self.parent.settings.targetMeshPath = filePath
            self.labelTargetMeshPath.setText(filePath)
            return

        filePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplatePathSetting, '设置' + mode + '文件路径', "",
                                                            "All Files (*);;Text Files (*.ply)")
        if mode == '模板Mesh':
            self.parent.settings.templateMeshPath = filePath
            self.labelTemplateMeshPath.setText(filePath)

        elif mode == '模板轨迹':
            self.parent.settings.templateTrajectoryPath = filePath
            self.labelTemplateTracjectoryPath.setText(filePath)

        elif mode == '轨迹索引':
            self.parent.settings.trajectoryIndicesPath = filePath
            self.labelTrajectoryIndicesPath.setText(filePath)

        elif mode == '变形图':
            self.parent.settings.deformationGraphPath = filePath
            self.labelDeformationGraphPath.setText(filePath)

        elif mode == '拼接矩阵':
            self.parent.settings.transMatPath = filePath
            self.labelTransMatPath.setText(filePath)

        elif mode == '目标点云短边1':
            self.parent.settings.targetShort1Path = filePath
            self.labelTargetShort1Path.setText(filePath)

        elif mode == '目标点云长边2':
            self.parent.settings.targetLong2Path = filePath
            self.labelTargetLong2Path.setText(filePath)

        elif mode == '目标点云短边3':
            self.parent.settings.targetShort3Path = filePath
            self.labelTargetShort3Path.setText(filePath)

        elif mode == '目标点云长边4':
            self.parent.settings.targetLong4Path = filePath
            self.labelTargetLong4Path.setText(filePath)



        self.parent.textBrowser.append(printLog('设置' + mode + '点云路径为: ' + filePath + '.'))
        logging.getLogger('mainLog').info('设置' + mode + '点云路径为: ' + filePath + '.')

    #################模板文件加载#################################################
    def slotLoadTemplateFiles(self):
        if self.parent.settings.templateMeshPath == '' or self.parent.settings.templateMeshPath is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置模板Mesh文件路径！")
            return

        if self.parent.settings.templateTrajectoryPath == '' or self.parent.settings.templateTrajectoryPath is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置模板轨迹文件路径！")
            return

        if self.parent.settings.trajectoryIndicesPath == '' or self.parent.settings.trajectoryIndicesPath is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置轨迹索引文件路径！")
            return

        if self.parent.settings.deformationGraphPath == '' or self.parent.settings.deformationGraphPath is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置模板变形图文件路径！")
            return

        if self.parent.settings.transMatPath == '' or self.parent.settings.transMatPath is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置拼接矩阵文件路径！")
            return

        if self.parent.settings.targetShort1Path == '' or self.parent.settings.targetShort1Path is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置目标点云短边1文件路径！")
            return

        if self.parent.settings.targetLong2Path == '' or self.parent.settings.targetLong2Path is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置目标点云长边2文件路径！")
            return

        if self.parent.settings.targetShort3Path == '' or self.parent.settings.targetShort3Path is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置目标点云短边3文件路径！")
            return

        if self.parent.settings.targetLong4Path == '' or self.parent.settings.targetLong4Path is None:
            QtWidgets.QMessageBox.warning(self.parent, "警告", "请先设置目标点云长边4文件路径！")
            return

        self.parent.deformationGraphDll = DeformationGraph()
        self.parent.deformationGraphDll.readData(self.parent.settings.deformationGraphPath)
        self.parent.deformationGraphDll.loadToDll()

        self.parent.textBrowser.append(printLog('读取变形图文件成功：' + self.parent.settings.deformationGraphPath + '.'))
        logging.getLogger('mainLog').info('读取变形图文件成功：' + self.parent.settings.deformationGraphPath + '.')

        templateMeshPath = ctypes.create_string_buffer(self.parent.settings.templateMeshPath.encode('gbk'))
        self.parent.templateMeshPtrDll = loadTemplateMeshDll(templateMeshPath,
                                                             self.parent.deformationGraphDll.scaleNorm)
        self.parent.textBrowser.append(printLog('加载模板Mesh文件成功：' + self.parent.settings.templateMeshPath + '.'))
        logging.getLogger('mainLog').info('加载模板Mesh文件成功：' + self.parent.settings.templateMeshPath + '.')

        self._loadTargetFromFiles()

        return

    def _loadTargetFromFiles(self):
        targetShort1Path = ctypes.create_string_buffer(self.parent.settings.targetShort1Path.encode('gbk'))
        self.parent.targetShort1PtrDll = readO3dFromFileDll(targetShort1Path)
        self.parent.textBrowser.append(printLog('加载工件短边1点云文件成功：' + self.parent.settings.targetShort1Path + '.'))
        logging.getLogger('mainLog').info('加载工件短边1点云文件成功：' + self.parent.settings.targetShort1Path + '.')

        targetLong2Path = ctypes.create_string_buffer(self.parent.settings.targetLong2Path.encode('gbk'))
        self.parent.targetLong2PtrDll = readO3dFromFileDll(targetLong2Path)
        self.parent.textBrowser.append(printLog('加载工件长边2点云文件成功：' + self.parent.settings.targetLong2Path + '.'))
        logging.getLogger('mainLog').info('加载工件长边2点云文件成功：' + self.parent.settings.targetLong2Path + '.')

        targetShort3Path = ctypes.create_string_buffer(self.parent.settings.targetShort3Path.encode('gbk'))
        self.parent.targetShort3PtrDll = readO3dFromFileDll(targetShort3Path)
        self.parent.textBrowser.append(printLog('加载工件短边3点云文件成功：' + self.parent.settings.targetShort3Path + '.'))
        logging.getLogger('mainLog').info('加载工件短边3点云文件成功：' + self.parent.settings.targetShort3Path + '.')

        targetLong4Path = ctypes.create_string_buffer(self.parent.settings.targetLong4Path.encode('gbk'))
        self.parent.targetLong4PtrDll = readO3dFromFileDll(targetLong4Path)
        self.parent.textBrowser.append(printLog('加载工件长边4点云文件成功：' + self.parent.settings.targetLong4Path + '.'))
        logging.getLogger('mainLog').info('加载工件长边4点云文件成功：' + self.parent.settings.targetLong4Path + '.')

    #################目标工件轨迹测量#################################################
    def slotGetTrajectory(self):
        self.windowGetTrajectory = MdiSubWindow()
        self.windowGetTrajectory.setWindowTitle('工件轨迹测量')

        widget = QtWidgets.QWidget()
        vLayout = QtWidgets.QVBoxLayout()

        hLayout1 = QtWidgets.QHBoxLayout()
        hLayout1.addStretch(100)
        self.buttonTrajectoryMeasurement = QtWidgets.QPushButton('轨迹测量')
        self.buttonTrajectoryMeasurement.clicked.connect(self._trajectoryMeasurement)
        self.buttonShow3D = QtWidgets.QPushButton('3D显示')
        self.buttonShow3D.clicked.connect(self._show3D)
        hLayout1.addWidget(self.buttonTrajectoryMeasurement)
        hLayout1.addWidget(self.buttonShow3D)
        hLayout1.addStretch(100)

        vLayout.addLayout(hLayout1)
        vLayout.addStretch(100)

        widget.setLayout(vLayout)
        self.windowGetTrajectory.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowGetTrajectory)
        self.windowGetTrajectory.show()

    def _show3D(self):
        trajectoryResult=o3d.io.read_point_cloud(self.parent.settings.trajectoryResultPath.encode('gbk'))
        targetMesh=o3d.io.read_point_cloud(self.parent.settings.targetMeshPath.encode('gbk'))
        o3d.visualization.draw_geometries([trajectoryResult,targetMesh], point_show_normal=True,window_name='工件轨迹', width=800, height=600, top=200, left=100)

        pass

    def _trajectoryMeasurement(self):

        targetMeshPath=ctypes.create_string_buffer(self.parent.settings.targetMeshPath.encode('gbk'))
        transMatPath = ctypes.create_string_buffer(self.parent.settings.transMatPath.encode('gbk'))
        trajPath = ctypes.create_string_buffer(self.parent.settings.templateTrajectoryPath.encode('gbk'))
        trajIndicesPath = ctypes.create_string_buffer(self.parent.settings.trajectoryIndicesPath.encode('gbk'))
        trajSavePath = ctypes.create_string_buffer(self.parent.settings.trajectoryResultPath.encode('gbk'))

        start=time.time()
        generateTrajectoryDll2(self.parent.deformationGraphDll.deformationGraphDll, self.parent.templateMeshPtrDll,
                               self.parent.targetShort1PtrDll, self.parent.targetLong2PtrDll,
                               self.parent.targetShort3PtrDll, self.parent.targetLong4PtrDll, transMatPath, trajPath,
                               trajIndicesPath, trajSavePath,targetMeshPath)
        end=time.time()
        self.parent.textBrowser.append(printLog('工件轨迹测量总耗时：' + str(end-start) + '秒.'))
        logging.getLogger('mainLog').info('工件轨迹测量总耗时：' + str(end-start) + '秒.')

        return
