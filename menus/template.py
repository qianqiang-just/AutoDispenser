from utils.utils import timeNow2Str, printLog
from mdiSubWindow import MdiSubWindow
from PyQt5 import QtWidgets, QtCore, QtGui
import ctypes
import open3d as o3d
import numpy as np
import logging
from autoDispenserDll import getAlignTransMatriceDll, getTemplateFilesDll



logger = logging.getLogger('mainLog')


class ActionTemplate():

    def __init__(self, parent=None):
        self.parent = parent
        self.isShowTemplateInTemplateFileBuild=False
        self.isShowTrajectoryInTemplateFileBuild = False
        self.isShowDeformationGraphInTemplateFileBuild = False

    #################边框点云路径设置#################################################
    def slotBorderPathSetting(self):
        self.windowBorderPathSetting = MdiSubWindow()
        self.windowBorderPathSetting.setWindowTitle('边框点云路径设置')

        widget = QtWidgets.QWidget()
        vLayout = QtWidgets.QVBoxLayout()

        hLayout1 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateShort1Path = QtWidgets.QPushButton('设置短边1数据文件路径')
        self.buttonSetTemplateShort1Path.clicked.connect(lambda: self._setTemplateEdgeCloudPath('短边1'))
        self.labelTemplateShort1Path = QtWidgets.QLabel()
        hLayout1.addWidget(self.buttonSetTemplateShort1Path)
        hLayout1.addWidget(self.labelTemplateShort1Path)
        hLayout1.addStretch(100)

        hLayout2 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateLong2Path = QtWidgets.QPushButton('设置' + '长边2' + '数据文件路径')
        self.buttonSetTemplateLong2Path.clicked.connect(lambda: self._setTemplateEdgeCloudPath('长边2'))
        self.labelTemplateLong2Path = QtWidgets.QLabel()
        hLayout2.addWidget(self.buttonSetTemplateLong2Path)
        hLayout2.addWidget(self.labelTemplateLong2Path)
        hLayout2.addStretch(100)

        hLayout3 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateShort3Path = QtWidgets.QPushButton('设置' + '短边3' + '数据文件路径')
        self.buttonSetTemplateShort3Path.clicked.connect(lambda: self._setTemplateEdgeCloudPath('短边3'))
        self.labelTemplateShort3Path = QtWidgets.QLabel()
        hLayout3.addWidget(self.buttonSetTemplateShort3Path)
        hLayout3.addWidget(self.labelTemplateShort3Path)
        hLayout3.addStretch(100)

        hLayout4 = QtWidgets.QHBoxLayout()
        self.buttonSetTemplateLong4Path = QtWidgets.QPushButton('设置' + '长边4' + '数据文件路径')
        self.buttonSetTemplateLong4Path.clicked.connect(lambda: self._setTemplateEdgeCloudPath('长边4'))
        self.labelTemplateLong4Path = QtWidgets.QLabel()
        hLayout4.addWidget(self.buttonSetTemplateLong4Path)
        hLayout4.addWidget(self.labelTemplateLong4Path)
        hLayout4.addStretch(100)

        vLayout.addLayout(hLayout1)
        vLayout.addLayout(hLayout2)
        vLayout.addLayout(hLayout3)
        vLayout.addLayout(hLayout4)
        vLayout.addStretch(100)

        widget.setLayout(vLayout)
        self.windowBorderPathSetting.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowBorderPathSetting)
        self.windowBorderPathSetting.show()

    def _setTemplateEdgeCloudPath(self, edgeName):
        filePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowBorderPathSetting, '设置' + edgeName + '点云路径', "",
                                                            "All Files (*);;Text Files (*.ply)")
        if edgeName == '短边1':
            self.parent.settings.templateShort1Path = filePath
            self.labelTemplateShort1Path.setText(filePath)
        elif edgeName == '长边2':
            self.parent.settings.templateLong2Path = filePath
            self.labelTemplateLong2Path.setText(filePath)
        elif edgeName == '短边3':
            self.parent.settings.templateShort3Path = filePath
            self.labelTemplateShort3Path.setText(filePath)
        elif edgeName == '长边4':
            self.parent.settings.templateLong4Path = filePath
            self.labelTemplateLong4Path.setText(filePath)

        self.parent.textBrowser.append(printLog('设置' + edgeName + '点云路径为: ' + filePath + '.'))
        logging.getLogger('mainLog').info('设置' + edgeName + '点云路径为: ' + filePath + '.')

    #################边框点云拼接#################################################
    def slotBorderAlignment(self):
        self.windowBorderAlignment = MdiSubWindow()
        self.windowBorderAlignment.setWindowTitle('边框点云拼接')

        widget = QtWidgets.QWidget()
        widget.setMinimumWidth(400)
        widget.setMinimumHeight(400)
        vLayout = QtWidgets.QVBoxLayout()

        self.buttonBorderAlignment = QtWidgets.QPushButton('点云拼接')
        self.buttonBorderAlignment.clicked.connect(self._borderAlignment)
        self.button3DShowInBorderAlignment = QtWidgets.QPushButton('3D显示')
        self.button3DShowInBorderAlignment.clicked.connect(self._show3dInBorderAlignment)
        self.buttonSaveMatrix = QtWidgets.QPushButton('拼接矩阵存储')
        self.buttonSaveMatrix.clicked.connect(self._saveMatrix)
        self.buttonSaveCloud = QtWidgets.QPushButton('拼接点云存储')
        self.buttonSaveCloud.clicked.connect(self._saveCloud)

        hLayout1 = QtWidgets.QHBoxLayout()
        hLayout1.addStretch(100)
        hLayout1.addWidget(self.buttonBorderAlignment)
        hLayout1.addWidget(self.button3DShowInBorderAlignment)
        hLayout1.addWidget(self.buttonSaveMatrix)
        hLayout1.addWidget(self.buttonSaveCloud)
        hLayout1.addStretch(100)

        self.labelLoadingInBorderAlignment = QtWidgets.QLabel()
        self.labelLoadingInBorderAlignment.setFixedWidth(210)
        self.labelLoadingInBorderAlignment.setFixedHeight(210)
        self.labelLoading2InBorderAlignment = QtWidgets.QLabel()

        vLayout2 = QtWidgets.QVBoxLayout()
        vLayout2.addStretch(100)
        vLayout2.addWidget(self.labelLoading2InBorderAlignment, alignment=QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        vLayout2.addWidget(self.labelLoadingInBorderAlignment, alignment=QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        vLayout2.addStretch(100)

        vLayout.addLayout(hLayout1)
        vLayout.addLayout(vLayout2)
        vLayout.addStretch(100)
        widget.setLayout(vLayout)
        self.windowBorderAlignment.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowBorderAlignment)
        self.windowBorderAlignment.show()

    def _borderAlignment(self):
        if self.parent.settings.templateShort1Path is None:
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "短边1数据文件路径未设置！")
        elif self.parent.settings.templateLong2Path is None:
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "长边2数据文件路径未设置！")
        elif self.parent.settings.templateShort3Path is None:
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "短边3数据文件路径未设置！")
        elif self.parent.settings.templateLong4Path is None:
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "长边4数据文件路径未设置！")
        else:
            self.movieLoadingInBorderAlignment = QtGui.QMovie('imgs/loading.gif')
            pe = QtGui.QPalette()
            pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.blue)  # 设置字体颜色
            self.labelLoading2InBorderAlignment.setPalette(pe)
            self.labelLoading2InBorderAlignment.setText('边框点云拼接中...')
            self.labelLoadingInBorderAlignment.setMovie(self.movieLoadingInBorderAlignment)

            self.movieLoadingInBorderAlignment.start()

            self.borderAlignmentThread = BorderAlignmentThread(self.parent)
            self.borderAlignmentThread.resultSignal.connect(self._stopLoadingInBorderAlignment)
            self.borderAlignmentThread.start()

    def _stopLoadingInBorderAlignment(self, signalValue):
        self.movieLoadingInBorderAlignment.stop()
        self.labelLoadingInBorderAlignment.clear()

        pe = QtGui.QPalette()
        pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.red)  # 设置字体颜色
        self.labelLoading2InBorderAlignment.setPalette(pe)
        self.labelLoading2InBorderAlignment.setText(signalValue)
        if signalValue != '边框点云拼接完成':
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", signalValue)

    def _saveMatrix(self):
        if isinstance(self.parent.settings.templateAlignmentMat1, list):
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "请先进行边框点云拼接计算！")
        else:
            filePath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowBorderAlignment, "存储转换矩阵", "",
                                                                "All Files (*);;Text Files (*.txt)")

            if filePath is not None and filePath != '':
                with open(filePath, 'w') as f:
                    mat1List = self.parent.settings.templateAlignmentMat1.tolist()
                    f.write('short1 transformation matrix:\n')
                    for row in mat1List:
                        for ele in row:
                            f.write(str(ele) + '\t\t')
                        f.write('\n')
                    self.parent.textBrowser.append(printLog('存储短边1转换矩阵成功.'))
                    logging.getLogger('mainLog').info('存储短边1转换矩阵成功.')

                    f.write('\n')
                    mat2List = self.parent.settings.templateAlignmentMat2.tolist()
                    f.write('long2 transformation matrix:\n')
                    for row in mat2List:
                        for ele in row:
                            f.write(str(ele) + '\t\t')
                        f.write('\n')
                    self.parent.textBrowser.append(printLog('存储长边2转换矩阵成功.'))
                    logging.getLogger('mainLog').info('存储长边2转换矩阵成功.')

                    f.write('\n')
                    mat3List = self.parent.settings.templateAlignmentMat3.tolist()
                    f.write('short3 transformation matrix:\n')
                    for row in mat3List:
                        for ele in row:
                            f.write(str(ele) + '\t\t')
                        f.write('\n')
                    self.parent.textBrowser.append(printLog('存储短边3转换矩阵成功.'))
                    logging.getLogger('mainLog').info('存储短边3转换矩阵成功.')

                    f.write('\n')
                    mat4List = self.parent.settings.templateAlignmentMat4.tolist()
                    f.write('long4 transformation matrix:\n')
                    for row in mat4List:
                        for ele in row:
                            f.write(str(ele) + '\t\t')
                        f.write('\n')
                    self.parent.textBrowser.append(printLog('存储长边4转换矩阵成功.'))
                    logging.getLogger('mainLog').info('存储长边4转换矩阵成功.')

                    self.parent.textBrowser.append(printLog('存储转换矩阵成功： ' + filePath + ' .'))
                    logging.getLogger('mainLog').info('存储转换矩阵成功： ' + filePath + ' .')

                pe = QtGui.QPalette()
                pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.magenta)  # 设置字体颜色
                self.labelLoading2InBorderAlignment.setPalette(pe)
                self.labelLoading2InBorderAlignment.setText('点云转换矩阵存储完成')

    def _saveCloud(self):
        if self.parent.templateCloudShort1 is not None:
            self.parent.tempalteCloud = self.parent.templateCloudShort1 + self.parent.templateCloudLong2 + self.parent.templateCloudShort3 + self.parent.templateCloudLong4

        filePath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowBorderAlignment, "设置手机边框点云存储文件路径", "",
                                                            "All Files (*);;PLY Files (*.ply)")
        if filePath is not None and filePath != '':
            o3d.io.write_point_cloud(filePath.encode('gbk'), self.parent.tempalteCloud)

            self.parent.textBrowser.append(printLog('存储拼接后整体点云成功： ' + filePath + ' .'))
            logging.getLogger('mainLog').info('存储拼接后整体点云成功： ' + filePath + ' .')

            pe = QtGui.QPalette()
            pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.green)  # 设置字体颜色
            self.labelLoading2.setPalette(pe)
            self.labelLoading2.setText('拼接后整体点云存储完成')

    def _show3dInBorderAlignment(self):
        if isinstance(self.parent.settings.templateAlignmentMat1, list):
            QtWidgets.QMessageBox.warning(self.windowBorderAlignment, "警告", "请先进行边框点云拼接计算！")
        else:
            pcdShort1 = o3d.io.read_point_cloud(self.parent.settings.templateShort1Path.encode('gbk'))
            pcdLong2 = o3d.io.read_point_cloud(self.parent.settings.templateLong2Path.encode('gbk'))
            pcdShort3 = o3d.io.read_point_cloud(self.parent.settings.templateShort3Path.encode('gbk'))
            pcdLong4 = o3d.io.read_point_cloud(self.parent.settings.templateLong4Path.encode('gbk'))

            self.parent.templateCloudShort1 = pcdShort1.transform(self.parent.settings.templateAlignmentMat1)
            self.parent.templateCloudLong2 = pcdLong2.transform(self.parent.settings.templateAlignmentMat2)
            self.parent.templateCloudShort3 = pcdShort3.transform(self.parent.settings.templateAlignmentMat3)
            self.parent.templateCloudLong4 = pcdLong4.transform(self.parent.settings.templateAlignmentMat4)

            pcdShort1 = self.parent.templateCloudShort1.voxel_down_sample(0.2)
            pcdLong2 = self.parent.templateCloudLong2.voxel_down_sample(0.2)
            pcdShort3 = self.parent.templateCloudShort3.voxel_down_sample(0.2)
            pcdLong4 = self.parent.templateCloudLong4.voxel_down_sample(0.2)

            pcdShort1.paint_uniform_color([1, 0, 0])
            pcdLong2.paint_uniform_color([0, 1, 0])
            pcdShort3.paint_uniform_color([0, 0, 1])
            pcdLong4.paint_uniform_color([0.2, 0.5, 0.8])
            allPcd = [pcdShort1, pcdLong2, pcdShort3, pcdLong4]

            o3d.visualization.draw_geometries(allPcd, window_name='手机边框点云', width=800, height=600, top=200, left=100)

    #################模板文件生成#################################################
    def slotTemplateFileBuild(self):
        self.windowTemplateFileBuild = MdiSubWindow()
        self.windowTemplateFileBuild.setWindowTitle('模板文件生成')

        widget = QtWidgets.QWidget()
        widget.setMinimumWidth(400)
        widget.setMinimumHeight(400)
        vLayout = QtWidgets.QVBoxLayout()

        self.buttonTransMatriceLoad = QtWidgets.QPushButton('拼接矩阵导入')
        self.buttonTransMatriceLoad.clicked.connect(self._transMatriceLoad)
        self.buttonTemplateCompose = QtWidgets.QPushButton('模板点云合成')
        self.buttonTemplateCompose.clicked.connect(self._templateCompose)
        self.buttonTrajCompose = QtWidgets.QPushButton('模板点胶轨迹合成')
        self.buttonTrajCompose.clicked.connect(self._trajCompose)
        self.buttonDeformationGraphBuild = QtWidgets.QPushButton('模板变形图生成')
        self.buttonDeformationGraphBuild.clicked.connect(self._deformationGraphBuild)
        self.button3DShowInTemplateFileBuild = QtWidgets.QPushButton('3D显示')
        self.button3DShowInTemplateFileBuild.clicked.connect(self._show3dInTemplateFileBuild)

        hLayout1 = QtWidgets.QHBoxLayout()
        hLayout1.addStretch(100)
        hLayout1.addWidget(self.buttonTransMatriceLoad)
        hLayout1.addWidget(self.buttonTemplateCompose)
        hLayout1.addWidget(self.buttonTrajCompose)
        hLayout1.addWidget(self.buttonDeformationGraphBuild)
        hLayout1.addWidget(self.button3DShowInTemplateFileBuild)
        hLayout1.addStretch(100)

        self.labelLoadingInTemplateFileBuild = QtWidgets.QLabel()
        self.labelLoadingInTemplateFileBuild.setFixedWidth(210)
        self.labelLoadingInTemplateFileBuild.setFixedHeight(210)
        self.labelLoading2InTemplateFileBuild = QtWidgets.QLabel()

        self.checkTemplate=QtWidgets.QCheckBox('显示模板')
        self.checkTemplate.stateChanged.connect(self._checkboxTemplateChanged)
        self.checkTraj = QtWidgets.QCheckBox('显示轨迹')
        self.checkTraj.stateChanged.connect(self._checkboxTrajectoryChanged)
        self.checkGraph = QtWidgets.QCheckBox('显示变形图')
        self.checkGraph.stateChanged.connect(self._checkboxDeformationGraphChanged)

        hLayout2 = QtWidgets.QHBoxLayout()
        hLayout2.addStretch(100)
        hLayout2.addWidget(self.checkTemplate)
        hLayout2.addWidget(self.checkTraj)
        hLayout2.addWidget(self.checkGraph)
        hLayout2.addStretch(100)

        vLayout3 = QtWidgets.QVBoxLayout()
        vLayout3.addStretch(100)
        vLayout3.addWidget(self.labelLoading2InTemplateFileBuild, alignment=QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        vLayout3.addWidget(self.labelLoadingInTemplateFileBuild, alignment=QtCore.Qt.AlignHCenter | QtCore.Qt.AlignVCenter)
        vLayout3.addStretch(100)

        vLayout.addLayout(hLayout1)
        vLayout.addLayout(hLayout2)
        vLayout.addLayout(vLayout3)
        vLayout.addStretch(100)
        widget.setLayout(vLayout)
        self.windowTemplateFileBuild.setWidget(widget)
        self.parent.mdiArea.addSubWindow(self.windowTemplateFileBuild)
        self.windowTemplateFileBuild.show()

    def _transMatriceLoad(self):

        filePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入拼接矩阵', "",
                                                            "All Files (*);;Text Files (*.txt)")

        if filePath is not None and filePath != '':
            with open(filePath, 'r') as f:
                lines = f.read().split('\n')
                matrices = [
                    [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
                    [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
                    [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
                    [[0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 0]],
                ]
                index = 0
                row = 0
                for line in lines:
                    if line.startswith('short1'):
                        continue
                    elif line.startswith('long2'):
                        index += 1
                        row = 0
                    elif line.startswith('short3'):
                        index += 1
                        row = 0
                    elif line.startswith('long4'):
                        index += 1
                        row = 0
                    elif line == '':
                        continue
                    else:
                        eles = line.split('\t\t')[0:4]
                        for i, ele in enumerate(eles):
                            matrices[index][row][i] = float(ele)
                        row += 1
                self.parent.settings.templateAlignmentMat1 = np.array(matrices[0])
                self.parent.settings.templateAlignmentMat2 = np.array(matrices[1])
                self.parent.settings.templateAlignmentMat3 = np.array(matrices[2])
                self.parent.settings.templateAlignmentMat4 = np.array(matrices[3])

                self.parent.textBrowser.append(printLog('点云拼接矩阵导入成功： ' + filePath + ' .'))
                logging.getLogger('mainLog').info('点云拼接矩阵导入成功： ' + filePath + ' .')

    def _templateCompose(self):
        eleSum = 0
        for row in self.parent.settings.templateAlignmentMat1:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat2:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat3:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat4:
            for ele in row:
                eleSum += ele

        if eleSum == 0:
            QtWidgets.QMessageBox.warning(self.windowTemplateFileBuild, "警告", "请先导入拼接矩阵或者进行边框点云拼接计算！")
            return

        short1Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入短边1点云', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if short1Path == '':
            return

        long2Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入长边2点云', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if long2Path == '':
            return

        short3Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入短边3点云', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if short3Path == '':
            return

        long4Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入长边4点云', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if long4Path == '':
            return

        self.templateCloudShort1 = o3d.io.read_point_cloud(short1Path.encode('gbk'))
        self.templateCloudLong2 = o3d.io.read_point_cloud(long2Path.encode('gbk'))
        self.templateCloudShort3 = o3d.io.read_point_cloud(short3Path.encode('gbk'))
        self.templateCloudLong4 = o3d.io.read_point_cloud(long4Path.encode('gbk'))

        self.templateCloudShort1 = self.templateCloudShort1.transform(self.parent.settings.templateAlignmentMat1)
        self.templateCloudLong2 = self.templateCloudLong2.transform(self.parent.settings.templateAlignmentMat2)
        self.templateCloudShort3 = self.templateCloudShort3.transform(self.parent.settings.templateAlignmentMat3)
        self.templateCloudLong4 = self.templateCloudLong4.transform(self.parent.settings.templateAlignmentMat4)

        self.parent.tempalteCloud = self.templateCloudShort1 + self.templateCloudLong2 + self.templateCloudShort3 + self.templateCloudLong4

        self.parent.textBrowser.append(printLog('模板点云合成完毕.'))
        logging.getLogger('mainLog').info('模板点云合成完毕.')

        templatePath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplateFileBuild, "设置模板点云存储文件路径", "",
                                                            "All Files (*);;Ply Files (*.ply)")
        if templatePath is not None and templatePath != '':
            o3d.io.write_point_cloud(templatePath.encode('gbk'), self.parent.tempalteCloud)

            self.parent.textBrowser.append(printLog('模板合成点云文件存储于： ' + templatePath + ' .'))
            logging.getLogger('mainLog').info('模板合成点云文件存储于： ' + templatePath + ' .')

    def _trajCompose(self):
        eleSum = 0
        for row in self.parent.settings.templateAlignmentMat1:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat2:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat3:
            for ele in row:
                eleSum += ele
        for row in self.parent.settings.templateAlignmentMat4:
            for ele in row:
                eleSum += ele

        if eleSum == 0:
            QtWidgets.QMessageBox.warning(self.windowTemplateFileBuild, "警告", "请先导入拼接矩阵或者进行边框点云拼接计算！")
            return

        traj1Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入短边1轨迹', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if traj1Path == '':
            return

        traj2Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入长边2轨迹', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if traj2Path == '':
            return

        traj3Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入短边3轨迹', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if traj3Path == '':
            return

        traj4Path, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入长边4轨迹', "",
                                                             "All Files (*);;Ply Files (*.ply)")
        if traj4Path == '':
            return

        traj1 = o3d.io.read_point_cloud(traj1Path.encode('gbk'))
        traj2 = o3d.io.read_point_cloud(traj2Path.encode('gbk'))
        traj3 = o3d.io.read_point_cloud(traj3Path.encode('gbk'))
        traj4 = o3d.io.read_point_cloud(traj4Path.encode('gbk'))

        traj1 = traj1.transform(self.parent.settings.templateAlignmentMat1)
        traj2 = traj2.transform(self.parent.settings.templateAlignmentMat2)
        traj3 = traj3.transform(self.parent.settings.templateAlignmentMat3)
        traj4 = traj4.transform(self.parent.settings.templateAlignmentMat4)

        self.parent.traj = traj1 + traj2 + traj3 + traj4

        self.parent.textBrowser.append(printLog('点胶轨迹合成完毕.'))
        logging.getLogger('mainLog').info('点胶轨迹合成完毕.')

        trajPath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplateFileBuild, "设置点胶轨迹存储文件路径", "",
                                                            "All Files (*);;Ply Files (*.ply)")
        if trajPath is not None and trajPath != '':
            o3d.io.write_point_cloud(trajPath.encode('gbk'), self.parent.traj)

            self.parent.textBrowser.append(printLog('点胶轨迹合成点云文件存储于： ' + trajPath + ' .'))
            logging.getLogger('mainLog').info('点胶轨迹合成点云文件存储于： ' + trajPath + ' .')

    def _deformationGraphBuild(self):

        templateCloudPath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入模板整体点云', "",
                                                                     "All Files (*);;Ply Files (*.ply)")
        if templateCloudPath == '':
            self.parent.textBrowser.append(printLog('【模板生成】模板点云未导入.'))
            logging.getLogger('mainLog').info('【模板生成】模板点云未导入.')
            return

        trajCloudPath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入模板点胶轨迹点云', "",
                                                                 "All Files (*);;Ply Files (*.ply)")
        if trajCloudPath == '':
            self.parent.textBrowser.append(printLog('【模板生成】点胶轨迹点云未导入.'))
            logging.getLogger('mainLog').info('【模板生成】点胶轨迹点云未导入.')
            return

        templateMeshPath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplateFileBuild, "设置模板Mesh存储文件路径", "",
                                                                    "All Files (*);;Ply Files (*.ply)")
        if templateMeshPath == '':
            self.parent.textBrowser.append(printLog('【模板生成】模板Mesh存储文件路径未设置.'))
            logging.getLogger('mainLog').info('【模板生成】模板Mesh存储文件路径未设置.')
            return

        trajIndicesPath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplateFileBuild, "设置点胶轨迹索引存储文件路径", "",
                                                                   "All Files (*);;Text Files (*.txt)")
        if trajIndicesPath == '':
            self.parent.textBrowser.append(printLog('【模板生成】点胶轨迹索引存储文件路径未设置.'))
            logging.getLogger('mainLog').info('【模板生成】点胶轨迹索引存储文件路径未设置.')
            return

        deformationGraphPath, _ = QtWidgets.QFileDialog.getSaveFileName(self.windowTemplateFileBuild, "设置模板变形图存储文件路径",
                                                                        "",
                                                                        "All Files (*);;Text Files (*.txt)")
        if deformationGraphPath == '':
            self.parent.textBrowser.append(printLog('【模板生成】模板变形图存储文件路径未设置.'))
            logging.getLogger('mainLog').info('【模板生成】模板变形图文件路径未设置.')
            return

        self.movieLoadingInTemplateFileBuild = QtGui.QMovie('imgs/loading.gif')
        pe = QtGui.QPalette()
        pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.blue)  # 设置字体颜色
        self.labelLoading2InTemplateFileBuild.setPalette(pe)
        self.labelLoading2InTemplateFileBuild.setText('模板变形图生成中...')
        self.labelLoadingInTemplateFileBuild.setMovie(self.movieLoadingInTemplateFileBuild)

        self.movieLoadingInTemplateFileBuild.start()

        self.templateFilesBuildThread= TemplateFilesBuildThread(self.parent)
        self.templateFilesBuildThread.templateCloudPath=templateCloudPath
        self.templateFilesBuildThread.trajCloudPath = trajCloudPath
        self.templateFilesBuildThread.templateMeshPath = templateMeshPath
        self.templateFilesBuildThread.trajIndicesPath = trajIndicesPath
        self.templateFilesBuildThread.deformationGraphPath = deformationGraphPath

        self.templateFilesBuildThread.resultSignal.connect(self._stopLoadingInTemplateFileBuild)
        self.templateFilesBuildThread.start()

    def _stopLoadingInTemplateFileBuild(self, signalValue):
        self.movieLoadingInTemplateFileBuild.stop()
        self.labelLoadingInTemplateFileBuild.clear()

        pe = QtGui.QPalette()
        pe.setColor(QtGui.QPalette.WindowText, QtCore.Qt.red)  # 设置字体颜色
        self.labelLoading2InTemplateFileBuild.setPalette(pe)
        self.labelLoading2InTemplateFileBuild.setText(signalValue)

    def _show3dInTemplateFileBuild(self):
        geometries=[]
        if self.isShowTemplateInTemplateFileBuild:
            if self.parent.tempalteCloud is None:
                templatePath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入模板点云', "",
                                                                    "All Files (*);;Ply Files (*.ply)")
                if templatePath=='':
                    return
                self.parent.tempalteCloud = o3d.io.read_point_cloud(templatePath.encode('gbk'))
            self.parent.tempalteCloud.paint_uniform_color([1,0,0])
            geometries.append(self.parent.tempalteCloud)
        if self.isShowTrajectoryInTemplateFileBuild:
            if self.parent.traj is None:
                trajPath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入点胶轨迹点云', "",
                                                                     "All Files (*);;Ply Files (*.ply)")
                if trajPath=='':
                    return
                self.parent.traj = o3d.io.read_point_cloud(trajPath.encode('gbk'))
            self.parent.traj.paint_uniform_color([0,0,1])
            geometries.append(self.parent.traj)
        if self.isShowDeformationGraphInTemplateFileBuild:
            if self.parent.deformationGraph is None:
                self._readDeformationGrpah()
            self.parent.deformationGraph.paint_uniform_color([0,1,0])
            geometries.append(self.parent.deformationGraph)

        o3d.visualization.draw_geometries(geometries, window_name='模板文件生成', width=800, height=600, top=200,
                                          left=100)

    def _readDeformationGrpah(self):
        deformationGraphPath, _ = QtWidgets.QFileDialog.getOpenFileName(self.windowTemplateFileBuild, '导入变形图文件', "",
                                                                     "All Files (*);;Text Files (*.txt)")

        points=[]
        lines=[]

        with open(deformationGraphPath,'r') as fp:
            scaleNorm=(float)(fp.readline().split(' ')[2].split('\n')[0])
            for i in range(3):
                nodeSize=fp.readline()
            nodeSize=(int)(nodeSize.split(' ')[2].split('\n')[0])
            for i in range(nodeSize):
                node=fp.readline()
                point=[scaleNorm*(float)(node.split(' ')[2]),scaleNorm*(float)(node.split(' ')[3]),scaleNorm*(float)(node.split(' ')[4].split('\n')[0])]
                points.append(point)
            for i in range(2):
                nodeEdgeSize = fp.readline()
            nodeEdgeSize = (int)(nodeEdgeSize.split(' ')[2].split('\n')[0])
            for i in range(nodeEdgeSize):
                nodeEdge=fp.readline()
                line=[(int)(nodeEdge.split(' ')[0]),(int)(nodeEdge.split(' ')[1])]
                lines.append(line)

        points=np.array(points)
        lines=np.array(lines)
        self.parent.deformationGraphLineSet = o3d.geometry.LineSet()
        self.parent.deformationGraphLineSet.points= o3d.utility.Vector3dVector(points)
        self.parent.deformationGraphLineSet.lines= o3d.utility.Vector2iVector(lines)


    def _checkboxTemplateChanged(self,state):
        if state == 2:  # Qt.Checked
            self.isShowTemplateInTemplateFileBuild=True
        else:
            self.isShowTemplateInTemplateFileBuild = False

    def _checkboxTrajectoryChanged(self,state):
        if state == 2:  # Qt.Checked
            self.isShowTrajectoryInTemplateFileBuild=True
        else:
            self.isShowTrajectoryInTemplateFileBuild = False

    def _checkboxDeformationGraphChanged(self,state):
        if state == 2:  # Qt.Checked
            self.isShowDeformationGraphInTemplateFileBuild=True
        else:
            self.isShowDeformationGraphInTemplateFileBuild = False


#################拼接线程类################################################
class BorderAlignmentThread(QtCore.QThread):
    resultSignal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super(QtCore.QThread, self).__init__()
        self.parent = parent

    def run(self):
        self.parent.textBrowser.append(printLog('边框点云拼接计算启动...'))
        logging.getLogger('mainLog').info('边框点云拼接计算启动...')
        signalMessage=self._align()
        self.resultSignal.emit(signalMessage)

    def _align(self):
        if self.parent.settings.templateShort1Path is None or self.parent.settings.templateShort1Path == '':
            return '短边1模板点云文件路径未设置'
        elif self.parent.settings.templateLong2Path is None or self.parent.settings.templateLong2Path == '':
            return '长边2模板点云文件路径未设置'

        elif self.parent.settings.templateShort3Path is None or self.parent.settings.templateShort3Path == '':
            return '短边3模板点云文件路径未设置'

        elif self.parent.settings.templateLong4Path is None or self.parent.settings.templateLong4Path == '':
            return '长边4模板点云文件路径未设置'

        else:
            short1Path = ctypes.create_string_buffer(self.parent.settings.templateShort1Path.encode('gbk'))
            long2Path = ctypes.create_string_buffer(self.parent.settings.templateLong2Path.encode('gbk'))
            short3Path = ctypes.create_string_buffer(self.parent.settings.templateShort3Path.encode('gbk'))
            long4Path = ctypes.create_string_buffer(self.parent.settings.templateLong4Path.encode('gbk'))

            matEle = (ctypes.c_double * 64)()

            getAlignTransMatriceDll(short1Path, long2Path, short3Path, long4Path, matEle)

            i = 0
            for ele in matEle:
                if i < 16:
                    row = (int)(i / 4)
                    col = i % 4
                    self.parent.settings.templateAlignmentMat1[col][row] = ele
                elif i < 32:
                    row = (int)((i - 16) / 4)
                    col = i % 4
                    self.parent.settings.templateAlignmentMat2[col][row] = ele
                elif i < 48:
                    row = (int)((i - 32) / 4)
                    col = i % 4
                    self.parent.settings.templateAlignmentMat3[col][row] = ele
                else:
                    row = (int)((i - 48) / 4)
                    col = i % 4
                    self.parent.settings.templateAlignmentMat4[col][row] = ele
                i = i + 1

            self.parent.settings.templateAlignmentMat1 = np.array(self.parent.settings.templateAlignmentMat1)
            self.parent.settings.templateAlignmentMat2 = np.array(self.parent.settings.templateAlignmentMat2)
            self.parent.settings.templateAlignmentMat3 = np.array(self.parent.settings.templateAlignmentMat3)
            self.parent.settings.templateAlignmentMat4 = np.array(self.parent.settings.templateAlignmentMat4)

            self.parent.textBrowser.append(printLog('边框点云拼接完成.'))
            logging.getLogger('mainLog').info('边框点云拼接完成.')

            mat1List = self.parent.settings.templateAlignmentMat1.tolist()
            mat1Str = '\t\t\t[\n'
            for row in mat1List:
                mat1Str = mat1Str + '\t\t\t'
                for ele in row:
                    mat1Str = mat1Str + str(ele) + '  '
                mat1Str = mat1Str + '\n'
            mat1Str = mat1Str + '\t\t\t]'
            self.parent.textBrowser.append(printLog('短边1转换矩阵为：\n' + mat1Str))
            logging.getLogger('mainLog').info('短边1转换矩阵为：\n' + mat1Str)

            mat2List = self.parent.settings.templateAlignmentMat2.tolist()
            mat2Str = '\t\t\t[\n'
            for row in mat2List:
                mat2Str = mat2Str + '\t\t\t'
                for ele in row:
                    mat2Str = mat2Str + str(ele) + '  '
                mat2Str = mat2Str + '\n'
            mat2Str = mat2Str + '\t\t\t]'
            self.parent.textBrowser.append(printLog('长边2转换矩阵为：\n' + mat2Str))
            logging.getLogger('mainLog').info('长边2转换矩阵为：\n' + mat2Str)

            mat3List = self.parent.settings.templateAlignmentMat3.tolist()
            mat3Str = '\t\t\t[\n'
            for row in mat3List:
                mat3Str = mat3Str + '\t\t\t'
                for ele in row:
                    mat3Str = mat3Str + str(ele) + '  '
                mat3Str = mat3Str + '\n'
            mat3Str = mat3Str + '\t\t\t]'
            self.parent.textBrowser.append(printLog('短边3转换矩阵为：\n' + mat3Str))
            logging.getLogger('mainLog').info('短边3转换矩阵为：\n' + mat3Str)

            mat4List = self.parent.settings.templateAlignmentMat4.tolist()
            mat4Str = '\t\t\t[\n'
            for row in mat4List:
                mat4Str = mat4Str + '\t\t\t'
                for ele in row:
                    mat4Str = mat4Str + str(ele) + '  '
                mat4Str = mat4Str + '\n'
            mat4Str = mat4Str + '\t\t\t]'
            self.parent.textBrowser.append(printLog('长边4转换矩阵为：\n' + mat4Str))
            logging.getLogger('mainLog').info('长边4转换矩阵为：\n' + mat4Str)

            return '边框点云拼接完成'


#################获取模板文件线程类################################################
class TemplateFilesBuildThread(QtCore.QThread):
    resultSignal = QtCore.pyqtSignal(str)

    def __init__(self, parent=None):
        super(QtCore.QThread, self).__init__()
        self.parent = parent
        self.templateCloudPath=''
        self.trajCloudPath = ''
        self.templateMeshPath = ''
        self.trajIndicesPath = ''
        self.deformationGraphPath = ''

    def run(self):
        self.parent.textBrowser.append(printLog('模板变形图等文件生成启动...'))
        logging.getLogger('mainLog').info('模板变形图等文件生成启动...')

        signalMessage=self._buildFiles()
        self.resultSignal.emit(signalMessage)


    def _buildFiles(self):
        templateCloudPath_ = ctypes.create_string_buffer(self.templateCloudPath.encode('gbk'))
        trajCloudPath_ = ctypes.create_string_buffer(self.trajCloudPath.encode('gbk'))
        templateMeshPath_ = ctypes.create_string_buffer(self.templateMeshPath.encode('gbk'))
        trajIndicesPath_ = ctypes.create_string_buffer(self.trajIndicesPath.encode('gbk'))
        deformationGraphPath_ = ctypes.create_string_buffer(self.deformationGraphPath.encode('gbk'))

        getTemplateFilesDll(templateCloudPath_, trajCloudPath_, templateMeshPath_, trajIndicesPath_,
                            deformationGraphPath_)

        self.parent.textBrowser.append(printLog('【模板生成】模板Mesh文件存储于： ' + self.templateMeshPath + ' .'))
        logging.getLogger('mainLog').info('【模板生成】模板Mesh文件存储于： ' + self.templateMeshPath + ' .')

        self.parent.textBrowser.append(printLog('【模板生成】轨迹索引文件存储于： ' + self.trajIndicesPath + ' .'))
        logging.getLogger('mainLog').info('【模板生成】轨迹索引文件存储于： ' + self.trajIndicesPath + ' .')

        self.parent.textBrowser.append(printLog('【模板生成】模板变形图文件存储于： ' + self.deformationGraphPath + ' .'))
        logging.getLogger('mainLog').info('【模板生成】模板变形图文件存储于： ' + self.deformationGraphPath + ' .')

        return '模板变形图等文件生成成功'