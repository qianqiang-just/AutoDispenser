import sys
from PyQt5 import QtWidgets, QtCore, QtGui
import menus
import dataStructure
import settings
import logging
import datetime
import random

logger = logging.getLogger('mainLog')
logName = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S_') + str(random.random())[-6:] + '.log'
fileHandle = logging.FileHandler('logs/' + logName)
fileHandle.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
logger.addHandler(fileHandle)
logger.setLevel(logging.DEBUG)

logger.info('程序启动...')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.initVariables()

        self.initUI()

    def initVariables(self):
        self.settings = settings.Settings()

        self.templateCloudShort1 = None
        self.templateCloudLong2 = None
        self.templateCloudShort3 = None
        self.templateCloudLong4 = None
        self.tempalteCloud = None
        self.traj=None
        self.deformationGraphLineSet=None #o3d.geometry.LineSet() 在模板生成时用于画图

        self.pointCloud=dataStructure.PointCloud() #轨迹标注时的暂存点云

        self.deformationGraphDll=None #用于和Dll传输数据

        self.templateMeshPtrDll=None
        self.targetShort1PtrDll=None
        self.targetLong2PtrDll = None
        self.targetShort3PtrDll = None
        self.targetLong4PtrDll = None

    def initUI(self):
        self.setWindowTitle("自动点胶轨迹定位系统DEMO");

        self.centralwidget = QtWidgets.QWidget(self)
        self.mdiArea = QtWidgets.QMdiArea(self.centralwidget)
        self.textBrowser = QtWidgets.QTextBrowser(self.centralwidget)

        self.verticalLayout = QtWidgets.QVBoxLayout(self.centralwidget)
        self.verticalLayout.addWidget(self.mdiArea)
        self.verticalLayout.addWidget(self.textBrowser)

        self.setCentralWidget(self.centralwidget)

        self.menubar = self.menuBar()

        self.menu1 = self.menubar.addMenu('模板')
        self.actMenu1_1 = self.menu1.addAction('边框文件路径')
        self.actMenu1_2 = self.menu1.addAction('边框点云拼接')
        self.actMenu1_3 = self.menu1.addAction('模板文件生成')


        self.actionTemplate = menus.ActionTemplate(self)  # 这个是和整个大菜单项关联的类
        self.actMenu1_1.triggered.connect(self.actionTemplate.slotBorderPathSetting)  # 每个子菜单具体去匹配ActionTemplateImport下面的不同函数（均以slot开头）
        self.actMenu1_2.triggered.connect(self.actionTemplate.slotBorderAlignment)
        self.actMenu1_3.triggered.connect(self.actionTemplate.slotTemplateFileBuild)

        self.menu2 = self.menubar.addMenu('轨迹')
        self.actMenu2_1 = self.menu2.addAction('模板轨迹标注')

        self.actionTrajectory = menus.ActionTrajectory(self)  # 这个是和整个大菜单项关联的类
        self.actMenu2_1.triggered.connect(self.actionTrajectory.slotTrajectoryLabeling)


        self.menu3 = self.menubar.addMenu('测量')
        self.actMenu3_1 = self.menu3.addAction('模板文件路径')
        self.actMenu3_2 = self.menu3.addAction('模板文件加载')
        self.actMenu3_3 = self.menu3.addAction('工件轨迹测量')

        self.actionMeasurement = menus.ActionMeasurement(self)
        self.actMenu3_1.triggered.connect(self.actionMeasurement.slotTemplatePathSetting)
        self.actMenu3_2.triggered.connect(self.actionMeasurement.slotLoadTemplateFiles)
        self.actMenu3_3.triggered.connect(self.actionMeasurement.slotGetTrajectory)

        self.menu4 = self.menubar.addMenu('工具')
        self.actMenu4_1 = self.menu4.addAction('文件格式转换')
        self.menu5 = self.menubar.addMenu('扫描')
        self.menu5 = self.menubar.addMenu('设置')
        self.actMenu5_1 = self.menu4.addAction('拼接参数设置')
        self.menu6 = self.menubar.addMenu('关于')


        self.showMaximized()


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mainWindow = MainWindow()

    sys.exit(app.exec_())
