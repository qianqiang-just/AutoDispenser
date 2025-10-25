from PyQt5 import QtWidgets, QtCore, QtGui
from utils.utils import timeNow2Str, printLog
import logging

class DrawingWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.paintMode = None
        # self.isAssistedLabel = False  # 开启辅助标注
        self.clickMode = 'TrajectorySetting'

        self.yMin = None  # 数据y最小值
        self.yMax = None
        self.yWidgetMin = 50  # 显示y最小值
        self.yWidgetMax = 400

        self.xWidgetMin = 50
        self.xWidgetMax = 850

        self.xLabeled = None  # 被标记点（和鼠标点击点相同）在屏幕中的x像素
        self.yLabeled = None  # 被标记点（和鼠标点击点不同）在屏幕中的y像素

    def _drawGrid(self):
        painter = QtGui.QPainter(self)
        pen = QtGui.QPen(QtCore.Qt.DashLine)
        pen.setColor(QtCore.Qt.black)
        painter.setFont(QtGui.QFont('Courier New', 8, 1))
        pen.setWidth(1)

        painter.setPen(pen)
        yDelta = (self.yWidgetMax - self.yWidgetMin) / 8
        if self.yMin is not None:
            yValueDelta = (self.yMax - self.yMin) / 8
        for i in range(9):  # 左上角 10,200 右下角是810,400
            painter.drawLine(self.xWidgetMin, int(self.yWidgetMin + i * yDelta), self.xWidgetMax,
                             int(self.yWidgetMin + i * yDelta))
            if self.yMin is not None:
                painter.drawText(0, int(self.yWidgetMin + i * yDelta + 5), "{:.2f}".format(self.yMin + i * yValueDelta))

        xDelta = (self.xWidgetMax - self.xWidgetMin) / 10
        for i in range(11):
            painter.drawLine(int(self.xWidgetMin + i * xDelta), self.yWidgetMin, int(self.xWidgetMin + i * xDelta),
                             self.yWidgetMax)
            painter.drawText(int(self.xWidgetMin + i * xDelta - 13), self.yWidgetMax + 20, str(int(i * xDelta)))

    def paintEvent(self, e):
        self._drawGrid()
        painter = QtGui.QPainter(self)
        pen = QtGui.QPen(QtCore.Qt.red, 2)
        painter.setFont(QtGui.QFont('Courier New', 15, 4))
        painter.setPen(pen)

        if self.paintMode == 'DrawLine':
            if self.parent.pointCloud.currentLinePoints.lineData is not None:
                painter.drawText(20, 20,
                                 self.parent.pointCloud.currentLinePoints.lineKey + '/' + self.parent.pointCloud.lineEndKey)

                for key, value in self.parent.pointCloud.currentLinePoints.lineData.items():
                    if key == 'labeled':
                        pen = QtGui.QPen(QtCore.Qt.blue, 4)
                        painter.setPen(pen)
                        self.xLabeled = self.xWidgetMax - self.parent.pointCloud.currentLinePoints.lineData[key][
                            1]
                        y = self.parent.pointCloud.currentLinePoints.lineData[key][5]

                        self.yLabeled = self.yWidgetMin + (y - self.yMin) * (self.yWidgetMax - self.yWidgetMin) / (
                                self.yMax - self.yMin)

                        painter.drawLine(self.xLabeled - 3, self.yLabeled - 3, self.xLabeled + 3, self.yLabeled + 3)
                        painter.drawLine(self.xLabeled - 3, self.yLabeled + 3, self.xLabeled + 3, self.yLabeled - 3)
                        pen = QtGui.QPen(QtCore.Qt.red, 2)
                        painter.setPen(pen)
                    else:
                        x = -value[1] + self.xWidgetMax

                        y = self.yWidgetMin + (value[5] - self.yMin) * (self.yWidgetMax - self.yWidgetMin) / (
                                self.yMax - self.yMin)
                        painter.drawPoint(x, y)

    def mousePressEvent(self, event):
        current_y = event.pos().y()
        # 在画图范围内点击有效
        if current_y < self.yWidgetMin or current_y > self.yWidgetMax:
            return

        if event.button() == QtCore.Qt.LeftButton:
            if self.clickMode == 'TrajectorySetting':
                pos = event.pos()  # 获取鼠标点击位置
                self.xLabeled = pos.x()
                x = self.xWidgetMax - self.xLabeled
                if self.parent.pointCloud.currentLinePoints.lineData is None:
                    return
                elif x in self.parent.pointCloud.currentLinePoints.lineData.keys():
                    self.parent.pointCloud.currentLinePoints.lineData['labeled'] = \
                        self.parent.pointCloud.currentLinePoints.lineData[x]


                    linesIndice = list(self.parent.pointCloud.indexicalCloud.keys())
                    end = len(linesIndice)
                    if self.parent.settings.labelStep != 'all':
                        end = self.parent.pointCloud.currentLinePoints.lineNo + int(self.parent.settings.labelStep)
                        if end > len(linesIndice):
                            end = len(linesIndice)
                    for i in range(self.parent.pointCloud.currentLinePoints.lineNo + 1, end):
                        key = linesIndice[i]
                        if x in self.parent.pointCloud.indexicalCloud[key].keys():
                            self.parent.pointCloud.indexicalCloud[key]['labeled'] = \
                                self.parent.pointCloud.indexicalCloud[key][x]
                        else:
                            break

                    ptStr = '(' + str(self.parent.pointCloud.currentLinePoints.lineData['labeled'][3]) + ' ,' + str(
                        self.parent.pointCloud.currentLinePoints.lineData['labeled'][4]) + ' ,' + str(
                        self.parent.pointCloud.currentLinePoints.lineData['labeled'][5]) + ')'
                    self.parent.textBrowser.append(printLog('【轨迹标注】设置轮廓线' + self.parent.pointCloud.currentLinePoints.lineKey + '的轨迹标注点: ' + ptStr))
                    logging.getLogger('mainLog').info('【轨迹标注】设置轮廓线' + self.parent.pointCloud.currentLinePoints.lineKey + '的轨迹标注点: ' + ptStr)
                    self.update()

        if event.button() == QtCore.Qt.RightButton:
            if self.clickMode == 'TrajectorySetting':
                if self.parent.pointCloud.currentLinePoints.lineData is None:
                    return
                if 'labeled' in self.parent.pointCloud.currentLinePoints.lineData.keys():
                    self.parent.pointCloud.currentLinePoints.lineData.pop('labeled')


                    linesIndice = list(self.parent.pointCloud.indexicalCloud.keys())
                    end = len(linesIndice)
                    if self.parent.settings.labelStep != 'all':
                        end = self.parent.pointCloud.currentLinePoints.lineNo + int(self.parent.settings.labelStep)
                        if end > len(linesIndice):
                            end = len(linesIndice)
                    for i in range(self.parent.pointCloud.currentLinePoints.lineNo + 1, end):
                        key = linesIndice[i]
                        if 'labeled' in self.parent.pointCloud.indexicalCloud[key].keys():
                            self.parent.pointCloud.indexicalCloud[key].pop('labeled')
                        else:
                            break

                    self.parent.textBrowser.append(printLog('【轨迹标注】删除轮廓线' + self.parent.pointCloud.currentLinePoints.lineKey + '的轨迹标注点.'))
                    logging.getLogger('mainLog').info('【轨迹标注】删除轮廓线' + self.parent.pointCloud.currentLinePoints.lineKey + '的轨迹标注点.')

                    self.update()
