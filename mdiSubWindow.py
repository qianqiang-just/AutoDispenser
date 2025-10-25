from PyQt5 import QtWidgets, QtCore

#所有在主窗口中打开的自窗口的父类，主要是为了在自窗口关闭时，触发额外的关闭信号
class MdiSubWindow(QtWidgets.QMdiSubWindow):
    sigClosed = QtCore.pyqtSignal(str)
    closed = False
    isChangeClose2Hidden=False

    def closeEvent(self, event):
        if not self.closed:
            self.sigClosed.emit(self.windowTitle())

        if not self.isChangeClose2Hidden:
            QtWidgets.QMdiSubWindow.closeEvent(self, event)

    # def mousePressEvent(self, event):
    #     try:
    #         if event.button() == QtCore.Qt.LeftButton:
    #             pos = event.pos()  # 获取鼠标点击位置
    #             print(f"点击了 ({pos.x()}, {pos.y()})")  # 输出坐标
    #     except Exception:
    #         t=0
