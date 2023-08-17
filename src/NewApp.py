import sys
from Window import MainWindow
print(sys.path)
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow

print("Starting Appication...")

# print("Connecting to robot...")
# robot = frrpc.RPC('192.168.58.2')
# ret = robot.GetSDKVersion()    # Query SDK version number
# if ret[0] == 0:
#     # 0-No fault, return format:[errcode,data],errcode-Fault code,data-Data
#     print("SDK version is:",ret[1])
#     print("Connected")
# else:
#     print("the errcode is: ", ret[0])
if __name__ == '__main__':
    app = QApplication(sys.argv)
    mainwindow = QMainWindow()
    ui = MainWindow(mainwindow)
    mainwindow.show()
    sys.exit(app.exec_())
