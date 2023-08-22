import sys

import typing
import os
import asyncio
import keyword
import pygame
import pkgutil
import pyperclip
import threading
import json
import datetime
import RobotAction
from fwd import Ui_MainWindow
from PyQt5 import QtCore
from PyQt5.QtGui import QKeySequence,QStandardItem,QStandardItemModel,QColor,QFont
from PyQt5.QtCore import Qt,pyqtSignal,QObject, QTimer
from PyQt5.QtWidgets import QApplication,\
    QMenuBar, QMenu, QAction, QFileDialog, \
        QInputDialog, QMessageBox, QStatusBar, \
            QToolBar,QDialog,QTextEdit,QVBoxLayout,\
                QPushButton,QTreeView,QSizePolicy
from PyQt5.Qsci import QsciScintilla, QsciLexerPython,QsciAPIs
from pathlib import Path
from enum import Enum
from inputs import get_gamepad
global current_directory
current_directory = os.path.dirname(os.path.realpath(__file__))
print(current_directory)

class PythonScriptDialog(QDialog):
    def __init__(self):
        super().__init__()

        self.init_ui()
        
    def init_ui(self):
        self.text_edit = QTextEdit()
        self.run_button = QPushButton("Run Script")
        self.run_button.clicked.connect(self.run_script)

        layout = QVBoxLayout()
        layout.addWidget(self.text_edit)
        layout.addWidget(self.run_button)

        self.setLayout(layout)
        self.setWindowTitle("Python Script Dialog")

    def run_script(self):
        script = self.text_edit.toPlainText()
        try:
            exec(script)
        except Exception as e:
            print("Error:", e)

class Communicator(QObject):
    # Create a custom signal to be emitted from Robot.py
    update_label_signal = pyqtSignal(str)

class MainWindow(Ui_MainWindow):
    def __init__(self, MainWindow):
        self.setupUi(MainWindow)
        self._SetUp_Triggers()
        self.AddModel_Treeview()
        self.dialog = PythonScriptDialog()
         # Create the communicator instance
        self.communicator = Communicator()
        self.communicator.update_label_signal.connect(self.update_label_text)
        self.timer = QTimer()
        self.timer.setInterval(500)
        self.timer.timeout.connect(self.update_pose)
        #self.timer.start()
        self.load_files()
        self.setup_tab_view_code()
        # _ = self.create_gamepad()

        #ininitialize value

        self.connected_robot = False

        self.Radiobtn_TX.setChecked(True)
        self.Ref_Frame_Box.setCurrentIndex(0)

        self.var_manual_speed = 20
        self.manual_speed.setValue(self.var_manual_speed)

        self.var_auto_speed = 20
        self.Auto_Speed.setValue(self.var_auto_speed)

        self.var_manual_acc = 20
        self.manual_acc.setValue(self.var_manual_acc)

        self.var_auto_acc = 20
        self.Auto_Acc.setValue(self.var_auto_acc)

        self.radio_auto_mode.setChecked(False)
        self.radio_non_teaching_mode.setChecked(True)

    def _SetUp_Triggers(self):
        # File Menu
        self.actionNew.triggered.connect(self.newFile)
        self.actionOpen.triggered.connect(self.openFile)
        self.actionSave.triggered.connect(self.saveFile)
        self.actionExit.triggered.connect(self.closeEvent)
        self.actionSave_As.triggered.connect(self.saveAs)
        # Connection Menu
        self.actionConnect.triggered.connect(self.connect)
        self.actionDisconnect.triggered.connect(self.disconnect)

        # Help Menu
        self.actionHelp.triggered.connect(self.helpContent)
        self.actionAbout.triggered.connect(self.about)

        #Program Menu
        self.actionTool.triggered.connect(self.poseViewShow)

        #Slider

        self.horizontalSlider.valueChanged.connect(self.join1Change)
        self.horizontalSlider_2.valueChanged.connect(self.join2Change)
        self.horizontalSlider_3.valueChanged.connect(self.join3Change)
        self.horizontalSlider_4.valueChanged.connect(self.join4Change)
        self.horizontalSlider_5.valueChanged.connect(self.join5Change)
        self.horizontalSlider_6.valueChanged.connect(self.join6Change)

        #DoubleSpinbox
        self.doubleSpinBox.valueChanged.connect(self.doubleSpinBoxChange)
        self.doubleSpinBox_2.valueChanged.connect(self.doubleSpinBoxChange2)
        self.doubleSpinBox_3.valueChanged.connect(self.doubleSpinBoxChange3)
        self.doubleSpinBox_4.valueChanged.connect(self.doubleSpinBoxChange4)
        self.doubleSpinBox_5.valueChanged.connect(self.doubleSpinBoxChange5)
        self.doubleSpinBox_6.valueChanged.connect(self.doubleSpinBoxChange6)

        self.manual_speed.valueChanged.connect(self.manual_speed_change)
        self.Auto_Speed.valueChanged.connect(self.Auto_Speed_change)
        self.manual_acc.valueChanged.connect(self.manual_acc_change)
        self.Auto_Acc.valueChanged.connect(self.Auto_Acc_change)
        #Button trigger

        self.btn_save_program.clicked.connect(self.saveProgram)
        self.btn_run_program.clicked.connect(self.run_script)

        self.btn_jog_positive.pressed.connect(self.jog_positive_start)
        self.btn_jog_positive.released.connect(self.jog_positive_stop)

        self.btn_jog_negative.pressed.connect(self.jog_negative_start)
        self.btn_jog_negative.released.connect(self.jog_negative_stop)
        #Radio button trigger
        self.Radiobtn_RX.toggled.connect(self.checkMode_jog)
        self.Radiobtn_RY.toggled.connect(self.checkMode_jog)
        self.Radiobtn_RZ.toggled.connect(self.checkMode_jog)
        self.Radiobtn_TX.toggled.connect(self.checkMode_jog)
        self.Radiobtn_TY.toggled.connect(self.checkMode_jog)
        self.Radiobtn_TZ.toggled.connect(self.checkMode_jog)
        self.radio_J1.toggled.connect(self.checkMode_jog)
        self.radio_J2.toggled.connect(self.checkMode_jog)
        self.radio_J3.toggled.connect(self.checkMode_jog)
        self.radio_J4.toggled.connect(self.checkMode_jog)
        self.radio_J5.toggled.connect(self.checkMode_jog)
        self.radio_J6.toggled.connect(self.checkMode_jog)

        self.radio_auto_mode.toggled.connect(self.Mode_Changed)
        self.radio_manual_mode.toggled.connect(self.Mode_Changed)

        self.radio_non_teaching_mode.toggled.connect(self.Drag_mode_changed)
        self.radio_teaching_mode.toggled.connect(self.Drag_mode_changed)
        # Dial
        # self.Dial_button_jog.valueChanged.connect(self.Dial_Jog)

        #Combobox triggers

        self.Ref_Frame_Box.currentIndexChanged.connect(self.Reference_FrameChanged)
#region TreeView
    def AddModel_Treeview(self):
        self.model = QStandardItemModel()
        self.program_treeview.setModel(self.model)
        self.program_treeview.setHeaderHidden(True)
        self.program_treeview.setSelectionMode(QTreeView.SelectionMode.SingleSelection)
        self.program_treeview.setSelectionBehavior(QTreeView.SelectionBehavior.SelectRows)
        self.program_treeview.clicked.connect(self.tree_view_clicked)
        self.program_treeview.setIndentation(10)
        root_item = self.model.invisibleRootItem()  # Get the root item
        new_item = QStandardItem('New Station')
        root_item.appendRow(new_item)

        self.program_treeview.setContextMenuPolicy(Qt.CustomContextMenu)
        self.program_treeview.customContextMenuRequested.connect(self.treeview_show_context_menu)
        
    
    def treeview_show_context_menu(self,position):
        menu = QMenu(self.program_treeview)
        add_action = QAction('Add New Target', self.program_treeview)
        add_action.triggered.connect(self.add_new_target_item)
        delete_action = QAction('Delete', self.program_treeview)
        delete_action.triggered.connect(self.delete_target_item)
        new_program_action = QAction('Add Program', self.program_treeview)
        new_program_action.triggered.connect(self.new_program_item)
        copy_point_action = QAction('Copy Point', self.program_treeview)
        copy_point_action.triggered.connect(self.copy_point_item)
        menu.addAction(add_action)
        menu.addAction(delete_action)
        menu.addAction(new_program_action)
        menu.addAction(copy_point_action)
        menu.exec_(self.program_treeview.viewport().mapToGlobal(position))

    def add_new_target_item(self):
        root_item = self.model.invisibleRootItem()  # Get the root item
        item1 = root_item.child(0)  # Get 'Item 1' directly
        num = item1.rowCount() + 1
        new_item = QStandardItem(f'Target {num}')
        array_data = [[self.doubleSpinBox.value(),\
                      self.doubleSpinBox_2.value(),\
                        self.doubleSpinBox_3.value(),\
                            self.doubleSpinBox_4.value(),\
                                self.doubleSpinBox_5.value(),\
                                    self.doubleSpinBox_6.value()]]
        if self.connected_robot:
            des_pos = robot.GetForwardKin(array_data[0])
            array_data.append(des_pos)
        new_item.setData(array_data,role= Qt.UserRole)
        item1.appendRow(new_item)

        print(new_item.data(Qt.UserRole))

    def delete_target_item(self):
        selected_index = self.program_treeview.selectionModel().currentIndex()
        selected_item = self.model.itemFromIndex(selected_index)
        selected_directory = current_directory
        program_directory = selected_directory +'/ProgramData'
        point_directory = selected_directory +'/Point'
        if selected_item is not None:
            print(selected_item.text())
            if isinstance(selected_item.data(Qt.UserRole), list):
                item_text = point_directory + '/' + selected_item.text()+ '.json'
                os.remove(item_text)
            else:
                item_text = program_directory + '/' + selected_item.text()+ '.py'
                os.remove(item_text)

        self.model.removeRow(selected_index.row(), selected_index.parent())
        
        

    def new_program_item(self):
        root_item = self.model.invisibleRootItem()  # Get the root item
        num = root_item.rowCount() + 1
        new_item = QStandardItem(f'Program {num}')
        selected_directory = current_directory
        program_directory = selected_directory +'/ProgramData'
        file_path = program_directory +'/'+ f'Program {num}.py'
        new_item.setData(file_path,Qt.UserRole)
        with open(file_path, 'w') as file:
            pass  # Placeholder content
        root_item.appendRow(new_item)
        self.set_new_tab(Path(file_path),True)

    def item_name_changed(self, item):
        new_name = item.text()
        print(f"Item name changed to: {new_name}")

    def copy_point_item(self,index):
        selected_index = self.program_treeview.selectionModel().currentIndex()
        selected_item = self.model.itemFromIndex(selected_index)
        if selected_item is not None:
            print(selected_item.text())
            if isinstance(selected_item.data(Qt.UserRole), list):
                targetpoints = [selected_item.data(Qt.UserRole)]

                txt = selected_item.text().replace(" ", "_")+'_J =['

                for i in targetpoints[0]:
                    txt += f'{i},'
                txt = txt[:-1] +']\n'

                
                
                if len(targetpoints) > 1:
                    txt += selected_item.text().replace(" ", "_")+'_P =['
                    for i in targetpoints[1]:
                        txt += f'{i},'
                    txt = txt[:-1] +']\n'
                
                external_pos = f'e{selected_item.text().replace(" ", "_")}P=[0.000,0.000,0.000,0.000]\nd{selected_item.text().replace(" ", "_")}P=[1.000,1.000,1.000,1.000,1.000,1.000]'
                txt += external_pos
                pyperclip.copy(txt)
            else:
                pass

    def save_items(self):
        # file_dialog = QFileDialog(self.centralwidget)
        # file_dialog.setFileMode(QFileDialog.DirectoryOnly)
        # file_dialog.setWindowTitle('Save Items')

        # if file_dialog.exec_() == QFileDialog.Accepted:
        selected_directory = current_directory
        point_directory = selected_directory +'/Point'
        program_directory = selected_directory +'/ProgramData'
        root_item = self.model.invisibleRootItem()
        
        item = root_item.child(0)
        for row in range(item.rowCount()):
            if item != None:
                item1 = item.child(row)
                item_name = item1.text()
                item_data = self.collect_item_data(item1)
                
                file_name = f"{point_directory}/{item_name}.json"
                with open(file_name, 'w') as file:
                    json.dump(item_data, file, indent=4)

        for row in range(1, root_item.rowCount()):  # Start from index 1 to exclude the first child
            item = root_item.child(row)
            item_name = item.text()
            item_data = self.collect_item_data(item)

            file_name = f"{program_directory}/{item_name}.json"
            with open(file_name, 'w') as file:
                json.dump(item_data, file, indent=4)
        # if item.text() == 'Item 1':
        #     pass
        # else:
        #     file_name = f"{selected_directory}/{item.text()}.py"
        #     with open(file_name, 'w') as file:
        #         file.write(f"# {item.text()} Python File\n\n")
        #         file.write(f"custom_data = {item.data(Qt.UserRole)}\n")
                

    def collect_item_data(self, item):
        return item.data(Qt.UserRole)
    
    def load_files(self):
        root_item = self.model.invisibleRootItem()
        item_1 = root_item.child(0)
        selected_directory = current_directory
        point_directory = selected_directory +'/Point'
        program_directory = selected_directory +'/ProgramData'

        if not os.path.exists(point_directory):
            os.makedirs(point_directory)
        if not os.path.exists(program_directory):
            os.makedirs(program_directory)


        for file_name in os.listdir(point_directory):
            file_path = os.path.join(point_directory, file_name)

            if os.path.isfile(file_path):
                if file_name.endswith('.json'):
                    with open(file_path, 'r') as file:
                        json_data = json.load(file)
                        self.add_item_to_tree(item_1, os.path.splitext(file_name)[0], json_data)
            
        for file_name in os.listdir(program_directory):
            file_path = os.path.join(program_directory, file_name)

            if os.path.isfile(file_path):
                if file_name.endswith('.py'):
                    with open(file_path, 'r') as file:
                        
                        self.add_item_to_tree(root_item, os.path.splitext(file_name)[0], file_path)


    def add_item_to_tree(self, parent_item, item_name, item_data):
        new_item = QStandardItem(item_name)
        new_item.setData(item_data, Qt.UserRole)
        parent_item.appendRow(new_item)

    def tree_view_clicked(self,index):
        root = self.model.invisibleRootItem()
        if index.isValid() and index.row() > 0:
            clicked_item = root.child(index.row())
            #if clicked_item.parent() == root:
            path = clicked_item.data(Qt.UserRole)
            # path = root.child(index.row()).data(Qt.UserRole)
            p = Path(path)
            self.set_new_tab(p,"")
        
            
#endregion
    
#region Join change
    def join1Change(self):

       self.doubleSpinBox.setValue(self.horizontalSlider.value())
       
    def join2Change(self):

       self.doubleSpinBox_2.setValue(self.horizontalSlider_2.value())

    def join3Change(self):

       self.doubleSpinBox_3.setValue(self.horizontalSlider_3.value())

    def join4Change(self):

       self.doubleSpinBox_4.setValue(self.horizontalSlider_4.value())

    def join5Change(self):

       self.doubleSpinBox_5.setValue(self.horizontalSlider_5.value())

    def join6Change(self):

       self.doubleSpinBox_6.setValue(self.horizontalSlider_6.value())

#endregion
    
#region doubleSpinBox
    def doubleSpinBoxChange(self):
        self.horizontalSlider.setValue(int(self.doubleSpinBox.value()))

    def doubleSpinBoxChange2(self):
        self.horizontalSlider_2.setValue(int(self.doubleSpinBox_2.value()))
    
    def doubleSpinBoxChange3(self):
        self.horizontalSlider_3.setValue(int(self.doubleSpinBox_3.value()))
    
    def doubleSpinBoxChange4(self):
        self.horizontalSlider_4.setValue(int(self.doubleSpinBox_4.value()))
    
    def doubleSpinBoxChange5(self):
        self.horizontalSlider_5.setValue(int(self.doubleSpinBox_5.value()))
    
    def doubleSpinBoxChange6(self):
        self.horizontalSlider_6.setValue(int(self.doubleSpinBox_6.value()))

#endregion
   
#region Menu Actions
    def poseViewShow(self,MainWindow):
        if not self.dockWidget_4.isVisible():
            self.dockWidget_4.setVisible(True)

    def update_label_text(self, text):
        self.lineEdit.setText(text)

    def newFile(self):
        print("new file")

    def openFile(self):
        print("open file")
        self.dialog.show()


    def saveFile(self):
        self.save_items()
        print("save file")

    def saveAs(self):
        pass

    def helpContent(self):
        print("help content")
    
    def about(self):
        box = QMessageBox()
        box.about(self.centralwidget, "About", "This is a program to control robot")
        # box.exec_()


    def connect(self):
        try:
            print("Connecting to robot...")
            global robot
            robot = RobotAction.RobotAction()
            self.connected_robot = True
            self.timer.start()
        except Exception as e:
            ret = QMessageBox.critical(self.centralwidget, "Error", "Connect to robot failed", QMessageBox.Ok)
            print(e)
        # print("connect")
        
    def disconnect(self):
        
        print("disconnect")
    

    def closeEvent(self, event):
        reply = QMessageBox.question(self.centralwidget, "Message", "Are you sure to quit?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
        if reply == QMessageBox.Yes:
            event.accept()
            self.close()
        else:
            event.ignore()
        

#endregion

#region Qscintilla methods

    def setup_tab_view_code(self):
        self.tab_code_view.setContentsMargins(0,0,0,0)
        self.tab_code_view.setTabsClosable(True)
        self.tab_code_view.setMovable(True)
        self.tab_code_view.setDocumentMode(True)
        self.tab_code_view.tabCloseRequested.connect(self.close_Tab)

    def close_Tab(self,index):
        self.tab_code_view.removeTab(index)
    def get_editors(self) -> QsciScintilla:
        
        #instance

        edittor = QsciScintilla()
        #encoding
        edittor.setUtf8(True)
        # Font
        edittor.setFont(self.centralwidget.font())
        
        # brace matching

        edittor.setBraceMatching(QsciScintilla.BraceMatch.SloppyBraceMatch)

        #indentation
        edittor.setIndentationGuides(True)
        edittor.setTabWidth(4)
        edittor.setIndentationsUseTabs(False)
        edittor.setAutoIndent(True)

        # auto complete
        # TODO: add autocomplete
        edittor.setAutoCompletionSource(QsciScintilla.AutoCompletionSource.AcsAll)
        edittor.setAutoCompletionThreshold(1)
        edittor.setAutoCompletionCaseSensitivity(False)
        # edittor.setAutoCompletionUseSingle(QsciScintilla.AutoCompletionUseSingle.AcusNever)
        #caret 
        # TODO: add caret settings

        edittor.setCaretForegroundColor(QColor("#decdc"))
        
        edittor.setEolMode(QsciScintilla.EolMode.EolWindows)
        edittor.setEolVisibility(False)
        edittor.setCaretLineVisible(True)
        edittor.setCaretWidth(2)
        edittor.setCaretLineBackgroundColor(QColor("#cdf1fe"))
        # edittor.setAutoCompletionWordSeparators(" .")
        edittor.setAutoCompletionReplaceWord(True)

        #lexer TODO: add lexer
        self.pylexer = QsciLexerPython() # there isa default for many languuages
        font = QFont("Consolas", 14)  # Replace with your desired font and size
        
        
        # Add words to the auto-completion list
        # edittor.setAutoCompletionWordList(["if", "else", "while", "for", "def", "class", "import"])
        
        # Create a QFont object for comments
        comment_font = QFont("Arial", 14)  # Adjust as needed
        self.pylexer.setDefaultFont(font)
        self.pylexer.setFont(font, QsciLexerPython.Comment)
        self.pylexer.setFont(font, QsciLexerPython.Keyword)
        self.pylexer.setFont(font, QsciLexerPython.HighlightedIdentifier)
        self.pylexer.setFont(font, QsciLexerPython.ClassName)
        self.pylexer.setFont(font, QsciLexerPython.FunctionMethodName)
        self.pylexer.setFont(font, QsciLexerPython.Identifier)
        self.pylexer.setFont(font, QsciLexerPython.Number)
        self.pylexer.setFont(font, QsciLexerPython.Operator)
        self.pylexer.setFont(font, QsciLexerPython.DoubleQuotedFString)
        self.pylexer.setFont(font, QsciLexerPython.DoubleQuotedString)
        self.pylexer.setFont(font, QsciLexerPython.SingleQuotedFString)
        self.pylexer.setFont(font, QsciLexerPython.DoubleQuotedString)


        # self.pylexer.setDefaultFont(self.centralwidget.font())
        edittor.setLexer(self.pylexer)

        # Api
        self.api = QsciAPIs(self.pylexer)
        for key in keyword.kwlist + dir(__builtins__):
            self.api.add(key)

        for _,name,_ in pkgutil.iter_modules():
            self.api.add(name)

        return edittor

    def set_new_tab(self,path: Path,filename:str, is_new_file = False):
        editor = self.get_editors()

        if is_new_file:
            if filename!="":
                self.tab_code_view.addTab(editor,filename)
                self.statusBar.setText(filename)
            else:
                self.tab_code_view.addTab(editor,"unitial")
                self.statusBar.setText("unitial")
            self.tab_code_view.setCurrentIndex(self.tab_code_view.count() - 1)
            self.current_file = None
            return
        if not path.is_file():
            return
        # if self.is_binary(path):
        #     self.statusBar.setText('Cannot Open Binary File')
        #     return
        
        # check if file already open
        if not is_new_file:
            for i in range(self.tab_code_view.count()):
                if self.tab_code_view.tabText(i) == path.name:
                    self.tab_code_view.setCurrentIndex(i)
                    self.current_file = path 
                    return
                

        # create new tab
        
        self.tab_code_view.addTab(editor,path.name)
        if not is_new_file:
            editor.setText(path.read_text())
        self.current_openfile.setText(f'File: %s' % path.name)
        self.current_file = path
        self.tab_code_view.setCurrentIndex(self.tab_code_view.count() - 1)
        
    def saveProgram(self):
        if self.current_file is None and self.tab_code_view.count() >0:
            self.save_as()
        
        editor = self.tab_code_view.currentWidget()
        self.current_file.write_text(editor.text())

        
    def save_as(self):
        pass
#endregion
    
#region trigger functions
    def manual_speed_change(self):
        self.var_manual_speed = self.manual_speed.text()
    def Auto_Speed_change(self):
        self.var_auto_speed = self.Auto_Speed.text()
    def manual_acc_change(self):
        self.var_manual_acc = self.manual_acc.text()
    def Auto_Acc_change(self):
        self.var_auto_acc = self.Auto_Acc.text()

    def Mode_Changed(self):
        print("Mode changed")
        if self.connected_robot == True:
            if self.radio_auto_mode.isChecked() == True:
                robot.Mode(0)
            elif self.radio_auto_mode.isChecked() == True:
                robot.Mode(1)
    
    def Drag_mode_changed(self):
        print("Drag Mode changed")
        if self.connected_robot == True:
            if self.radio_non_teaching_mode.isChecked() == True:
                robot.DragTeachSwitch(2)
            elif self.radio_teaching_mode.isChecked() == True:
                robot.DragTeachSwitch(1)
#endregion

#region Functions ------------------------
    def LoadCurrentPossition(self):
        try:
            if self.connected_robot == False: 
                return
            ret = robot.GetActualJointPosDegree(0)
            ret2 = robot.GetActualTCPPose(0)
            if ret[0] == 0:
                self.doubleSpinBox.setValue(ret[0])
                self.doubleSpinBox_2.setValue(ret[1])
                self.doubleSpinBox_3.setValue(ret[2])
                self.doubleSpinBox_4.setValue(ret[3])
                self.doubleSpinBox_5.setValue(ret[4])
                self.doubleSpinBox_6.setValue(ret[5])
            else:
                print("Error Code Joint: " + ret[1])

            if ret2[0] == 0:
                self.ToolX.setValue(ret2[0])
                self.ToolY.setValue(ret2[1])
                self.ToolZ.setValue(ret2[2])
                self.ToolRx.setValue(ret[3])
                self.ToolRy.setValue(ret[4])
                self.ToolRz.setValue(ret[5])
            else:
                print("Error Code TCPose: " + ret2[1])
        except Exception as e:
            print("Variable Error: " + str(e))

    def checkMode_jog(self):
        if self.Radiobtn_RX.isChecked():
            self.JogMode = JogMode.Rx
            print("Jog Mode: " + str(self.JogMode))
        elif self.Radiobtn_RY.isChecked():
            self.JogMode = JogMode.Ry
            print("Jog Mode: " + str(self.JogMode))
        elif self.Radiobtn_RZ.isChecked():
            self.JogMode = JogMode.Rz
            print("Jog Mode: " + str(self.JogMode))
        elif self.Radiobtn_TX.isChecked():
            self.JogMode = JogMode.Tx
            print("Jog Mode: " + str(self.JogMode))
        elif self.Radiobtn_TY.isChecked():
            self.JogMode = JogMode.Ty
            print("Jog Mode: " + str(self.JogMode))
        elif self.Radiobtn_TZ.isChecked():
            self.JogMode = JogMode.Tz
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J1.isChecked():
            self.JogMode = JogMode.J1
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J2.isChecked():
            self.JogMode = JogMode.J2
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J3.isChecked():
            self.JogMode = JogMode.J3
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J4.isChecked():
            self.JogMode = JogMode.J4
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J5.isChecked():
            self.JogMode = JogMode.J5
            print("Jog Mode: " + str(self.JogMode))
        elif self.radio_J6.isChecked():
            self.JogMode = JogMode.J6
            print("Jog Mode: " + str(self.JogMode))
        else:
            print("Exceptions")
            pass

    def Reference_FrameChanged(self):

        if self.Ref_Frame_Box.currentIndex() == 0:
            self.Ref_Frame = ReferenceFrame.Tool_Frame
            print(1)
        elif self.Ref_Frame_Box.currentIndex() == 1:
            self.Ref_Frame = ReferenceFrame.RobotBaseFrame
            print(2)
        elif self.Ref_Frame_Box.currentIndex() == 2:
            self.Ref_Frame = ReferenceFrame.Workpiece
            print(3)
        elif self.Ref_Frame_Box.currentIndex() == 3:
            self.Ref_Frame = ReferenceFrame.JointJog
            print(3)
        else:
            pass


    def Dial_Jog(self):
        print(self.Dial_button_jog.value())
    
    def jog_positive_start(self):

        if self.connected_robot == False:
            return
        # Jogging
        if self.JogMode == JogMode.J1:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=1,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)
        elif self.JogMode == JogMode.J2:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=2,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=340)
        elif self.JogMode == JogMode.J3:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=3,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=320)
        elif self.JogMode == JogMode.J4:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=4,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=340)
        elif self.JogMode == JogMode.J5:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=5,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)
        elif self.JogMode == JogMode.J6:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=6,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)

        # base coordinates

        elif self.JogMode == JogMode.Tx:
            robot.StartJOG(ref= self.Ref_Frame,nb = 1,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=1000)
        elif self.JogMode == JogMode.Ty:
            robot.StartJOG(ref= self.Ref_Frame,nb = 2,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=800)
        elif self.JogMode == JogMode.Tz:
            robot.StartJOG(ref= self.Ref_Frame,nb = 3,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=660)
        elif self.JogMode == JogMode.Rx:
            robot.StartJOG(ref= self.Ref_Frame,nb = 4,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)
        elif self.JogMode == JogMode.Ry:
            robot.StartJOG(ref= self.Ref_Frame,nb = 5,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)
        elif self.JogMode == JogMode.Rz:
            robot.StartJOG(ref= self.Ref_Frame,nb = 6,dir=1,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)

    def jog_positive_stop(self):
        if self.connected_robot == False:
            return
        if self.JogMode == JogMode.J1:
            robot.StopJOG(ref = 1)
        elif self.JogMode == JogMode.J2:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J3:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J4:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J5:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J6:
            robot.StopJOG(ref=1)

        # Base Coordiante

        else:
            robot.StopJOG(ref=self.Ref_Frame + 1)


    def jog_negative_start(self):
        if self.connected_robot == False:
            return
        if self.JogMode == JogMode.J1:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=1,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)
        elif self.JogMode == JogMode.J2:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=2,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=340)
        elif self.JogMode == JogMode.J3:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=3,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=320)
        elif self.JogMode == JogMode.J4:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=4,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=340)
        elif self.JogMode == JogMode.J5:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=5,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)
        elif self.JogMode == JogMode.J6:
            robot.StartJOG(ref= ReferenceFrame.JointJog,nb=6,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=350)

         # base coordinates

        elif self.JogMode == JogMode.Tx:
            robot.StartJOG(ref= self.Ref_Frame,nb = 1,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=1000)
        elif self.JogMode == JogMode.Ty:
            robot.StartJOG(ref= self.Ref_Frame,nb = 2,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=800)
        elif self.JogMode == JogMode.Tz:
            robot.StartJOG(ref= self.Ref_Frame,nb = 3,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=660)
        elif self.JogMode == JogMode.Rx:
            robot.StartJOG(ref= self.Ref_Frame,nb = 4,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)
        elif self.JogMode == JogMode.Ry:
            robot.StartJOG(ref= self.Ref_Frame,nb = 5,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)
        elif self.JogMode == JogMode.Rz:
            robot.StartJOG(ref= self.Ref_Frame,nb = 6,dir=0,vel=self.var_manual_speed,acc=self.var_manual_acc,max_dis=200)


    def jog_negative_stop(self):
        if self.connected_robot == False:
            return
        if self.JogMode == JogMode.J1:
            robot.StopJOG(ref = 1)
        elif self.JogMode == JogMode.J2:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J3:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J4:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J5:
            robot.StopJOG(ref=1)
        elif self.JogMode == JogMode.J6:
            robot.StopJOG(ref=1)
        
        else:
            robot.StopJOG(ref=self.Ref_Frame + 1)
#endregion
#region Timer
    def update_pose(self):
        current_time = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        # print(current_time)
        # self.LoadCurrentPossition()
#endregion

#region Gamer
    async def gamepad_thread(self):
        pygame.init()

        # Initialize the joystick module
        pygame.joystick.init()

        # Check for available joysticks
        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            print("No game controllers found.")
            return

        # Initialize the first joystick
        joystick = pygame.joystick.Joystick(0)
        joystick.init()

        # Main loop
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            # Read and print axis values
            num_axes = joystick.get_numaxes()
            for i in range(num_axes):
                axis_value = joystick.get_axis(i)
                print(f"Axis {i}: {axis_value:.2f}")

            pygame.time.wait(50)  # Add a short delay to avoid high CPU usage
        
        pygame.quit()

    def create_gamepad(self):

        run_game_pad = self.gamepad_thread()
        asyncio.run(run_game_pad)
# endregion 
  
#region Excute Script
    def run_script(self):
        editor = self.tab_code_view.currentWidget()

        script = editor.text()
        try:
            exec(script)
        except Exception as e:
            print("Error:", e)
#endregion
# if __name__ == '__main__':
#     app = QApplication(sys.argv)
#     window = MainWindow(None)
#     window.show()
#     sys.exit(app.exec_())

#region ENUM
class JogMode(Enum):
    Tx = 1
    Ty = 2
    Tz = 3
    Rx = 4
    Ry = 5
    Rz = 6
    J1 = 7
    J2 = 8
    J3 = 9
    J4 = 10
    J5 = 11
    J6 = 12

class ReferenceFrame(Enum):
    Tool_Frame = 4
    JointJog = 0
    RobotBaseFrame = 2
    Workpiece = 8
#endregion