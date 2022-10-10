#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'robot_kontrol.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMainWindow, QApplication, QWidget, QPushButton, QAction, QLineEdit, QMessageBox, QLabel, QComboBox
from PyQt5.QtGui import QImage, QPixmap, QPainter, QPen, QBrush, QColor, QFont, QIcon

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan

from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import time


class Ui_MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(Ui_MainWindow, self).__init__()
        self.initializeUi()

    def initializeUi(self):
        '''
        setup app gui
        '''
        self.setObjectName("MainWindow")  # set the window name
        # set the window size (width, height)
        self.setMinimumSize(QtCore.QSize(1000, 600))
        self.setMaximumSize(QtCore.QSize(
            1920, 800))  # set the maximum size
        self.setWindowTitle("Robot Kontrol İnterface")

        self.setUpMainWindow()  # Create and arrange widgets in the main window

        self.createActions()  # Create the application's menu actions,if you want to add a menu to the main window,you must create the actions first

        self.createMenu()  # Create the application's menu bar
        self.createToolBar()  # Create the application's toolbar,
        self.createStatusBar()  # Create the application's status bar
        # self.ros_connect()  # connect to ros
        # Create the application's dock widget ,it will be added to the left side of the main window
        self.createdockWidgetLeft()
        self.createdockwidgetRight()
        self.teleop_widget()

        self.retranslateUi()  # translate the ui elements to the current language
        # connect the signals and slots of the ui elements
        QtCore.QMetaObject.connectSlotsByName(self)

        self.show()  # Show the main windo

    def createStatusBar(self):
        """Create the application's status bar."""
        self.statusBar().showMessage("Ready")
        self.statusBar().setStyleSheet(
            "QStatusBar{background:rgb(10,250,10); color:rgb(20,20,20);}")
        # set the status bar message
        self.statusBar().setFont(QtGui.QFont("Arial", 12, QtGui.QFont.Bold))

    def createToolBar(self):
        """Create the application's toolbar."""
        toolbar_obj = QtWidgets.QToolBar()
        toolbar_obj.setObjectName("toolBar")
        # set the toolbar style to be text under icon
        toolbar_obj.setToolButtonStyle(QtCore.Qt.ToolButtonTextUnderIcon)
        # set the toolbar icon size
        toolbar_obj.setIconSize(QtCore.QSize(32, 32))
        toolbar_obj.setMovable(False)  # set the toolbar to be movable
        toolbar_obj.setStatusTip("Toolbar")  # set the toolbar status tip
        toolbar_obj.setStyleSheet(
            "QToolBar { border: 0px dashed green; border-radius: 2px; background: turquoise; }")
        toolbar_obj.addAction(self.quit_act)
        toolbar_obj.addAction(self.detect_act)
        toolbar_obj.addAction(self.line_trace_act)

        toolbar_obj.addSeparator()  # add a separator
        # create toogle button for lift
        # create toolbutton for the action
        toolbutton = QtWidgets.QToolButton()
        # set the action to the toolbutton
        toolbutton.setDefaultAction(self.toggle_lift_act)
        toolbutton.setObjectName("toolbuttonlift")
        # add the toolbutton to the toolbar
        toolbar_obj.addWidget(toolbutton)

        self.addToolBar(toolbar_obj)

    def setUpMainWindow(self):
        # ros connection topic
        self.cmd_topic = "/cmd_vel"
        self.odom_topic = "/odom"
        self.camera_topic = "/camera/rgb/image_raw"

        # create widget for central widget
        central_widget = QWidget()
        central_widget.setObjectName("central_widget")
        self.setCentralWidget(central_widget)

        # create status bar for main window
        # self.statusBar(QtWidgets.QStatusBar())

        self.gridLayout_2 = QtWidgets.QGridLayout(
            central_widget)  # create a grid layout
        self.gridLayout_2.setObjectName(
            "gridLayout_2")  # set the grid layout name

        font = QtGui.QFont()  # create a font
        font.setBold(True)
        font.setWeight(75)

        # create a label for the speed display
        self.label_speed = QtWidgets.QLabel()
        self.label_speed.setFont(font)  # set the font for the label
        self.label_speed.setObjectName("label_speed")

        # add the label to the main grid layout
        # add the label to the grid layout (row, column, rowspan, columnspan)
        self.gridLayout_2.addWidget(self.label_speed, 4, 1, 1, 1)

        # create a form layout for the speed display
        self.formLayout = QtWidgets.QFormLayout()
        self.formLayout.setObjectName("formLayout")

        self.label_linear = QtWidgets.QLabel()
        self.label_linear.setObjectName("label_linear")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.LabelRole, self.label_linear)
        self.line_lineer = QtWidgets.QLineEdit()
        self.line_lineer.setReadOnly(True)
        self.line_lineer.setObjectName("line_lineer")
        self.formLayout.setWidget(
            0, QtWidgets.QFormLayout.FieldRole, self.line_lineer)
        self.label_angular = QtWidgets.QLabel()
        self.label_angular.setObjectName("label_angular")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.LabelRole, self.label_angular)
        self.line_angular = QtWidgets.QLineEdit()
        self.line_angular.setReadOnly(True)
        self.line_angular.setObjectName("line_angular")
        self.formLayout.setWidget(
            1, QtWidgets.QFormLayout.FieldRole, self.line_angular)

        # add the form layout to the main grid layout
        self.gridLayout_2.addLayout(self.formLayout, 5, 1, 1, 1)

        # # add the grid layout to the main window grid layout2 (the central widget grid layout)
        # self.gridLayout_2.addLayout(self.gridLayout, 3, 1, 1, 2)

        self.label_position = QtWidgets.QLabel()
        self.label_position.setFont(font)
        self.label_position.setObjectName("label_position")
        self.gridLayout_2.addWidget(self.label_position, 4, 2, 1, 1)
        self.formLayout_2 = QtWidgets.QFormLayout()
        self.formLayout_2.setObjectName("formLayout_2")
        self.label_x = QtWidgets.QLabel()
        self.label_x.setObjectName("label_x")
        self.formLayout_2.setWidget(
            0, QtWidgets.QFormLayout.LabelRole, self.label_x)
        self.line_x = QtWidgets.QLineEdit()
        self.line_x.setReadOnly(True)
        self.line_x.setObjectName("line_x")
        self.formLayout_2.setWidget(
            0, QtWidgets.QFormLayout.FieldRole, self.line_x)
        self.label_y = QtWidgets.QLabel()
        self.label_y.setObjectName("label_y")
        self.formLayout_2.setWidget(
            1, QtWidgets.QFormLayout.LabelRole, self.label_y)
        self.line_y = QtWidgets.QLineEdit()
        self.line_y.setReadOnly(True)
        self.line_y.setObjectName("line_y")
        self.formLayout_2.setWidget(
            1, QtWidgets.QFormLayout.FieldRole, self.line_y)
        self.gridLayout_2.addLayout(self.formLayout_2, 5, 2, 1, 1)

        self.menubar = QtWidgets.QMenuBar(self)
        # menubar'ın koordinatları (x,y,width,height)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 840, 22))
        # set the menubar name and object name for the menubar (object name is used for finding the object)
        self.menubar.setObjectName("menubar")

        self.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(self)
        self.statusbar.setObjectName("statusbar")
        self.setStatusBar(self.statusbar)

        self.imglabel()
        # self.topicslayout()

    def createActions(self):
        """Create the application's menu actions."""
        # Create actions for File menu
        # QIcon is a class that provides access to the application's icon. QAction is a class that provides an action in a menu or toolbar.
        self.quit_act = QAction(QIcon("images/exit.png"), "Quit")
        # Set the keyboard shortcut for the action
        self.quit_act.setShortcut("Ctrl+Q")
        # Set the status tip for the action,it displays text in the status bar when the mouse cursor is over the menu item
        self.quit_act.setStatusTip("Quit program")
        # Connect the triggered signal to the close() method
        self.quit_act.triggered.connect(self.close)

        # actions for ros connection
        self.connect_act = QAction(QIcon("images/connect.png"), "Ros Connect")
        self.connect_act.setShortcut("Ctrl+R+C")
        self.connect_act.setStatusTip("Connecting to ROS")
        self.connect_act.triggered.connect(self.ros_connect)
        # Create actions for Edit menu

        self.edit_action = QAction(QIcon("images/edit.png"), "Edit")
        self.edit_action.setShortcut("Ctrl+E")
        self.edit_action.setStatusTip("Editing the text")

        # Create actions for View menu
        # Set the action to be checkable (i.e. it can be checked or unchecked),it means that the action will be displayed as a checkable menu item.
        self.full_screen_act = QAction("Full Screen", checkable=True)
        # Set the keyboard shortcut for the action
        self.full_screen_act.setShortcut("F11")
        # Set the status tip for the action
        self.full_screen_act.setStatusTip("Switch to full screen mode")
        # set trigger for the action,when the action is triggered,the method will be called
        self.full_screen_act.triggered.connect(self.switchToFullScreen)

        # action for detect and line traking
        self.line_trace_act = QAction(
            QIcon("images/line_trace.png"), "Line Trace")
        self.line_trace_act.setShortcut("Ctrl+L+T")
        self.line_trace_act.setStatusTip("Line Trace start")
        self.line_trace_act.triggered.connect(self.line_trace)

        self.detect_act = QAction(QIcon("images/detect.png"), "Detect Line")
        self.detect_act.setShortcut("Ctrl+L+D")
        self.detect_act.setStatusTip("Detect Line start")
        self.detect_act.triggered.connect(self.detect)

        self.normal_screen_act = QAction("Normal Screen", checkable=True)
        self.normal_screen_act.setShortcut("F11")
        self.normal_screen_act.setStatusTip("Switch to normal screen mode")
        self.normal_screen_act.triggered.connect(self.switchToNormalScreen)

        # create actions for save appearance
        self.save_layout_act = QAction(
            QIcon("images/save.png"), "Save Layout")
        self.save_layout_act.setShortcut("Ctrl+S+L")
        self.save_layout_act.setStatusTip("Save the current layout")
        self.save_layout_act.triggered.connect(self.save_layout)

        self.load_layout_act = QAction(
            QIcon("images/load.png"), "Load Layout")
        self.load_layout_act.setShortcut("Ctrl+L+A")
        self.load_layout_act.setStatusTip("Load the current layout")
        self.load_layout_act.triggered.connect(self.load_layout)

        # create toogle action show or hide dock widgets
        self.toggle_action_explorer = QAction(
            "show Topic explorer", checkable=True)
        self.toggle_action_explorer.setShortcut("Ctrl+T")
        self.toggle_action_explorer.setStatusTip("show or hide topic explorer")
        self.toggle_action_explorer.triggered.connect(
            self.toggle_topic_explorer)

        self.toggle_action_topiclist = QAction(
            "show Topic list", checkable=True)
        self.toggle_action_topiclist.setShortcut("Ctrl+shift+T")
        self.toggle_action_topiclist.setStatusTip("show or hide topic list")
        self.toggle_action_topiclist.triggered.connect(
            self.toggle_topic_list_dockwidget)

        self.toggle_action_cont = QAction("show Control", checkable=True)
        self.toggle_action_cont.setShortcut("Ctrl+alt+C")
        self.toggle_action_cont.setStatusTip("show or hide control")
        self.toggle_action_cont.triggered.connect(
            self.toggle_control_dockwidget)

        self.toggle_action_image = QAction("show Image", checkable=True)
        self.toggle_action_image.setShortcut("Ctrl+shift+I")
        self.toggle_action_image.setStatusTip("show or hide image")
        self.toggle_action_image.triggered.connect(
            self.toggle_image_dockwidget)

        # create lift on /off toggle action
        self.toggle_lift_act = QAction("Lift On", checkable=True)
        self.toggle_lift_act.setShortcut("Ctrl+L+O")
        self.toggle_lift_act.setStatusTip("Lift On runnig")
        self.toggle_lift_act.triggered.connect(
            self.toggle_lift)  # connect the signal to the method

    def toggle_lift(self):
        toolbuttonlift = self.findChild(
            QtWidgets.QToolButton, "toolbuttonlift")
        if self.toggle_lift_act.isChecked():
            self.lift_on()
            # set style toolbuttonlift object
            self.toggle_lift_act.setText("Lift On")
            toolbuttonlift.setStyleSheet(
                "QToolButton { background-color: rgb(0, 255, 0); }")
        else:
            self.lift_off()
            self.toggle_lift_act.setChecked(False)
            self.toggle_lift_act.setText("Lift Off")
            toolbuttonlift.setStyleSheet(
                "QToolButton { background-color: rgb(255, 0, 0); }")

    def lift_on(self):
        pass

    def liff_off(self):
        pass

    def toggle_image_dockwidget(self):
        pass
        # if self.toggle_action_4.isChecked():
        #     # kself.image_dockwidget.show()
        # else:
        #     self.image_dockwidget.hide()

    def toggle_control_dockwidget(self):
        if self.toggle_action_cont.isChecked():
            self.dock_control_widget.show()
        else:
            self.dock_control_widget.hide()

    def detect(self):
        pass

    def line_trace(self):
        pass

    def toggle_topic_list_dockwidget(self):
        if self.toggle_action_topiclist.isChecked():
            self.dockWidgetRight.show()
        else:
            self.dockWidgetRight.hide()

    def toggle_topic_explorer(self):
        if self.toggle_action_explorer.isChecked():
            self.dockWidgetLeft.show()
        else:
            self.dockWidgetLeft.hide()

    def switchToFullScreen(self):
        """Switch to full screen mode."""
        if self.isFullScreen():  # if the window is already in full screen mode
            self.showNormal()  # show the window in normal mode
        else:
            self.showFullScreen()  # show the window in full screen mode

    def switchToNormalScreen(self):
        """Switch to normal screen mode."""
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showNormal()

    def save_layout(self):
        """Save the current layout settings."""
        self.settings = QtCore.QSettings(
            "settings.ini", QtCore.QSettings.IniFormat)  # create a settings object with the file name and the format
        print(self.settings.fileName())  # print the file name
        # save the window size
        self.settings.setValue("MainWindow/Size", self.size())
        print(self.settings.value("MainWindow/Size"))  # print the window size
        # save the window position
        self.settings.setValue("MainWindow/Position", self.pos())
        # print the window position
        print(self.settings.value("MainWindow/Position"))
        # save the window state
        self.settings.setValue("MainWindow/State", self.saveState())
        # print the window state
        print(self.settings.value("MainWindow/State"))
        # save the window full screen status
        self.settings.setValue("MainWindow/FullScreen", self.isFullScreen())
        # save the window maximized status
        self.settings.setValue("MainWindow/Maximized", self.isMaximized())
        # save the window minimized status
        self.settings.setValue("MainWindow/Minimized", self.isMinimized())
        # save the window visible status
        self.settings.setValue("MainWindow/Visible", self.isVisible())
        self.settings.setValue("MainWindow/WindowTitle",
                               self.windowTitle())  # save the window title
        # print the window title
        print(self.settings.value("MainWindow/WindowTitle"))
        # save the window icon
        self.settings.setValue("MainWindow/WindowIcon",
                               self.windowIcon().pixmap(QtCore.QSize(16, 16)))
        # save the window flags (window state) (minimized, maximized, etc.)
        self.settings.setValue("MainWindow/WindowFlags", self.windowFlags())
        # save the window modality (modal, non-modal)
        self.settings.setValue(
            "MainWindow/WindowModality", self.windowModality())
        print(self.settings.value("MainWindow/WindowModality"))
        # save the window state (minimized, maximized, etc.)
        self.settings.setValue("MainWindow/WindowState", self.windowState())
        # print the window state
        print(self.settings.value("MainWindow/WindowState"))
        # save the window type (dialog, main window, etc.)
        self.settings.setValue("MainWindow/WindowType", self.windowType())
        # print the window type
        print(self.settings.value("MainWindow/WindowType"))
        # save the window geometry
        self.settings.setValue("MainWindow/Geometry", self.saveGeometry())

        # save the dock widget area
        self.settings.setValue("MainWindow/DockWidgetArea",
                               self.dockWidgetArea(self.dockWidgetRight))
        # save the dock widget area
        self.settings.setValue("MainWindow/DockWidgetArea",
                               self.dockWidgetArea(self.dockWidgetLeft))

        # save the splitter state
        self.settings.setValue("MainWindow/SplitterState",
                               self.splitter.saveState())

    def load_layout(self):
        """Load the saved layout settings."""
        print("self.settings.all keys", self.settings.allKeys()
              )  # print all the keys in the settings object

    def switchToFullScreen(self, state):
        """Switch to full screen mode."""
        if state:
            self.showFullScreen()
        else:
            self.showNormal()

    def createMenu(self):
        """Create the application's menu bar."""
        menu_bar = self.menuBar()  # create a menu bar
        # Set the menu bar to use native style (i.e. no OS-specific styling)
        menu_bar.setNativeMenuBar(False)
        # Create File menu and add actions
        file_menu = menu_bar.addMenu("File")  # Add a menu to the menu bar
        file_menu.addAction(self.quit_act)  # Add an action to the menu

        # Create edit menu and add actions
        edit_menu = menu_bar.addMenu("Edit")  # Add a menu to the menu bar

        # Create Ros menu and add actions
        ros_menu = menu_bar.addMenu("Ros")  # Add a menu to the menu bar
        ros_connect_menu = ros_menu.addMenu("Connect")
        ros_connect_menu.addAction(self.connect_act)

        # Create Tools menu and add actions
        tools_menu = menu_bar.addMenu("Tools")  # Add a menu to the menu bar
        toggle_menu = tools_menu.addMenu(" Show/Hide Explorer")
        # Add an action for dock widget topic explorer
        toggle_menu.addAction(self.toggle_action_explorer)
        # Add an action for dock widget right
        toggle_menu.addAction(self.toggle_action_topiclist)
        # Add an action for dock widget control
        toggle_menu.addAction(self.toggle_action_cont)

        # Create View menu, Appearance submenu and add actions
        view_menu = menu_bar.addMenu("View")
        appearance_submenu = view_menu.addMenu("Appearance")
        appearance_submenu.addAction(self.full_screen_act)

        # save layout appearance
        save_layout_menu = menu_bar.addMenu("Save Layout")
        save_layout_menu.addAction(self.save_layout_act)

        # Create Settings menu and add actions
        settings_menu = menu_bar.addMenu(
            "Settings")  # Add a menu to the menu bar

        # Create Run menu and add actions
        run_menu = menu_bar.addMenu("Run")  # Add a menu to the menu bar

        # Create Terminals menu and add actions
        terminals_menu = menu_bar.addMenu(
            "Terminals")  # Add a menu to the menu bar

        # Create Help menu and add actions
        help_menu = menu_bar.addMenu("Help")  # Add a menu to the menu bar

    def createdockwidgetRight(self):
        """Create a dock widget on the right side of the main window."""

        # get the toplist
        topic_list = self.get_topic_list()
        self.dockWidgetRight = QtWidgets.QDockWidget("Topic List", self)
        # SET ALL THE DOCK WIDGETS TO BE ALLOWED TO BE FLOATED
        # self.dockWidgetRight.setFloating(True)
        # SET ALL THE DOCK WIDGETS ALLQWED AREES TO BE ALLOWED TO BE FLOATED
        self.dockWidgetRight.setAllowedAreas(QtCore.Qt.AllDockWidgetAreas)
        self.dockWidgetRight.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable)

        # create acontrol widget for the dock widget
        listwidgetcontainer = QtWidgets.QWidget()
        listwidgetcontainer.setObjectName("listwidgetcontainer")
        # create a layout for the control widget
        v_layout = QtWidgets.QVBoxLayout(listwidgetcontainer)
        self.listWidget = QtWidgets.QListWidget()
        self.listWidget.setObjectName("listWidget")
        # set the selection mode to extended selection mode
        self.listWidget.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection)
        # set the selection behavior to select rows instead of single items
        self.listWidget.setSelectionBehavior(
            QtWidgets.QAbstractItemView.SelectRows)
        # set the alternating row colors to true to make the rows alternate colors
        self.listWidget.setAlternatingRowColors(True)
        # set the sorting to true to enable sorting
        self.listWidget.setSortingEnabled(True)
        # add the topics to the list widget
        self.listWidget.addItems(topic_list)
        # connect the item clicked signal to the topic_selected method
        self.listWidget.itemClicked.connect(self.topic_selected)
        v_layout.addWidget(self.listWidget)

        self.dockWidgetRight.setWidget(listwidgetcontainer)
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea, self.dockWidgetRight)
        if self.toggle_action_topiclist.isChecked():
            self.dockWidgetRight.show()
        else:
            self.dockWidgetRight.hide()

        # toggle the dock widget

    def topic_selected(self):
        pass

    def createdockWidgetLeft(self):
        """Create a dock widget."""

        topic_list = self.get_topic_list()  # get the topic list from the server

        # create a dock widget  object for below widgets
        self.dockWidgetLeft = QtWidgets.QDockWidget()
        self.dockWidgetLeft.setWindowTitle("Topics Tools")
        self.dockWidgetLeft.setObjectName("dockWidget")
        # set the dock widget movable
        self.dockWidgetLeft.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable)
        # set the dock widget area to left or right
        self.dockWidgetLeft.setAllowedAreas(
            QtCore.Qt.DockWidgetArea.AllDockWidgetAreas)
        self.dockWidgetLeft.setFloating(True)

        # set the dock widget's style sheet
        self.dockWidgetLeft.setStyleSheet(
            "QDockWidget { border: 1px solid red; border-radius: 4px; background-color :  yellow }")

        # create a label object in dock widget for command topic
        cmd_topiclabel = QtWidgets.QLabel()
        cmd_topiclabel.setObjectName("cmd_topiclabel")
        cmd_topiclabel.setText("Command Topic")

        # create a line edit object in dock widget for command topic
        self.line_cmd_topic = QtWidgets.QLineEdit()
        # self.cmd_topic.resize(200, 20)  # width,height of the line edit
        self.line_cmd_topic.setObjectName("cmd_topic")
        self.line_cmd_topic.setText(self.cmd_topic)
        self.line_cmd_topic.textChanged.connect(self.cmd_topic_changed)

        # create a label object in dock widget for camera topic
        camera_topiclabel = QtWidgets.QLabel()
        camera_topiclabel.setText("Camera Topic")  # set the label text
        # camera_topiclabel.resize(200, 20)  # width,height of the label
        camera_topiclabel.setObjectName(
            "camera_topiclabel")  # set the label object name

        self.line_camera_topic = QtWidgets.QLineEdit()
        self.line_camera_topic.textChanged.connect(self.camera_topic_changed)
        # self.camera_topic.resize(200, 20)  # width,height
        self.line_camera_topic.setText(self.camera_topic)

        odom_topiclabel = QtWidgets.QLabel()
        odom_topiclabel.setText("Odom Topic")

        self.line_odom_topic = QtWidgets.QLineEdit()
        self.line_odom_topic.textChanged.connect(self.odom_topic_changed)
        self.line_odom_topic.setText(self.odom_topic)
        # self.odom_topic.resize(200, 20)  # width,height

        # find expre

        for t in topic_list:
            if "cmd_vel" in t and t.endswith("/cmd_vel"):
                self.line_cmd_topic.setText(t)

            elif "/image_raw" in t and t.endswith("/image_raw"):
                self.line_camera_topic.setText(t)
            elif "odom" in t and t.endswith("/odom"):
                self.line_odom_topic.setText(t)

        # create layout for dock widget
        dock_layout = QtWidgets.QVBoxLayout()
        dock_layout.addWidget(cmd_topiclabel)
        # add a stretch to the layout to make the layout look better
        dock_layout.addStretch(1)
        dock_layout.addWidget(self.line_cmd_topic)
        dock_layout.addStretch(1)
        dock_layout.addWidget(camera_topiclabel)
        dock_layout.addStretch(1)
        dock_layout.addWidget(self.line_camera_topic)
        dock_layout.addStretch(1)
        dock_layout.addWidget(odom_topiclabel)
        dock_layout.addStretch(1)
        dock_layout.addWidget(self.line_odom_topic)
        dock_layout.addStretch(1)
        # dock_layout.setAlignment(QtCore.Qt.AlignTop)

        # create a widget container for dock widget
        dock_conteiner = QtWidgets.QWidget()
        dock_conteiner.setLayout(dock_layout)
        self.dockWidgetLeft.setWidget(dock_conteiner)

        # add the dock widget to the main window at the left side
        self.addDockWidget(QtCore.Qt.LeftDockWidgetArea, self.dockWidgetLeft)
        if self.toggle_action_topiclist.isChecked():
            self.dockWidgetLeft.show()
        else:
            self.dockWidgetLeft.hide()

    def teleop_widget(self):

        font = QtGui.QFont()  # create a font
        font.setBold(True)
        font.setWeight(75)
        # create grid layout for the control buttons
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")

        self.button_right = QtWidgets.QPushButton()
        self.button_right.setObjectName("button_right")
        self.gridLayout.addWidget(self.button_right, 1, 3, 1, 1)

        self.button_stop = QtWidgets.QPushButton()
        self.button_stop.setObjectName("button_stop")
        self.gridLayout.addWidget(self.button_stop, 1, 2, 1, 1)

        self.button_left = QtWidgets.QPushButton()
        self.button_left.setObjectName("button_left")
        self.gridLayout.addWidget(self.button_left, 1, 1, 1, 1)

        self.button_forward = QtWidgets.QPushButton()
        self.button_forward.setObjectName("button_forward")
        self.gridLayout.addWidget(self.button_forward, 0, 2, 1, 1)

        self.button_bacward = QtWidgets.QPushButton()
        self.button_bacward.setObjectName("button_bacward")
        self.gridLayout.addWidget(self.button_bacward, 2, 2, 1, 1)

        # create container widget for the control buttons
        self.container_control_widget = QWidget()
        self.container_control_widget.setObjectName("container_control_widget")
        self.container_control_widget.setLayout(self.gridLayout)

        # create a dock widget for the control buttons
        self.dock_control_widget = QtWidgets.QDockWidget("Control")
        self.dock_control_widget.setObjectName("dock_control_widget")
        self.dock_control_widget.setFeatures(
            QtWidgets.QDockWidget.DockWidgetMovable)
        self.dock_control_widget.setFloating(True)
        self.dock_control_widget.setWidget(self.container_control_widget)

        # add the dock widget to the main window
        self.addDockWidget(QtCore.Qt.RightDockWidgetArea,
                           self.dock_control_widget)
        if self.toggle_action_cont.isChecked():

            self.dock_control_widget.show()
        else:
            self.dock_control_widget.hide()

    # odom topic changed
    def odom_topic_changed(self, text):
        self.odom_topic = text
        print("odom topic changed to: ", self.odom_topic)

    # camera topic changed
    def camera_topic_changed(self, text):
        self.camera_topic = text
        print("camera topic changed to: ", self.camera_topic)

    # cmd topic changed
    def cmd_topic_changed(self, text):
        self.cmd_topic = text
        print("cmd topic changed to: ", self.cmd_topic)

    def imglabel(self):
        self.imglabel = QtWidgets.QLabel()
        self.imglabel.setObjectName("imglabel")
        # load the image from the camera
        self.foto = np.zeros((480, 640, 3), np.uint8)
        pixmap = self.img_to_pixmap(self.foto)
        self.imglabel.setPixmap(pixmap)
        self.imglabel.setScaledContents(True)
        self.gridLayout_2.addWidget(self.imglabel, 1, 1, 1, 2)

    def img_to_pixmap(self, cvimg):
        img = cv2.cvtColor(cvimg, cv2.COLOR_BGR2RGB)  # bgr -> rgb
        h, w, c = img.shape  # h: height, w: width, c: channel
        # 3 * w: 3 channels
        image = QImage(img, w, h, 3 * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        return pixmap

    def retranslateUi(self):
        _translate = QtCore.QCoreApplication.translate
        self.setWindowTitle(_translate(
            "MainWindow", "Robot Kontrol Arayüzü"))
        self.label_speed.setText(_translate("MainWindow", "Speed İndicator"))
        self.label_linear.setText(_translate(
            "MainWindow", "linear speed (m/s) :"))
        self.label_angular.setText(_translate(
            "MainWindow", "Angular speed (rad/s) :"))

        self.button_right.setText(_translate("MainWindow", "Right"))
        self.button_stop.setText(_translate("MainWindow", "Stop"))
        self.button_left.setText(_translate("MainWindow", "Left"))
        self.button_forward.setText(_translate("MainWindow", "Forward"))
        self.button_bacward.setText(_translate("MainWindow", "Backward"))
        self.label_position.setText(_translate("MainWindow", "Robot Position"))
        self.label_x.setText(_translate("MainWindow", "Position X (m):"))
        self.label_y.setText(_translate("MainWindow", "Position Y (m):"))

    def topicslayout(self):
        topics = self.get_topic_list()

        self.topicslabel = QtWidgets.QLabel()
        self.topicslabel.setObjectName("topicslabel")
        self.topicslabel.setText("Topics List")
        self.comboBoxtopic = QComboBox()
        self.comboBoxtopic.setObjectName("comboBoxtopic")
        self.comboBoxtopic.addItems(topics)
        self.comboBoxtopic.currentIndexChanged.connect(self.topic_changed)

        self.formLayout_3 = QtWidgets.QFormLayout()
        self.formLayout_3.setObjectName("formLayout_3")
        self.formLayout_3.addRow(self.topicslabel, self.comboBoxtopic)
        self.gridLayout_2.addLayout(self.formLayout_3, 0, 1, 1, 1)

    def topic_changed(self):
        self.topic = self.comboBoxtopic.currentText()
        print(self.topic)

    def check_ros_master(self):
        try:
            rospy.get_master().getSystemState()  # check if ros master is running,
            # if not, throw an exception
            return True
            self.setStatusTip("ROS Master is running")
            self.statusBar().setStyleSheet("QStatusBar{background:green;}")

        except:
            return False
            self.setStatusTip("ROS Master is not running")
            self.statusBar().setStyleSheet("QStatusBar{background:red;}")

    def get_topic_list(self):
        topic_list = []
        # control  roscore is running
        if self.check_ros_master():
            # get the list of topics from the master
            master = rospy.get_master()
            topic_list = master.getTopicTypes()  # get the list of topics
            print(topic_list)
            topic_list = [x[0] for x in topic_list[2]]

            topic_list.sort()
            return topic_list
        else:
            # for topic in rospy.get_published_topics():
            #     topic_list.append(topic[0])
            return topic_list

    def ros_disconnect(self):
        rospy.signal_shutdown("GUI closed")
        rospy.wait_for_shutdown()
        print("GUI closed")

    def ros_connect(self):
        # create a ROS node

        # CHECK IF ROS MASTER IS RUNNING
        if self.check_ros_master():
            # create a ROS node
            rospy.init_node("robot_control_interface")

            self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)
            self.cmd_msg = Twist()

            rospy.Subscriber(self.cmd_topic, Twist, self.cmd_callback)
            rospy.Subscriber(self.odom_topic, Odometry, self.odomCallback)
            rospy.Subscriber(self.camera_topic, Image, self.cameraCallback)
            self.bridge = CvBridge()

            self.width = 480
            self.height = 400

            self.button_stop.clicked.connect(self.stop)
            self.button_forward.clicked.connect(self.goforward)
            self.button_bacward.clicked.connect(self.gobackward)
            self.button_left.clicked.connect(self.turnleft)
            self.button_right.clicked.connect(self.turnright)

            self.line_angular.setText(str(0.0))
            self.line_lineer.setText(str(0.0))
            self.line_x.setText(str(0.0))
            self.line_y.setText(str(0.0))
            # create a timer to update the GUI
            self.timer = rospy.Timer(rospy.Duration(2), self.update_gui)
            # set the status tip
            self.setStatusTip("ROS Nodes is running")
            self.statusBar().setStyleSheet("QStatusBar{background:green;}")
        else:
            self.setStatusTip("ROS Master is not running")
            self.statusBar().setStyleSheet("QStatusBar{background:red;}")
            # create Qdialog to show error message
            diag = QtWidgets.QDialog()  # create a dialog
            # set the title of the dialog
            diag.setWindowTitle("Ros Connect Error")
            diag.setFixedSize(300, 100)  # set the size of the dialog
            # set the modality of the dialog
            diag.setWindowModality(QtCore.Qt.ApplicationModal)
            diag.setModal(True)  # set the modal of the dialog
            # set the background color of the dialog
            diag.setStyleSheet("background-color: rgb(255, 0, 0);")
            # show the dialog
            diag.show()

    def update_gui(self, event):
        self.line_cmd_topic.setText(self.cmd_topic)
        self.line_odom_topic.setText(self.odom_topic)
        self.line_camera_topic.setText(self.camera_topic)

    def cmd_callback(self, cmd):
        self.line_angular.setText(str(cmd.angular.z))
        self.line_lineer.setText(str(cmd.linear.x))

    def cameraCallback(self, img):
        cvimg = self.bridge.imgmsg_to_cv2(img, "bgr8")
        cvimg = cv2.resize(cvimg, (self.width, self.height))
        self.imglabel.setPixmap(self.img_to_pixmap(cvimg))

    def odomCallback(self, mesaj):
        self.line_x.setText(str(round(mesaj.pose.pose.position.x, 4)))
        self.line_y.setText(str(round(mesaj.pose.pose.position.y, 4)))

    def stop(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.0
        self.pub.publish(self.cmd_msg)

    def goforward(self):
        self.cmd_msg.linear.x = 1.0
        self.cmd_msg.angular.z = 0.0
        self.pub.publish(self.cmd_msg)

    def gobackward(self):
        self.cmd_msg.linear.x = -1.0
        self.cmd_msg.angular.z = 0.0
        self.pub.publish(self.cmd_msg)

    def turnleft(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = 0.5
        self.pub.publish(self.cmd_msg)

    def turnright(self):
        self.cmd_msg.linear.x = 0.0
        self.cmd_msg.angular.z = -0.5
        self.pub.publish(self.cmd_msg)


if __name__ == "__main__":
    import sys
    app = QtWidgets.QApplication(sys.argv)
    ui_main = Ui_MainWindow()
    sys.exit(app.exec_())
