# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainUI.ui'
##
## Created by: Qt User Interface Compiler version 6.7.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QLabel, QPlainTextEdit, QPushButton,
    QSizePolicy, QWidget)

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(1300, 720)
        self.image_label = QLabel(guiDlg)
        self.image_label.setObjectName(u"image_label")
        self.image_label.setGeometry(QRect(0, 0, 640, 480))
        self.image_label.setScaledContents(True)
        self.segmented_image_label = QLabel(guiDlg)
        self.segmented_image_label.setObjectName(u"segmented_image_label")
        self.segmented_image_label.setGeometry(QRect(650, 0, 640, 480))
        self.segmented_image_label.setScaledContents(True)
        self.image_coords_label = QLabel(guiDlg)
        self.image_coords_label.setObjectName(u"image_coords_label")
        self.image_coords_label.setGeometry(QRect(10, 490, 181, 17))
        self.image_sel_coords_label = QLabel(guiDlg)
        self.image_sel_coords_label.setObjectName(u"image_sel_coords_label")
        self.image_sel_coords_label.setGeometry(QRect(210, 490, 221, 17))
        self.segment_button = QPushButton(guiDlg)
        self.segment_button.setObjectName(u"segment_button")
        self.segment_button.setGeometry(QRect(490, 480, 131, 31))
        self.save_new_folder_button = QPushButton(guiDlg)
        self.save_new_folder_button.setObjectName(u"save_new_folder_button")
        self.save_new_folder_button.setGeometry(QRect(650, 490, 200, 31))
        self.current_folder_label = QLabel(guiDlg)
        self.current_folder_label.setObjectName(u"current_folder_label")
        self.current_folder_label.setGeometry(QRect(860, 490, 430, 31))
        self.bump_present_button = QPushButton(guiDlg)
        self.bump_present_button.setObjectName(u"bump_present_button")
        self.bump_present_button.setGeometry(QRect(10, 525, 190, 30))
        self.bump_absent_button = QPushButton(guiDlg)
        self.bump_absent_button.setObjectName(u"bump_absent_button")
        self.bump_absent_button.setGeometry(QRect(210, 525, 190, 30))
        self.debug_log = QPlainTextEdit(guiDlg)
        self.debug_log.setObjectName(u"debug_log")
        self.debug_log.setGeometry(QRect(10, 565, 1280, 145))
        self.debug_log.setReadOnly(True)

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"vision_sam", None))
        self.image_label.setText(QCoreApplication.translate("guiDlg", u"Esperando imagen de Webots...", None))
        self.segmented_image_label.setText(QCoreApplication.translate("guiDlg", u"No hay imagen segmentada a\u00fan...", None))
        self.image_coords_label.setText(QCoreApplication.translate("guiDlg", u"Hover coords", None))
        self.image_sel_coords_label.setText(QCoreApplication.translate("guiDlg", u"Selected coords", None))
        self.segment_button.setText(QCoreApplication.translate("guiDlg", u"segment", None))
        self.save_new_folder_button.setText(QCoreApplication.translate("guiDlg", u"save on new folder", None))
        self.current_folder_label.setText(QCoreApplication.translate("guiDlg", u"Saving to: segmented_objects", None))
        self.bump_present_button.setText(QCoreApplication.translate("guiDlg", u"Con bache", None))
        self.bump_absent_button.setText(QCoreApplication.translate("guiDlg", u"Sin bache", None))
    # retranslateUi

