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
from PySide6.QtWidgets import (QApplication, QLabel, QPushButton, QSizePolicy,
    QWidget)

class Ui_guiDlg(object):
    def setupUi(self, guiDlg):
        if not guiDlg.objectName():
            guiDlg.setObjectName(u"guiDlg")
        guiDlg.resize(641, 517)
        self.image_label = QLabel(guiDlg)
        self.image_label.setObjectName(u"image_label")
        self.image_label.setGeometry(QRect(0, 0, 640, 480))
        self.image_label.setScaledContents(True)
        self.image_coords_label = QLabel(guiDlg)
        self.image_coords_label.setObjectName(u"image_coords_label")
        self.image_coords_label.setGeometry(QRect(10, 490, 181, 17))
        self.image_sel_coords_label = QLabel(guiDlg)
        self.image_sel_coords_label.setObjectName(u"image_sel_coords_label")
        self.image_sel_coords_label.setGeometry(QRect(210, 490, 221, 17))
        self.segment_button = QPushButton(guiDlg)
        self.segment_button.setObjectName(u"segment_button")
        self.segment_button.setGeometry(QRect(490, 480, 131, 31))

        self.retranslateUi(guiDlg)

        QMetaObject.connectSlotsByName(guiDlg)
    # setupUi

    def retranslateUi(self, guiDlg):
        guiDlg.setWindowTitle(QCoreApplication.translate("guiDlg", u"vision_sam", None))
        self.image_label.setText(QCoreApplication.translate("guiDlg", u"Esperando imagen de Webots...", None))
        self.image_coords_label.setText(QCoreApplication.translate("guiDlg", u"Hover coords", None))
        self.image_sel_coords_label.setText(QCoreApplication.translate("guiDlg", u"Selected coords", None))
        self.segment_button.setText(QCoreApplication.translate("guiDlg", u"segment", None))
    # retranslateUi

