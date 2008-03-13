/********************************************************************************
** Form generated from reading ui file 'VRenderInterface.Qt4.ui'
**
** Created: Fri Jun 22 11:38:57 2007
**      by: Qt User Interface Compiler version 4.3.0
**
** WARNING! All changes made in this file will be lost when recompiling ui file!
********************************************************************************/

#ifndef UI_VRENDERINTERFACE_H
#define UI_VRENDERINTERFACE_H

#include <Qt3Support/Q3MimeSourceFactory>
#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QComboBox>
#include <QtGui/QDialog>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>

class Ui_VRenderInterface
{
public:
    QVBoxLayout *vboxLayout;
    QCheckBox *includeHidden;
    QCheckBox *cullBackFaces;
    QCheckBox *blackAndWhite;
    QCheckBox *colorBackground;
    QCheckBox *tightenBBox;
    QHBoxLayout *hboxLayout;
    QLabel *sortLabel;
    QComboBox *sortMethod;
    QSpacerItem *spacerItem;
    QHBoxLayout *hboxLayout1;
    QPushButton *SaveButton;
    QPushButton *CancelButton;

    void setupUi(QDialog *VRenderInterface)
    {
    if (VRenderInterface->objectName().isEmpty())
        VRenderInterface->setObjectName(QString::fromUtf8("VRenderInterface"));
    QSize size(298, 224);
    size = size.expandedTo(VRenderInterface->minimumSizeHint());
    VRenderInterface->resize(size);
    vboxLayout = new QVBoxLayout(VRenderInterface);
    vboxLayout->setSpacing(6);
    vboxLayout->setMargin(5);
    vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
    includeHidden = new QCheckBox(VRenderInterface);
    includeHidden->setObjectName(QString::fromUtf8("includeHidden"));

    vboxLayout->addWidget(includeHidden);

    cullBackFaces = new QCheckBox(VRenderInterface);
    cullBackFaces->setObjectName(QString::fromUtf8("cullBackFaces"));

    vboxLayout->addWidget(cullBackFaces);

    blackAndWhite = new QCheckBox(VRenderInterface);
    blackAndWhite->setObjectName(QString::fromUtf8("blackAndWhite"));

    vboxLayout->addWidget(blackAndWhite);

    colorBackground = new QCheckBox(VRenderInterface);
    colorBackground->setObjectName(QString::fromUtf8("colorBackground"));

    vboxLayout->addWidget(colorBackground);

    tightenBBox = new QCheckBox(VRenderInterface);
    tightenBBox->setObjectName(QString::fromUtf8("tightenBBox"));

    vboxLayout->addWidget(tightenBBox);

    hboxLayout = new QHBoxLayout();
    hboxLayout->setSpacing(6);
    hboxLayout->setMargin(11);
    hboxLayout->setObjectName(QString::fromUtf8("hboxLayout"));
    sortLabel = new QLabel(VRenderInterface);
    sortLabel->setObjectName(QString::fromUtf8("sortLabel"));

    hboxLayout->addWidget(sortLabel);

    sortMethod = new QComboBox(VRenderInterface);
    sortMethod->setObjectName(QString::fromUtf8("sortMethod"));

    hboxLayout->addWidget(sortMethod);


    vboxLayout->addLayout(hboxLayout);

    spacerItem = new QSpacerItem(31, 41, QSizePolicy::Minimum, QSizePolicy::Expanding);

    vboxLayout->addItem(spacerItem);

    hboxLayout1 = new QHBoxLayout();
    hboxLayout1->setSpacing(6);
    hboxLayout1->setMargin(0);
    hboxLayout1->setObjectName(QString::fromUtf8("hboxLayout1"));
    SaveButton = new QPushButton(VRenderInterface);
    SaveButton->setObjectName(QString::fromUtf8("SaveButton"));

    hboxLayout1->addWidget(SaveButton);

    CancelButton = new QPushButton(VRenderInterface);
    CancelButton->setObjectName(QString::fromUtf8("CancelButton"));

    hboxLayout1->addWidget(CancelButton);


    vboxLayout->addLayout(hboxLayout1);

    QWidget::setTabOrder(SaveButton, CancelButton);
    QWidget::setTabOrder(CancelButton, includeHidden);
    QWidget::setTabOrder(includeHidden, cullBackFaces);
    QWidget::setTabOrder(cullBackFaces, blackAndWhite);
    QWidget::setTabOrder(blackAndWhite, colorBackground);
    QWidget::setTabOrder(colorBackground, tightenBBox);
    QWidget::setTabOrder(tightenBBox, sortMethod);

    retranslateUi(VRenderInterface);
    QObject::connect(SaveButton, SIGNAL(released()), VRenderInterface, SLOT(accept()));
    QObject::connect(CancelButton, SIGNAL(released()), VRenderInterface, SLOT(reject()));

    sortMethod->setCurrentIndex(0);


    QMetaObject::connectSlotsByName(VRenderInterface);
    } // setupUi

    void retranslateUi(QDialog *VRenderInterface)
    {
    VRenderInterface->setWindowTitle(QApplication::translate("VRenderInterface", "Vectorial rendering options", 0, QApplication::UnicodeUTF8));
    includeHidden->setToolTip(QApplication::translate("VRenderInterface", "Hidden poligons are also included in the output (usually twice bigger)", 0, QApplication::UnicodeUTF8));
    includeHidden->setText(QApplication::translate("VRenderInterface", "Include hidden parts", 0, QApplication::UnicodeUTF8));
    cullBackFaces->setToolTip(QApplication::translate("VRenderInterface", "Back faces (non clockwise point ordering) are removed from the output", 0, QApplication::UnicodeUTF8));
    cullBackFaces->setText(QApplication::translate("VRenderInterface", "Cull back faces", 0, QApplication::UnicodeUTF8));
    blackAndWhite->setToolTip(QApplication::translate("VRenderInterface", "Black and white rendering", 0, QApplication::UnicodeUTF8));
    blackAndWhite->setText(QApplication::translate("VRenderInterface", "Black and white", 0, QApplication::UnicodeUTF8));
    colorBackground->setToolTip(QApplication::translate("VRenderInterface", "Use current background color instead of white", 0, QApplication::UnicodeUTF8));
    colorBackground->setText(QApplication::translate("VRenderInterface", "Color background", 0, QApplication::UnicodeUTF8));
    tightenBBox->setToolTip(QApplication::translate("VRenderInterface", "Fit output bounding box to current display", 0, QApplication::UnicodeUTF8));
    tightenBBox->setText(QApplication::translate("VRenderInterface", "Tighten bounding box", 0, QApplication::UnicodeUTF8));
    sortLabel->setToolTip(QApplication::translate("VRenderInterface", "Polygon depth sorting method", 0, QApplication::UnicodeUTF8));
    sortLabel->setText(QApplication::translate("VRenderInterface", "Sort method :", 0, QApplication::UnicodeUTF8));
    sortMethod->clear();
    sortMethod->insertItems(0, QStringList()
     << QApplication::translate("VRenderInterface", "No sorting", 0, QApplication::UnicodeUTF8)
     << QApplication::translate("VRenderInterface", "BSP ", 0, QApplication::UnicodeUTF8)
     << QApplication::translate("VRenderInterface", "Topological", 0, QApplication::UnicodeUTF8)
     << QApplication::translate("VRenderInterface", "Advanced topological", 0, QApplication::UnicodeUTF8)
    );
    sortMethod->setToolTip(QApplication::translate("VRenderInterface", "Polygon depth sorting method", 0, QApplication::UnicodeUTF8));
    SaveButton->setText(QApplication::translate("VRenderInterface", "Save", 0, QApplication::UnicodeUTF8));
    CancelButton->setText(QApplication::translate("VRenderInterface", "Cancel", 0, QApplication::UnicodeUTF8));
    Q_UNUSED(VRenderInterface);
    } // retranslateUi

};

namespace Ui {
    class VRenderInterface: public Ui_VRenderInterface {};
} // namespace Ui

#endif // UI_VRENDERINTERFACE_H
