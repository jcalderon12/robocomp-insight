#include "MissionDelegate.h"
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>

MissionDelegate::MissionDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QRect MissionDelegate::getToggleButtonRect(const QStyleOptionViewItem &option) const {
    return QRect(option.rect.right() - 2 * (BTN_WIDTH + MARGIN),
                 option.rect.top() + (ITEM_HEIGHT - BTN_HEIGHT) / 2,
                 BTN_WIDTH, BTN_HEIGHT);
}

QRect MissionDelegate::getDefaultButtonRect(const QStyleOptionViewItem &option) const {
    return QRect(option.rect.right() - (BTN_WIDTH + MARGIN),
                 option.rect.top() + (ITEM_HEIGHT - BTN_HEIGHT) / 2,
                 BTN_WIDTH, BTN_HEIGHT);
}

void MissionDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                             const QModelIndex &index) const {
    painter->save();

    // Aplicar color de fondo según selección
    if (option.state & QStyle::State_Selected)
    {
        painter->fillRect(option.rect, QColor(180, 200, 220));  // Azul claro para seleccionado
    }
    else if (index.row() % 2 == 0)
        painter->fillRect(option.rect, QColor(230, 230, 230));
    else
        painter->fillRect(option.rect, QColor(245, 245, 245));

    QString name = index.data(Qt::UserRole).toString();
    QString type = index.data(Qt::UserRole + 1).toString();
    auto status  = static_cast<MissionStatus>(index.data(Qt::UserRole + 2).toInt());

    QFont nameFont = painter->font();
    nameFont.setBold(true);
    nameFont.setPointSize(12);
    nameFont.setItalic(false);
    painter->setFont(nameFont);
    painter->setPen(QColor(50, 50, 50));
    painter->drawText(option.rect.adjusted(MARGIN, 10, -2*(BTN_WIDTH+MARGIN)-MARGIN, -ITEM_HEIGHT/2 + 5),
                      Qt::AlignLeft | Qt::AlignTop, name);

    QFont typeFont = painter->font();
    typeFont.setBold(false);
    typeFont.setPointSize(9);
    typeFont.setItalic(true);
    painter->setFont(typeFont);
    painter->setPen(QColor(100, 100, 100));
    painter->drawText(option.rect.adjusted(MARGIN, 32, -2*(BTN_WIDTH+MARGIN)-MARGIN, -ITEM_HEIGHT/2 + 5),
                      Qt::AlignLeft | Qt::AlignTop, "Type: " + type);

    QString statusText;
    QColor  statusColor;
    bool isCompleted = false;
    switch (status)
    {
        case MissionStatus::IDLE:
            statusText  = "State: Idle";
            statusColor = QColor(130, 130, 130);
            break;
        case MissionStatus::RUNNING:
            statusText  = "State: Running";
            statusColor = QColor(50, 160, 50);
            break;
        case MissionStatus::STOPPED:
            statusText  = "State: Stopped";
            statusColor = QColor(200, 60, 60);
            break;
        case MissionStatus::COMPLETED:
            statusText  = QString("State: Completed (%1s)").arg(index.data(Qt::UserRole + 3).toInt());
            statusColor = QColor(70, 130, 180);
            isCompleted = true;
            break;
    }

    QFont statusFont = painter->font();
    statusFont.setBold(false);
    statusFont.setPointSize(10);
    statusFont.setItalic(false);
    painter->setFont(statusFont);
    painter->setPen(statusColor);
    painter->drawText(option.rect.adjusted(MARGIN, ITEM_HEIGHT/2 + 10, -2*(BTN_WIDTH+MARGIN)-MARGIN, -8),
                      Qt::AlignLeft | Qt::AlignVCenter, statusText);

    QStyleOptionButton toggleBtn;
    toggleBtn.rect  = getToggleButtonRect(option);
    toggleBtn.text  = (status == MissionStatus::RUNNING) ? "Stop" : "Start";
    if (isCompleted) {
        toggleBtn.state = QStyle::State_None;
    } else {
        toggleBtn.state = QStyle::State_Enabled;
    }
    QApplication::style()->drawControl(QStyle::CE_PushButton, &toggleBtn, painter);
    
    // Efecto visual para botón deshabilitado
    if (isCompleted) {
        painter->fillRect(toggleBtn.rect, QColor(200, 200, 200, 150));
        painter->setPen(QColor(150, 150, 150));
        painter->drawText(toggleBtn.rect, Qt::AlignCenter, toggleBtn.text);
    }

    QStyleOptionButton defaultBtn;
    defaultBtn.rect  = getDefaultButtonRect(option);
    defaultBtn.text  = "Complete";
    if (isCompleted || status == MissionStatus::STOPPED) {
        defaultBtn.state = QStyle::State_None;
    } else {
        defaultBtn.state = QStyle::State_Enabled;
    }
    QApplication::style()->drawControl(QStyle::CE_PushButton, &defaultBtn, painter);

    painter->restore();
}

QSize MissionDelegate::sizeHint(const QStyleOptionViewItem &, const QModelIndex &) const {
    return QSize(0, ITEM_HEIGHT);
}

bool MissionDelegate::editorEvent(QEvent *event, QAbstractItemModel *,
                                   const QStyleOptionViewItem &option,
                                   const QModelIndex &index) {
    if (event->type() == QEvent::MouseButtonRelease) {
        auto *mouseEvent = static_cast<QMouseEvent *>(event);
        QPoint pos = mouseEvent->pos();

        if (getToggleButtonRect(option).contains(pos)) {
            emit missionToggleClicked(index.row());
            return true;
        }
        if (getDefaultButtonRect(option).contains(pos)) {
            emit missionDefaultClicked(index.row());
            return true;
        }
    }
    return false;
}