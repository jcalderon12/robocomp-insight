#include "MissionDelegate.h"
#include <QPainter>
#include <QApplication>
#include <QMouseEvent>
#include <iostream>

QPair<QColor, QString> MissionDelegate::getPriorityColorAndName(int priority) const {
    switch (priority) {
        case 5: return {QColor(200, 0, 0), "Critical"};
        case 4: return {QColor(255, 100, 0), "High"};
        case 3: return {QColor(255, 200, 0), "Normal"};
        case 2: return {QColor(100, 150, 255), "Low"};
        case 1: return {QColor(150, 150, 150), "Very Low"};
        default: return {QColor(128, 128, 128), "Unknown"};
    }
}

MissionDelegate::MissionDelegate(QObject *parent) : QStyledItemDelegate(parent) {}

QRect MissionDelegate::getToggleButtonRect(const QStyleOptionViewItem &option) const {
    return QRect(option.rect.right() - 2 * (BTN_WIDTH + MARGIN),
                 option.rect.top() + (ITEM_HEIGHT - BTN_HEIGHT) / 2,
                 BTN_WIDTH, BTN_HEIGHT);
}

QRect MissionDelegate::getCompletedButtonRect(const QStyleOptionViewItem &option) const {
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
    int priority = index.data(Qt::UserRole + 4).toInt();
    auto [priorityColor, priorityName] = getPriorityColorAndName(priority);

    // Draw priority color bar on the left
    painter->fillRect(option.rect.left(), option.rect.top(), PRIORITY_BAR_WIDTH, option.rect.height(), priorityColor);

    QFont nameFont = painter->font();
    nameFont.setBold(true);
    nameFont.setPointSize(12);
    nameFont.setItalic(false);
    painter->setFont(nameFont);
    painter->setPen(QColor(50, 50, 50));
    painter->drawText(option.rect.adjusted(MARGIN + PRIORITY_BAR_WIDTH, 8, -2*(BTN_WIDTH+MARGIN)-MARGIN, -75),
                      Qt::AlignLeft | Qt::AlignTop, name);

    QFont typeFont = painter->font();
    typeFont.setBold(false);
    typeFont.setPointSize(9);
    typeFont.setItalic(true);
    painter->setFont(typeFont);
    painter->setPen(QColor(100, 100, 100));
    painter->drawText(option.rect.adjusted(MARGIN + PRIORITY_BAR_WIDTH, 26, -2*(BTN_WIDTH+MARGIN)-MARGIN, -60),
                      Qt::AlignLeft | Qt::AlignTop, "Type: " + type);

    // Draw priority text
    QFont priorityFont = painter->font();
    priorityFont.setBold(false);
    priorityFont.setPointSize(10);
    painter->setFont(priorityFont);
    painter->setPen(priorityColor);
    painter->drawText(option.rect.adjusted(MARGIN + PRIORITY_BAR_WIDTH, 46, -2*(BTN_WIDTH+MARGIN)-MARGIN, -40),
                      Qt::AlignLeft | Qt::AlignTop, "Priority: " + priorityName);

    QString statusText;
    QColor  statusColor;
    bool isCompleted = false;
    float elapsedTime = index.data(Qt::UserRole + 3).toFloat();
    switch (status)
    {
        case MissionStatus::IDLE:
            statusText  = "State: Idle";
            statusColor = QColor(130, 130, 130);
            break;
        case MissionStatus::RUNNING:
            statusText  = QString("State: Running (%1s)").arg(elapsedTime, 0, 'f', 2);
            statusColor = QColor(50, 160, 50);
            break;
        case MissionStatus::STOPPED:
            statusText  = QString("State: Stopped (%1s)").arg(elapsedTime, 0, 'f', 2);
            statusColor = QColor(200, 60, 60);
            break;
        case MissionStatus::COMPLETED:
            statusText  = QString("State: Completed (%1s)").arg(elapsedTime, 0, 'f', 2);
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
    
    if (isCompleted) {
        painter->fillRect(toggleBtn.rect, QColor(200, 200, 200, 150));
        painter->setPen(QColor(150, 150, 150));
        painter->drawText(toggleBtn.rect, Qt::AlignCenter, toggleBtn.text);
    }

    QStyleOptionButton defaultBtn;
    defaultBtn.rect  = getCompletedButtonRect(option);
    defaultBtn.text  = "Complete";
    if (isCompleted || status == MissionStatus::STOPPED) {
        defaultBtn.state = QStyle::State_None;
    } else {
        defaultBtn.state = QStyle::State_Enabled;
    }
    QApplication::style()->drawControl(QStyle::CE_PushButton, &defaultBtn, painter);
    
    // Visual feedback for disabled Complete button
    if (isCompleted || status == MissionStatus::STOPPED) {
        painter->fillRect(defaultBtn.rect, QColor(200, 200, 200, 150));
        painter->setPen(QColor(150, 150, 150));
        painter->setFont(painter->font());
        painter->drawText(defaultBtn.rect, Qt::AlignCenter, defaultBtn.text);
    }

    painter->restore();
}

QSize MissionDelegate::sizeHint(const QStyleOptionViewItem &, const QModelIndex &) const {
    return QSize(0, ITEM_HEIGHT);
}

bool MissionDelegate::editorEvent(QEvent *event, QAbstractItemModel *model,
                                   const QStyleOptionViewItem &option,
                                   const QModelIndex &index) {
    if (event->type() == QEvent::MouseButtonRelease) {
        auto *mouseEvent = static_cast<QMouseEvent *>(event);
        QPoint pos = mouseEvent->pos();
        
        // Get mission status to check if operations are allowed
        auto status = static_cast<MissionStatus>(index.data(Qt::UserRole + 2).toInt());
        bool isCompleted = (status == MissionStatus::COMPLETED);

        if (getToggleButtonRect(option).contains(pos)) {
            emit missionToggleClicked(index.row());
            return true;
        }
        if (getCompletedButtonRect(option).contains(pos)) {
            // Only emit signal if mission is not already completed
            if (!isCompleted && status != MissionStatus::STOPPED) {
                emit missionCompletedClicked(index.row());
            }
            return true;
        }
    }
    return false;
}