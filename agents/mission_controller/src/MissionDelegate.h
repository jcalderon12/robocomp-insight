#pragma once
#include <QStyledItemDelegate>
#include "MissionModel.h"

class MissionDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    explicit MissionDelegate(QObject *parent = nullptr);

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const override;
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const override;
    bool editorEvent(QEvent *event, QAbstractItemModel *model,
                     const QStyleOptionViewItem &option,
                     const QModelIndex &index) override;

signals:
    void missionToggleClicked(int row);
    void missionCompletedClicked(int row);

private:
    QRect getToggleButtonRect(const QStyleOptionViewItem &option) const;
    QRect getCompletedButtonRect(const QStyleOptionViewItem &option) const;

    static constexpr int ITEM_HEIGHT = 100;
    static constexpr int BTN_WIDTH   = 80;
    static constexpr int BTN_HEIGHT  = 35;
    static constexpr int MARGIN      = 10;
};