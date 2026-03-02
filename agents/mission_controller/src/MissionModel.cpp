#include "MissionModel.h"

MissionModel::MissionModel(QObject *parent) : QAbstractListModel(parent) {}

int MissionModel::rowCount(const QModelIndex &) const {
    return static_cast<int>(missions.size());
}

QVariant MissionModel::data(const QModelIndex &index, int role) const {
    if (!index.isValid() || index.row() >= rowCount()) return {};

    const Mission &m = missions[index.row()];

    if (role == Qt::DisplayRole)
        return m.name;

    if (role == Qt::UserRole)     return m.name;
    if (role == Qt::UserRole + 1) return m.type;
    if (role == Qt::UserRole + 2) return static_cast<int>(missions[index.row()].status);
    if (role == Qt::UserRole + 3) return m.elapsedTime;

    return {};
}

void MissionModel::addMission(const Mission &mission) {
    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    missions.push_back(mission);
    endInsertRows();
}

Mission MissionModel::getMission(int row) const {
    return missions[row];
}

void MissionModel::setMissionStatus(int row, MissionStatus status) {
    missions[row].status = status;
    emit dataChanged(index(row), index(row));
}

void MissionModel::setMissionElapsedTime(int row, int elapsed) {
    missions[row].elapsedTime = elapsed;
    emit dataChanged(index(row), index(row));
}