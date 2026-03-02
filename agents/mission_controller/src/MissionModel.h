#pragma once
#include <QAbstractListModel>
#include <QString>
#include <vector>

enum class MissionStatus { IDLE, RUNNING, STOPPED, COMPLETED };

struct Mission {
    QString name;
    QString type;  // Tipo de misión elegido
    int elapsedTime = 0;  // Tiempo transcurrido en segundos
    MissionStatus status = MissionStatus::IDLE;
};

class MissionModel : public QAbstractListModel {
    Q_OBJECT
public:
    explicit MissionModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role = Qt::DisplayRole) const override;

    void addMission(const Mission &mission);
    Mission getMission(int row) const;

    void setMissionStatus(int row, MissionStatus status);
    void setMissionElapsedTime(int row, int elapsed);

private:
    std::vector<Mission> missions;
};