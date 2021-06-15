#ifndef INFOCLASSES_H
#define INFOCLASSES_H

#include <QObject>

class QMeasurementInfo
{
public:
	QMeasurementInfo();
	quint8 type;
	quint8 unit;
	quint16 idSensor;
};

class QParameterInfo
{
public:
	QParameterInfo();

	quint8 number;
	quint8 type;
};

class QSerialDeviceInfo
{
public:
	QSerialDeviceInfo();

	quint8 address;
	QString manufacturer;
	QString serialNumber;

	QVector<QMeasurementInfo> measurements;
	QVector<QParameterInfo> parameters;

	bool isEmptyInfo();
	bool isEmptyMeasurements();

};

#endif // INFOCLASSES_H
