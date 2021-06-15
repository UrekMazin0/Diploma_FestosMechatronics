#ifndef QSERIALPROTOKOL_H
#define QSERIALPROTOKOL_H

#include <QObject>
#include <QIODevice>
#include <QTimer>
#include <QDateTime>
#include <QQueue>
#include <inttypes.h>

#include <QDebug>

#include "infoClasses.h"

struct PacketInfo {
	quint8 address;
	quint8 command;
	QByteArray data;
};

class QSerialProtocol : public QObject
{
	Q_OBJECT
public:
	explicit QSerialProtocol(QIODevice *device,QObject *parent = nullptr);
	~QSerialProtocol();

private:
	int debugValue = 0;
	QIODevice *serial; // указатель работающий с QSerialPort

	QVector<QSerialDeviceInfo> devices;
	int searchDevices(quint8 address);

	QVector<PacketInfo> queuePackets; // Вектор команд
	uint16_t commandCounter = 0; // номер отправляемой команды

	QByteArray data;

	void handlerPacket(QByteArray packet); // обработчик пакетов
	void sendCommand(quint8 address,quint8 command,QByteArray &data); //команду в вектор
	void sendNextCommand(); // послать следующую команду из вектора
	bool acceptPacket(quint8 command); //
	bool setBusAddresRange(QPair<quint8, quint8> r);

	quint8 currentDeviceAddress;
	void sendNextRequest();
	void sendPacket01(quint8 address); //Отправка запроса устройству
	void packet01(QByteArray inputData); //Получение ответа от устройства
	void errorPacket01(quint8 address); //Ошибка опроса устройства

	quint8 getControlSumm(QByteArray *input);

	QPair<quint8,quint8> range;
	bool flagSendTime = false;
	bool flagPollInitializeMachine = false;
	bool flagWait = false;

	QTimer *timerPoll;
	QTimer *timerQueue;

	int intervalPoll = 20;

	int timeWaitAnswer = 500;
	int countS = 1; //Количество попыток отправки пакета
	int currentCount = 1; //Остаток попыток отправки

	quint16 DeSerializationQuint16(QByteArray input);

signals:

	void signalSerialRawData(QByteArray data);
	void signalSendSerialRawData(QByteArray data);

private slots:

	void slotReadyRead();
	void slotClosePort();
	void slotTimerPoll();
};
#endif // QSERIALPROTOKOL_H
