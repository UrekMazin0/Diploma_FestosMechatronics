#include "qserialprotocol.h"

QSerialProtocol::QSerialProtocol(QIODevice *device, QObject *parent) : QObject(parent)
{
	serial = device;

	connect(serial,SIGNAL(readyRead()),this,SLOT(slotReadyRead()));
	connect(serial,SIGNAL(aboutToClose()),this,SLOT(slotClosePort()));

	range.first = 0x01;
	range.second = 0x01;

	timerPoll = new QTimer(this);
	connect(timerPoll, &QTimer::timeout, this, &QSerialProtocol::slotTimerPoll);
	timerPoll->start(intervalPoll * 100);

	timerQueue = new QTimer(this);
	qDebug() << "protocol init";
}

QSerialProtocol::~QSerialProtocol()
{

}

void QSerialProtocol::sendPacket01(quint8 address)
{
	QByteArray buf;
	buf.push_back(2); // 1 byte = trash

	sendCommand(address,0x01,buf);
	qDebug() << "sendPacket01";
}

void QSerialProtocol::sendCommand(quint8 address, quint8 command, QByteArray &data)
{
	QByteArray buf;

	PacketInfo A;
	A.address = address;
	A.command = command;
	A.data = data;

	queuePackets.push_back(A);

	sendNextCommand();
}

void QSerialProtocol::sendNextCommand()
{
	if ((!flagWait) && (queuePackets.length()>0) && (serial->isOpen()))
	{
		QByteArray buf;
		PacketInfo A = queuePackets.at(commandCounter);

		quint16 length;
		quint8 *ptr = (quint8*)(&length);
		length = A.data.length() + 4;


		buf.push_back(A.address);
		buf.push_back(*ptr);
		buf.push_back(*(ptr+1));
		buf.push_back(A.command);
		buf.push_back(A.data);
		buf.push_back(getControlSumm(&buf));

		serial->write(buf);

		commandCounter++;
	}
}

void QSerialProtocol::slotReadyRead()
{
	qDebug() << "slotReadyRead " << debugValue;
	debugValue++;
	QByteArray Buf = serial->readAll();
	data.push_back(Buf);

	while (data.length() > 0)
	{
		if (data[0]==0xFF)
		{
			if (data.length()>3)
			{
				quint16 length = DeSerializationQuint16(data.mid(1,2));//data[2]<<8 + data[1];
				if (data.length() == length+1)
				{
					QByteArray forCRC;
					forCRC = data.mid(0,length);

					if (data[length+1] == getControlSumm(&forCRC))
					{
						qDebug() << "govno";
						emit signalSerialRawData(data.mid(0,length));
						handlerPacket(data.mid(3,1));
						data.remove(0,length); //Удаляем пакет из буфера
						//break;
					} else
					{
						data.remove(0,1); //Удаляем заголовок неверного пакета
						break;
					}
				} else break;
			} else break;
		} else data.remove(0,1);
	}
}

void QSerialProtocol::handlerPacket (QByteArray packet)
{
	bool ret = acceptPacket(packet[0]);

	switch (packet[0])
	{
		case 0x01: packet01(packet.mid(1,packet.length()-1));
			break;
	}

//	if (ret)
//		sendNextCommand();
}

void QSerialProtocol::slotClosePort()
{
//	if (timerQueue->isActive())
//		timerQueue->stop();

//	queuePackets.clear();
}

quint8 QSerialProtocol::getControlSumm(QByteArray *input)
{
	quint8 output =0;
	for (int i=0;i<input->length();i++)
		output = output + input->at(i);

	return output;
}

void QSerialProtocol::slotTimerPoll()
{
	if ((!flagPollInitializeMachine) && (serial->isOpen()))
	{
		flagPollInitializeMachine = true;
		currentDeviceAddress = range.first;
		sendNextRequest();
	}
}

void QSerialProtocol::sendNextRequest()
{
	if (currentDeviceAddress <= range.second)
	{
		sendPacket01(currentDeviceAddress);
	}
	else
	{
		qDebug() << "jopa";
		flagPollInitializeMachine = false;
	}
}

bool QSerialProtocol::setBusAddresRange(QPair<quint8, quint8> r)
{
	if(r.first <= r.second)
	{
		range = r;
		return true;
	}
	else
		return false;
}

bool QSerialProtocol::acceptPacket(quint8 command)
{
	bool Output = false;

	if (queuePackets.length() > 0)
	if (command == queuePackets.at(0).command) {
		if (timerQueue->isSingleShot())
			timerQueue->stop();
		flagWait = false;
		queuePackets.remove(0);
		Output = true;
	}

	return Output;
}

void QSerialProtocol::packet01(QByteArray inputData)
{
	if(inputData[0] == 0x01)
	{
		qDebug() << "Festo MicroController.";
		qDebug() << "Version: ATMega328P.";
	}
}

int QSerialProtocol::searchDevices(quint8 address)
{
	int Output = -1;

	for (int i=0;i<devices.length();i++)
		if (devices.at(i).address == address) {
			Output = i;
			break;
		}

	return Output;
}

quint16 QSerialProtocol::DeSerializationQuint16(QByteArray input)
{
	quint16 Output;
	quint8 *a = (quint8*)&Output;
	for (int i=0;i<input.length();i++)
	{
		*(a+i)=input[i];
	}
	return Output;
}
