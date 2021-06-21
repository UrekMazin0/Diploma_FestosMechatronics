#include <iostream>
#include <QByteArray>

using namespace std;

quint8 getControlSumm(QByteArray *input);
quint8 ControlSumm(QByteArray *Mass, quint8 length);

int main()
{
	QByteArray buf;
	buf.push_back(0xFF);
	buf.push_back(2);
	buf.push_back(0x01);
	buf.push_back(getControlSumm(&buf));

	uint8_t jopa = (uint8_t)buf[3];
	cout << "CRC " << jopa ;


	return 0;
}

quint8 getControlSumm(QByteArray *input)
{
	quint8 output =0;
	for (int i=0;i<input->length();i++)
		output = output + input->at(i);
	return output;
}

quint8 ControlSumm(QByteArray *Mass, quint8 length)
{
	quint8 output;
	output=*(Mass);
	for (unsigned int i=1;i<length;i++) output = output + *(Mass+i);
	return output;
}
