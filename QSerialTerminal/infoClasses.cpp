#include "infoClasses.h"

QMeasurementInfo::QMeasurementInfo()
{

}

QParameterInfo::QParameterInfo()
{

}

QSerialDeviceInfo::QSerialDeviceInfo()
{
	address = 0xff;
}

bool QSerialDeviceInfo::isEmptyInfo()
{
	bool Output;

	if (manufacturer.isEmpty() && serialNumber.isEmpty())
		Output = true;
			else Output = false;

	return Output;
}

bool QSerialDeviceInfo::isEmptyMeasurements()
{
	bool Output;

	if (measurements.length() == 0)
		Output = true;
			else Output = false;

	return Output;
}
