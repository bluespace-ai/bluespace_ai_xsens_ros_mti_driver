
//  Copyright (c) 2003-2019 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "nmea_protocolhandler.h"
#include "nmea_parser.h"
#include <xstypes/xsdatapacket.h>
#include "xsprotocoltype.h"
#include "xscontrollerconfig.h"

namespace nmea
{

#define XSENS_MATH_DEG2RAD	0.017453292519943295769236907684886 // pi/180.0

/*! \class ProtocolHandler
	\brief The protocol handler for a different NMEA data types
*/

/*! \brief Default constructor
*/
ProtocolHandler::ProtocolHandler()
	: m_parser(xsens::NonIntrusiveSharedPointer<IParser>(new Parser()))
{
	m_parser->attachObserver(*this);
}

/*! \brief Deafult destructor
*/
ProtocolHandler::~ProtocolHandler() throw()
{
	try {
		m_parser->detachObserver(*this);
	}
	catch(...)
	{
	}
}

/*! \brief Initializes objects of this class
*/
void ProtocolHandler::onInitializeParse(char const * begin, char const * end)
{
	(void)begin; (void)end;
	m_dataPacket = XsDataPacket(); // Clear the current packet otherwise messages will be merged
}

/*! \brief Handles HCHDM
*/
void ProtocolHandler::onHCHDM(DoubleValue heading)
{
	if (heading.valid())
	{
		XsEuler euler(0.0, 0.0, calcEulerAngle(heading));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles HCHDG
*/
void ProtocolHandler::onHCHDG(DoubleValue heading,
							  DoubleValue deviation, BoolValue positiveDeviation,
							  DoubleValue variation, BoolValue positiveVariation)
{
	(void)deviation;
	(void)positiveDeviation;
	(void)variation;
	(void)positiveVariation;

	if (heading.valid())
	{
		XsEuler euler(0.0, 0.0, calcEulerAngle(heading));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles PHTRO
*/
void ProtocolHandler::onPHTRO(DoubleValue pitch, BoolValue bowUp,
							  DoubleValue roll, BoolValue portUp)
{
	(void)bowUp;
	(void)portUp;

	if (pitch.valid() && roll.valid())
	{
		XsEuler euler(calcEulerAngle(roll),calcEulerAngle(pitch), 0.0);
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles HCMTW
*/
void ProtocolHandler::onHCMTW(DoubleValue temperature)
{
	if (temperature.valid())
	{
		m_dataPacket.setTemperature(temperature);
	}
}

/*! \brief Handles PRDID
*/
void ProtocolHandler::onPRDID(DoubleValue pitch, DoubleValue roll, DoubleValue heading)
{
	if (pitch.valid() && roll.valid() & heading.valid())
	{
		XsEuler euler(calcEulerAngle(roll),calcEulerAngle(pitch), calcEulerAngle(heading));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles PSONCMS
*/
void ProtocolHandler::onPSONCMS(
	DoubleValue quat1, DoubleValue quat2, DoubleValue quat3, DoubleValue quat4,
	DoubleValue acc_x, DoubleValue acc_y, DoubleValue acc_z,
	DoubleValue omega_x, DoubleValue omega_y, DoubleValue omega_z,
	DoubleValue mag_x, DoubleValue mag_y, DoubleValue mag_z,
	DoubleValue temperature)
{
	XsVector3 vec3;
	if (quat1.valid() && quat2.valid() && quat3.valid() && quat4.valid())
	{
		XsQuaternion quaternion(quat1, quat2, quat3, quat4);
		m_dataPacket.setOrientationQuaternion(quaternion, XDI_CoordSysNwu);
	}

	vec3[0] = acc_x.valid() ? acc_x.value() : 0.0;
	vec3[1] = acc_y.valid() ? acc_y.value() : 0.0;
	vec3[2] = acc_z.valid() ? acc_z.value() : 0.0;
	m_dataPacket.setCalibratedAcceleration(vec3);

	vec3[0] = omega_x.valid() ? omega_x.value() : 0.0;
	vec3[1] = omega_y.valid() ? omega_y.value() : 0.0;
	vec3[2] = omega_z.valid() ? omega_z.value() : 0.0;
	m_dataPacket.setCalibratedGyroscopeData(vec3);

	vec3[0] = mag_x.valid() ? mag_x.value() : 0.0;
	vec3[1] = mag_y.valid() ? mag_y.value() : 0.0;
	vec3[2] = mag_z.valid() ? mag_z.value() : 0.0;
	m_dataPacket.setCalibratedMagneticField(vec3);

	m_dataPacket.setTemperature(temperature);
}

/*! \brief Handles TSS2
*/
void ProtocolHandler::onTSS2(DoubleValue heading, DoubleValue heave, DoubleValue roll, DoubleValue pitch)
{
	(void)heave;
	if (pitch.valid() && roll.valid() & heading.valid())
	{
		XsEuler euler(roll/100,pitch/100, calcEulerAngle(heading/100));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles EM1000
*/
void ProtocolHandler::onEM1000(DoubleValue roll, DoubleValue pitch, DoubleValue heave, DoubleValue heading)
{
	(void)heave;
	if (pitch.valid() && roll.valid() & heading.valid())
	{
		XsEuler euler(roll/100,-pitch/100, calcEulerAngle(heading/100));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles HEHDT
*/
void ProtocolHandler::onHEHDT(DoubleValue heading)
{
	if (heading.valid())
	{
		XsEuler euler(0.0, 0.0, calcEulerAngle(heading));
		m_dataPacket.setOrientationEuler(euler, XDI_CoordSysNwu);
	}
}

/*! \brief Handles HEROT
*/
void ProtocolHandler::onHEROT(DoubleValue rateOfTurn)
{
	if (rateOfTurn.valid())
	{
		XsVector gyro(3);
		gyro.zero();
		gyro[2] = -rateOfTurn / 60 * XSENS_MATH_DEG2RAD;
		m_dataPacket.setCalibratedGyroscopeData(gyro);
	}
}

/*! \brief Searches for a message in an array
	\param rcv The refernce where the found message is stored
	\param raw The byte array to search in
	\returns The found message location
*/
MessageLocation ProtocolHandler::findMessage(XsMessage& rcv, const XsByteArray& raw) const
{
	MessageLocation location;

	char const * begin = (char const *) raw.data();
	char const * end = begin + raw.size();

	while (begin < end)
	{
		IParser::ParseResult parseResult = m_parser->parseMessage(begin, end);
		if (parseResult.first)
		{
			assert((int)parseResult.second <= MAXIMUM_MESSAGE_SIZE);
			if (parseResult.second > 0)
			{
				location.m_startPos = static_cast<int>(begin - (char const *) raw.data());
				location.m_size = static_cast<int>(parseResult.second);
				JLTRACEG("NMEA: StartPos: " << location.m_startPos << " Size: " << location.m_size);
				rcv = m_dataPacket.toMessage();
				break;
			}
		}
		++begin;
	}

	return location;
}

/*! \returns The minimum message size
*/
int ProtocolHandler::minimumMessageSize() const
{
	return MINIMUM_MESSAGE_SIZE;
}

/*! \returns The maximum message size
*/
int ProtocolHandler::maximumMessageSize() const
{
	return MAXIMUM_MESSAGE_SIZE;
}

/*! \brief Calculates the euler angle fro heading value
	\param heading The heading value
	\returns The calculated euler angle
*/
inline XsReal ProtocolHandler::calcEulerAngle(XsReal heading) const
{
	if (heading > 180)
	{
		heading = heading - 360;
	}
	else if (heading < -180)
	{
		heading = 360 + heading;
	}

	return heading;
}

/*! \returns The type of the protocol */
int ProtocolHandler::type() const
{
	return XPT_Nmea;
}

}
