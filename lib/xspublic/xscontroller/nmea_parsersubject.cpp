
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

#include "nmea_parsersubject.h"
#include "nmea_iparserobserver.h"

namespace nmea
{

/*! \brief Attaches an observer
	\param observer The parser observer to attach
*/
void ParserSubject::attachObserver(IParserObserver& observer)
{
	m_observers.insert(&observer);
}


/*! \brief Detaches an observer
\param observer The parser observer to detach
*/
void ParserSubject::detachObserver(IParserObserver& observer)
{
	m_observers.erase(&observer);
}

/*! \brief Notifies about parser initialization
	\param begin The beginning of a message
	\param end The end of a message
*/
void ParserSubject::notifyInitializeParse(char const * begin, char const * end) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onInitializeParse(begin, end);
	}
}

/*! \brief Notifies HCHDM
	\param heading The heading value
*/
void ParserSubject::notifyHCHDM(DoubleValue heading) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onHCHDM(heading);
	}
}

/*! \brief Notifies HCHDG
	\param heading The heading value
	\param deviation The deviation value
	\param positiveDeviation The positive deviation value
	\param variation The variation value
	\param positiveVariation The positive variation value
*/
void ParserSubject::notifyHCHDG(DoubleValue heading,
								DoubleValue deviation, BoolValue positiveDeviation,
								DoubleValue variation, BoolValue positiveVariation) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onHCHDG(heading, deviation, positiveDeviation, variation, positiveVariation);
	}
}

/*! \brief Notifies PHTRO
	\param pitch The pitch value
	\param bowUp The bow up value
	\param roll The roll value
	\param portUp The port up value
*/
void ParserSubject::notifyPHTRO(DoubleValue pitch, BoolValue bowUp,
								DoubleValue roll, BoolValue portUp) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onPHTRO(pitch, bowUp, roll, portUp);
	}
}

/*! \brief Notifies HCMTW
	\param temperature The temperature value
*/
void ParserSubject::notifyHCMTW(DoubleValue temperature) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onHCMTW(temperature);
	}
}

/*! \brief Notifies PRDID
	\param pitch The pitch value
	\param roll The roll value
	\param heading The heading value
*/
void ParserSubject::notifyPRDID(DoubleValue pitch, DoubleValue roll, DoubleValue heading) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onPRDID(pitch, roll, heading);
	}
}

/*! \brief Notifies PSONCMS
	\param quat1 The first quaternion value
	\param quat2 The second quaternion value
	\param quat3 The third quaternion value
	\param quat4 The fourth quaternion value
	\param acc_x The X-axis accelerometer value
	\param acc_y The Y-axis accelerometer value
	\param acc_z The Z-axis accelerometer value
	\param omega_x The X-axis gyroscope value
	\param omega_y The Y-axis gyroscope value
	\param omega_z The Z-axis gyroscope value
	\param mag_x The X-axis magnetometer value
	\param mag_y The Y-axis magnetometer value
	\param mag_z The Z-axis magnetometer value
	\param temperature The temperature value
*/
void ParserSubject::notifyPSONCMS(
	DoubleValue quat1, DoubleValue quat2, DoubleValue quat3, DoubleValue quat4,
	DoubleValue acc_x, DoubleValue acc_y, DoubleValue acc_z,
	DoubleValue omega_x, DoubleValue omega_y, DoubleValue omega_z,
	DoubleValue mag_x, DoubleValue mag_y, DoubleValue mag_z,
	DoubleValue temperature) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onPSONCMS(
			quat1, quat2, quat3, quat4, acc_x, acc_y, acc_z,
			omega_x, omega_y, omega_z, mag_x, mag_y, mag_z, temperature
		);
	}
}

/*! \brief Notifies TSS2
	\param heading The heading value
	\param heave The heave value
	\param roll The roll value
	\param pitch The pitch value
*/
void ParserSubject::notifyTSS2(DoubleValue heading, DoubleValue heave, DoubleValue roll, DoubleValue pitch) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onTSS2(heading, heave, roll, pitch);
	}
}

/*! \brief Notifies EM1000
	\param roll The roll value
	\param pitch The pitch value
	\param heave The heave value
	\param heading The heading value
*/
void ParserSubject::notifyEM1000(DoubleValue roll, DoubleValue pitch, DoubleValue heave, DoubleValue heading) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onEM1000(roll, pitch, heave, heading);
	}
}

/*! \brief Notifies HEHDT
	\param heading The heading value
*/
void ParserSubject::notifyHEHDT(DoubleValue heading) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onHEHDT(heading);
	}
}

/*! \brief Notifies HEROT
	\param rateOfTurn The rate of turn value
*/
void ParserSubject::notifyHEROT(DoubleValue rateOfTurn) const
{
	Observers tmp(m_observers); // make copy to prevent modification of observers while notifying
	for (Observers::iterator i = tmp.begin(); i != tmp.end(); ++i)
	{
		(*i)->onHEROT(rateOfTurn);
	}
}
}
