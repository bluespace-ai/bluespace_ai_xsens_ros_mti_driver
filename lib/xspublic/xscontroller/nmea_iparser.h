
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

#ifndef NMEA_IPARSER_H
#define NMEA_IPARSER_H

#include "nmea_value.h"
#include <utility>

namespace nmea
{

class IParserObserver;


class IParser
{
public:
	typedef std::pair<bool, size_t> ParseResult;

	virtual ~IParser() throw() {}

	virtual ParseResult parseMessage(char const * begin, char const * end) const = 0;

	virtual void attachObserver(IParserObserver&) = 0;
	virtual void detachObserver(IParserObserver&) = 0;

protected:
	virtual void notifyHCHDM(DoubleValue heading) const = 0;
	virtual void notifyHCHDG(DoubleValue heading,
							 DoubleValue deviation, BoolValue positiveDeviation,
							 DoubleValue variation, BoolValue positiveVariation) const = 0;
	virtual void notifyPHTRO(DoubleValue pitch, BoolValue bowUp,
							 DoubleValue roll, BoolValue portUp) const = 0;
	virtual void notifyPRDID(DoubleValue pitch, DoubleValue roll, DoubleValue heading) const = 0;
	virtual void notifyPSONCMS(DoubleValue quat1, DoubleValue quat2, DoubleValue quat3, DoubleValue quat4,
							   DoubleValue acc_x, DoubleValue acc_y, DoubleValue acc_z,
							   DoubleValue omega_x, DoubleValue omaga_y, DoubleValue omaga_z,
							   DoubleValue mag_x, DoubleValue mag_y, DoubleValue mag_z,
							   DoubleValue temperature) const = 0;
	virtual void notifyHCMTW(DoubleValue temperature) const = 0;
	virtual void notifyTSS2(DoubleValue heading, DoubleValue heave, DoubleValue roll, DoubleValue pitch) const = 0;
	virtual void notifyEM1000(DoubleValue roll, DoubleValue pitch, DoubleValue heave, DoubleValue heading) const = 0;
	virtual void notifyHEHDT(DoubleValue heading) const = 0;
	virtual void notifyHEROT(DoubleValue rateOfTurn) const = 0;
};
}

#endif

