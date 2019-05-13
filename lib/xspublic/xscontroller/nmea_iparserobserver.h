
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

#ifndef NMEA_IPARSEROBSERVER_H
#define NMEA_IPARSEROBSERVER_H

#include "nmea_value.h"

namespace nmea
{


class IParserObserver
{
public:
	virtual ~IParserObserver() throw() = 0;

	virtual void onInitializeParse(char const * begin, char const * end) {(void)begin; (void)end;}

	virtual void onHCHDM(DoubleValue heading) {(void)heading;}
	virtual void onHCHDG(DoubleValue heading,
						 DoubleValue deviation, BoolValue positiveDeviation,
						 DoubleValue variation, BoolValue positiveVariation) {(void)heading;(void)deviation;(void)positiveDeviation;(void)variation;(void)positiveVariation;}
	virtual void onPHTRO(DoubleValue pitch, BoolValue bowUp,
						 DoubleValue roll, BoolValue portUp) {(void)pitch;(void)bowUp;(void)roll;(void)portUp;}
	virtual void onHCMTW(DoubleValue temperature) {(void)temperature;}
	virtual void onPRDID(DoubleValue pitch, DoubleValue roll, DoubleValue heading) {(void)pitch;(void)roll;(void)heading;}
	virtual void onPSONCMS(DoubleValue quat1, DoubleValue quat2, DoubleValue quat3, DoubleValue quat4,
						   DoubleValue acc_x, DoubleValue acc_y, DoubleValue acc_z,
						   DoubleValue omega_x, DoubleValue omega_y, DoubleValue omega_z,
						   DoubleValue mag_x, DoubleValue mag_y, DoubleValue mag_z,
						   DoubleValue temperature) {(void) quat1;(void)quat2;(void)quat3;(void)quat4;
													 (void) acc_x;(void)acc_y;(void)acc_z;
													 (void) omega_x;(void)omega_y;(void)omega_z;
													 (void) mag_x;(void)mag_y;(void)mag_z;
													 (void) temperature;}
	virtual void onTSS2(DoubleValue heading, DoubleValue heave, DoubleValue roll, DoubleValue pitch) {(void)heading;(void)heave;(void)roll;(void)pitch;}
	virtual void onEM1000(DoubleValue roll, DoubleValue pitch, DoubleValue heave, DoubleValue heading) {(void)heading;(void)heave;(void)roll;(void)pitch;}
	virtual void onHEHDT(DoubleValue heading) {(void)heading;}
	virtual void onHEROT(DoubleValue rateOfTurn) {(void)rateOfTurn;}
};

inline IParserObserver::~IParserObserver() throw() {}

}

#endif
