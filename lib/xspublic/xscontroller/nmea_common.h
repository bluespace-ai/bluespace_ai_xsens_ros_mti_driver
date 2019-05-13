
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

#ifndef NMEA_COMMON_H
#define NMEA_COMMON_H

namespace nmea
{

class Common
{
protected:
	static const unsigned int MAX_MESSAGE_IDENTIFIER_LENGTH = 10;

	static const char PREAMBLE;
	static const char PREAMBLE_TSS2;
	static const char PREAMBLE_EM1000;
	static const char COMMA;
	static const char PLUS;
	static const char MINUS;
	static const char DECIMAL_POINT;
	static const char CHECKSUM_MARKER;
	static const char CR;
	static const char LF;
	static const char NMEA_TRUE;
	static const char HCHDM_MAGNETIC;
	static const char HCHDG_POSITIVE;
	static const char HCHDG_NEGATIVE;
	static const char PHTRO_BOW_UP;
	static const char PHTRO_BOW_DOWN;
	static const char PHTRO_PORT_UP;
	static const char PHTRO_PORT_DOWN;
	static const char HCMTW_CENTIGRADE;

	static const char * const ID_HCHDM;
	static const char * const ID_HCHDG;
	static const char * const ID_PHTRO;
	static const char * const ID_HCMTW;
	static const char * const ID_PRDID;
	static const char * const ID_PSONCMS;
	static const char * const ID_TSS2;
	static const char * const ID_EM1000;
	static const char * const ID_HEHDT;
	static const char * const ID_HEROT;

	virtual ~Common() {}
};

}

#endif	// file guard
