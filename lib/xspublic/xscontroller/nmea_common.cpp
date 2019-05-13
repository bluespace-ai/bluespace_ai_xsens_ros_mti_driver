
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

#include "nmea_common.h"

namespace nmea
{

const char * const Common::ID_HCHDM = "HCHDM";
const char * const Common::ID_HCHDG = "HCHDG";
const char * const Common::ID_PHTRO = "PHTRO";
const char * const Common::ID_HCMTW = "HCMTW";
const char * const Common::ID_PRDID = "PRDID";
const char * const Common::ID_PSONCMS = "PSONCMS";
const char * const Common::ID_TSS2 = ":";
const char * const Common::ID_EM1000 = "0x0";
const char * const Common::ID_HEHDT = "HEHDT";
const char * const Common::ID_HEROT = "HEROT";

const char Common::PREAMBLE = '$';
const char Common::PREAMBLE_TSS2 = ':';
const char Common::PREAMBLE_EM1000 = '\0';
const char Common::COMMA = ',';
const char Common::PLUS = '+';
const char Common::MINUS = '-';
const char Common::DECIMAL_POINT = '.';
const char Common::CHECKSUM_MARKER = '*';
const char Common::CR = '\r';
const char Common::LF = '\n';
const char Common::NMEA_TRUE = 'T';
const char Common::HCHDM_MAGNETIC = 'M';
const char Common::HCHDG_POSITIVE = 'E';
const char Common::HCHDG_NEGATIVE = 'W';
const char Common::PHTRO_BOW_UP = 'M';
const char Common::PHTRO_BOW_DOWN = 'P';
const char Common::PHTRO_PORT_UP = 'T';
const char Common::PHTRO_PORT_DOWN = 'B';
const char Common::HCMTW_CENTIGRADE = 'C';

}
