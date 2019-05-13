
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

#include "nmea_parser.h"
#include "xscontrollerconfig.h"
#include <xstypes/pstdint.h>

// Remove windows global min/max macros
#undef min
#undef max

namespace nmea
{

/*! \class Parser
	\brief Default constructor
*/
Parser::Parser() : m_reportIdMapIterator(m_reportIdMap.end())
{
	m_reportIdMap.insert(ReportIdMap::value_type(ID_HCHDM, &Parser::parseReport_HCHDM));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_HCHDG, &Parser::parseReport_HCHDG));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_PHTRO, &Parser::parseReport_PHTRO));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_HCMTW, &Parser::parseReport_HCMTW));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_PRDID, &Parser::parseReport_PRDID));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_PSONCMS, &Parser::parseReport_PSONCMS));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_TSS2, &Parser::parseReport_TSS2));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_EM1000, &Parser::parseReport_EM1000));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_HEHDT, &Parser::parseReport_HEHDT));
	m_reportIdMap.insert(ReportIdMap::value_type(ID_HEROT, &Parser::parseReport_HEROT));

	for (ReportIdMap::const_iterator i = m_reportIdMap.begin(); i != m_reportIdMap.end(); ++i)
	{
		assert(i->first.size() <= MAX_MESSAGE_IDENTIFIER_LENGTH);
	}
}

/*! \brief Destructor
*/
Parser::~Parser() throw()
{
}

/*! \brief Parses a message
	\param begin The begining of a message
	\param end The end of a message
	\returns The result of parsing
*/
Parser::ParseResult Parser::parseMessage(char const * begin, char const * end) const
{
	bool success = false;

	notifyInitializeParse(begin, end);

	CharBufferStream s(begin, end);

	std::string messageIdentifier;
	if (parsePreamble(s) && parseMessageIdentifier(s, messageIdentifier) && parseComma(s))
	{
		JLTRACEG("Found NMEA message");
		success = true;
	}
	else if (parseEm1000Preamble(s) && parseCharacter(s, '\x90'))
	{
		JLTRACEG("Found EM1000 message");
		success = true;
		messageIdentifier = ID_EM1000;
	}
	else if (parseTss2Preamble(s))
	{
		JLTRACEG("Found TSS2 message");
		success = true;
		messageIdentifier = ID_TSS2;
	}

	if (success)
	{
		if (m_reportIdMapIterator == m_reportIdMap.end() || (m_reportIdMapIterator->first != messageIdentifier))
		{
			m_reportIdMapIterator = m_reportIdMap.find(messageIdentifier);
		}

		if (m_reportIdMapIterator != m_reportIdMap.end())
		{
			ReportParser reportParser = m_reportIdMapIterator->second;
			if ((this->*reportParser)(s))
			{
				success = true;
			}
		}
	}

	return std::make_pair(success, s.size());
}

/*! \brief Parses a report of HCHDM
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_HCHDM(IStream& s) const
{
	bool success = false;
	DoubleValue heading_val;
	if (parseNumber(s, heading_val) && parseComma(s) && parseCharacter(s, HCHDM_MAGNETIC) && parseChecksum(s) && parseCrLf(s))
	{
		notifyHCHDM(heading_val);
		success = true;
	}
	return success;
}

/*! \brief Parses a report of HCHDG
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_HCHDG(IStream& s) const
{
	bool success = false;
	DoubleValue heading_val, deviation_val, variation_val;
	BoolValue positive1, positive2;
	if (parseNumber(s, heading_val) && parseComma(s) &&
		parseNumber(s, deviation_val) && parseComma(s) &&
		parseTrueFalse(s, HCHDG_POSITIVE, HCHDG_NEGATIVE, positive1) && parseComma(s) &&
		parseNumber(s, variation_val) && parseComma(s) &&
		parseTrueFalse(s, HCHDG_POSITIVE, HCHDG_NEGATIVE, positive2) &&
		parseChecksum(s) && parseCrLf(s))
	{
		notifyHCHDG(heading_val, deviation_val, positive1, variation_val, positive2);
		success = true;
	}

	return success;
}

/*! \brief Parses a report of PHTRO
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_PHTRO(IStream& s) const
{
	bool success = false;
	DoubleValue pitch, roll;
	BoolValue bowUp, portUp;
	if (parseNumber(s, pitch) && parseComma(s) &&
		parseTrueFalse(s, PHTRO_BOW_UP, PHTRO_BOW_DOWN, bowUp) && parseComma(s) &&
		parseNumber(s, roll) && parseComma(s) &&
		parseTrueFalse(s, PHTRO_PORT_UP, PHTRO_PORT_DOWN, portUp) &&
		parseChecksum(s) && parseCrLf(s))
	{
		notifyPHTRO(pitch, bowUp, roll, portUp);
		success = true;
	}
	return success;
}

/*! \brief Parses a report of HCMTW
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_HCMTW(IStream& s) const
{
	bool success = false;
	DoubleValue temperature;
	if (parseNumber(s, temperature) && parseComma(s) && parseCharacter(s, HCMTW_CENTIGRADE) &&
		parseChecksum(s) && parseCrLf(s))
	{
		notifyHCMTW(temperature);
		success = true;
	}
	return success;
}

/*! \brief Parses a report of PRDID
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_PRDID(IStream& s) const
{
	bool success = false;
	DoubleValue pitch, roll, heading;
	if (parseNumber(s, pitch) && parseComma(s) && parseNumber(s, roll) && parseComma(s) &&
		parseNumber(s, heading) && parseCrLf(s))
	{
		notifyPRDID(pitch, roll, heading);
		success = true;
	}

	return success;
}

/*! \brief Parses a report of PSONCMS
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_PSONCMS(IStream& s) const
{
	bool success = false;
	DoubleValue q1, q2, q3, q4, acc_x, acc_y, acc_z, omega_x, omega_y, omega_z, mag_x, mag_y, mag_z, sensor_temp;
	if (parseNumber(s, q1) && parseComma(s) &&
		parseNumber(s, q2) && parseComma(s) &&
		parseNumber(s, q3) && parseComma(s) &&
		parseNumber(s, q4) && parseComma(s) &&
		parseNumber(s, acc_x) && parseComma(s) &&
		parseNumber(s, acc_y) && parseComma(s) &&
		parseNumber(s, acc_z) && parseComma(s) &&
		parseNumber(s, omega_x) && parseComma(s) &&
		parseNumber(s, omega_y) && parseComma(s) &&
		parseNumber(s, omega_z) && parseComma(s) &&
		parseNumber(s, mag_x) && parseComma(s) &&
		parseNumber(s, mag_y) && parseComma(s) &&
		parseNumber(s, mag_z) && parseComma(s) &&
		parseNumber(s, sensor_temp) &&
		parseChecksum(s) && parseCrLf(s))
	{
		notifyPSONCMS(q1, q2, q3, q4, acc_x, acc_y, acc_z, omega_x, omega_y, omega_z, mag_x, mag_y, mag_z, sensor_temp);
		success = true;
	}

	return success;
}

/*! \brief Parses a report of TSS2
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_TSS2(IStream& s) const
{
	bool success = false;
	DoubleValue heading, heave, roll, pitch;
	char anyChar;

	if (parseNumber(s, heading) &&
		parseWhiteSpace(s) &&
		parseNumber(s, heave) &&
		parseAnyCharacter(s, anyChar) && // Status Flag (ignored)
		parseWhiteSpace(s) &&
		parseNumber(s, roll) &&
		parseWhiteSpace(s) &&
		parseNumber(s, pitch) &&
		parseAnyCharacter(s, anyChar) && // Headig Status Flag (ignored)
		parseCrLf(s))
	{
		notifyTSS2(heading, heave, roll, pitch);
		success = true;
	}

	return success;
}

/*! \brief Parses a report of EM1000
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_EM1000(IStream& s) const
{
	bool success = false;
	double iRoll, iPitch, iHeave, iHeading;
	if (parseInt16(s, iRoll) && parseInt16(s, iPitch) && parseInt16(s, iHeave) && parseUint16(s, iHeading))
	{
		notifyEM1000((DoubleValue)iRoll, (DoubleValue)iPitch, (DoubleValue)iHeave, (DoubleValue)iHeading);
		success = true;
	}

	return success;
}

/*! \brief Parses a report of HEHDT
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_HEHDT(IStream& s) const
{
	bool success = false;
	DoubleValue heading_val;
	if (parseNumber(s, heading_val) && parseComma(s) && parseCharacter(s, NMEA_TRUE) && parseChecksum(s) && parseCrLf(s))
	{
		notifyHEHDT(heading_val);
		success = true;
	}
	return success;
}

/*! \brief Parses a report of HEROT
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseReport_HEROT(IStream& s) const
{
	bool success = false;
	DoubleValue rateOfTurn_val;
	if (parseNumber(s, rateOfTurn_val) && parseComma(s) && parseCharacter(s, NMEA_TRUE) && parseChecksum(s) && parseCrLf(s))
	{
		notifyHEROT(rateOfTurn_val);
		success = true;
	}
	return success;
}

/*! \brief Parses a preamble
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parsePreamble(IStream& s) const
{
	bool success = false;
	if (parseCharacter(s, PREAMBLE))
	{
		s.begin_checksum();
		success = true;
	}
	return success;
}

/*! \brief Parses a TSS2 preamble
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseTss2Preamble(IStream &s) const
{
	bool success = false;
	if (parseCharacter(s, PREAMBLE_TSS2))
	{
		success = true;
	}
	return success;
}

/*! \brief Parses a EM1000 preamble
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseEm1000Preamble(IStream &s) const
{
	bool success = false;
	if (parseCharacter(s, PREAMBLE_EM1000))
	{
		success = true;
	}
	return success;
}

/*! \brief Parses a comma
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseComma(IStream& s) const
{
	return parseCharacter(s, COMMA);
}

/*! \brief Parses true and false
	\param s The stream of data
	\param trueChar The character value for true
	\param falseChar The character value for false
	\param trueVal The true boolean value
	\returns True if successful
*/
bool Parser::parseTrueFalse(IStream& s, char const & trueChar, char const & falseChar, BoolValue& trueVal) const
{
	trueVal.setValid(false);
	bool success = true; // may be empty as well
	if (s.valid() && (s.peek() == trueChar || s.peek() == falseChar))
	{
		trueVal = BoolValue(s.peek() == trueChar);
		trueVal.setValid(true);
		s.next();
		success = true;
	}
	return success;
}

/*! \brief Parses a checsum marker
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseChecksumMarker(IStream& s) const
{
	bool success = false;
	if (s.valid() && s.peek() == CHECKSUM_MARKER)
	{
		s.end_checksum();
		s.next();
		success = true;
	}
	return success;
}

/*! \brief Parses a checsum
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseChecksum(IStream& s) const
{
	bool success = false;
	char msw_char;
	char lsw_char;
	unsigned char msw_decimal;
	unsigned char lsw_decimal;
	if (parseChecksumMarker(s) && parseAnyCharacter(s, msw_char) && parseAnyCharacter(s, lsw_char) &&
		hexDigitToDecimal(msw_char, msw_decimal) && hexDigitToDecimal(lsw_char, lsw_decimal))
	{
		unsigned char parsedChecksum = (msw_decimal << 4) | lsw_decimal;
		unsigned char calculatedChecksum = s.get_checksum();
		if (parsedChecksum == calculatedChecksum)
		{
			success = true;
		}
	}
	return success;
}

/*! \brief Parses an value of int16 type
	\param s The stream of data
	\param number The reference in which the parsed number will be stored
	\returns True if successful
*/
bool Parser::parseInt16(IStream &s, double &number) const
{
	bool success = false;
	char msb;
	char lsb;

	if (parseAnyCharacter(s, lsb) && parseAnyCharacter(s, msb))
	{
		number = ((double)((int16_t)((uint8_t)lsb + ((uint8_t)msb << 8))));
		success = true;
	}

	return success;
}

/*! \brief Parses an value of uint16 type
	\param s The stream of data
	\param number The reference in which the parsed number will be stored
	\returns True if successful
*/
bool Parser::parseUint16(IStream &s, double &number) const
{
	bool success = false;
	char msb;
	char lsb;

	if (parseAnyCharacter(s, lsb) && parseAnyCharacter(s, msb))
	{
		number = ((double)((uint16_t)((uint8_t)lsb + ((uint8_t)msb << 8))));
		success = true;
	}

	return success;
}

/*! \brief Parses a message identifier
	\param s The stream of data
	\param id The reference in which the parsed id will be stored
	\returns True if successful
*/
inline bool Parser::parseMessageIdentifier(IStream& s, std::string& id) const
{
	bool success = false;
	if (s.valid() && isLetter(s.peek()))
	{
		success = true;
		id.clear();
		while (s.valid() && isalpha((unsigned char)s.peek()) && id.size() <= MAX_MESSAGE_IDENTIFIER_LENGTH)
		{
			id.push_back(s.peek());
			s.next();
		}

	}
	return success;
}

/*! \brief Parses a number
	\param s The stream of data
	\param number The reference in which the parsed number will be stored
	\returns True if successful
*/
bool Parser::parseNumber(IStream& s, DoubleValue& number) const
{
	bool success = true; // empty numbers parse ok as well but the resulting number will be invalid.
	//bool positive = true;
	number.setValid(false);
	std::string parsedNumber;

	if (s.valid())
	{
		if (s.peek() == PLUS)
		{
			//positive = true;
			parsedNumber += s.peek();
			s.next();
		}
		else if (s.peek() == MINUS)
		{
			//positive = false;
			parsedNumber += s.peek();
			s.next();
		}

		if (s.valid() && isdigit(s.peek()))
		{
			char digit_char;
			unsigned int digit;
			while (parseDigit(s, digit_char, digit))
			{
				parsedNumber += digit_char;
				success = true;
				number.setValid(true);
			}
			if (s.valid() && s.peek() == DECIMAL_POINT)
			{
				number.setValid(false);
				parsedNumber += s.peek();
				s.next();
				success = false;
				if (s.valid() && isdigit(s.peek()))
				{
					while (parseDigit(s, digit_char, digit))
					{
						parsedNumber += digit_char;
						success = true;
						number.setValid(true);
					}
				}
			}
		}
		if (success && number.valid())
		{
			char const * beginptr = parsedNumber.c_str();
			char* endptr = 0;

			number = DoubleValue(strtod(beginptr, &endptr));
			success = static_cast<size_t>(endptr - beginptr) == parsedNumber.size();
		}
	}

	return success;
}

/*! \brief Parses a character
	\param s The stream of data
	\param character The reference in which the parsed character is stored
	\returns True if successful
*/
bool Parser::parseCharacter(IStream& s, char const & character) const
{
	bool success = false;
	if (s.valid() && s.peek() == character)
	{
		s.next();
		success = true;
	}
	return success;
}

/*! \brief Parses an any character
	\param s The stream of data
	\param character The reference in which the parsed character will be stored
	\returns True if successful
*/
bool Parser::parseAnyCharacter(IStream& s, char& character) const
{
	bool success = false;
	if (s.valid())
	{
		character = s.peek();
		s.next();
		success = true;
	}
	return success;
}

/*! \brief Parses an white space
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseWhiteSpace(IStream& s) const
{
	while (s.valid() && (s.peek() == '\t' || s.peek() == ' ')) {
		s.next();
	}
	return true;
}

/*! \brief Parses a digit
	\param s The stream of data
	\param c The reference in which the parsed character will be stored
	\param digit The reference in which the parsed digit will be stored
	\returns True if successful
*/
bool Parser::parseDigit(IStream& s, char& c, unsigned int& digit) const
{
	bool success = false;

	if (s.valid())
	{
		if (isDigit(s.peek()))
		{
			c = s.peek();
			digit = (unsigned char) (c - '0');
			s.next();
			success = true;
		}
	}
	return success;
}

/*! \brief Does the convertion of digit from hex to dec
	\param hexDigit The hex digit to convert
	\param decimal The reference in which the parsed decimal digit will be stored
	\returns True if successful
*/
inline bool Parser::hexDigitToDecimal(char hexDigit, unsigned char& decimal) const
{
	bool success = false;
	if (hexDigit >= '0' && hexDigit <= '9')
	{
		decimal = hexDigit - '0';
		success = true;
	}
	else if (hexDigit >= 'A' && hexDigit <= 'F')
	{
		decimal = hexDigit - 'A' + 10;
		success = true;
	}
	else if (hexDigit >= 'a' && hexDigit <= 'b')
	{
		decimal = hexDigit - 'a' + 10;
		success = true;
	}
	return success;
}

/*! \brief Parses CR and LF values
	\param s The stream of data
	\returns True if successful
*/
bool Parser::parseCrLf(IStream& s) const
{
	bool success = false;
	if (s.valid() && s.peek() == CR)
	{
		s.next();
		if (s.valid() && s.peek() == LF)
		{
			s.next();
			success = true;
		}
	}
	return success;
}

/*! \brief Checks if character is a digit
	\param c The character to check
	\returns True if successful
*/
inline bool Parser::isDigit(char c) const
{
	return (c >= '0') && (c <= '9');
}

/*! \brief Checks if character is a letter
	\param c The character to check
	\returns True if successful
*/
inline bool Parser::isLetter(char c) const
{
	return ((c >= 'A') && (c <= 'Z')) || ((c >= 'a') && (c <= 'z'));
}



}
