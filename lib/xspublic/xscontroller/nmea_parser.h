
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

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include "nmea_parsersubject.h"
#include "nmea_common.h"

#include <map>
#include <cassert>

namespace nmea
{


class Parser : public Common, public virtual ParserSubject
{
public:
	Parser();
	virtual ~Parser() throw();
	virtual ParseResult parseMessage(char const * begin, char const * end) const;

	/*! \class IStream
		\brief An internal abstract class
	*/
	class IStream
	{
	public:
		//! Destroys IStream
		virtual ~IStream() throw() {}

		//! Checks for validity of stream
		virtual bool valid() const = 0;

		//! Peeks the data from stream
		virtual char peek() const = 0;

		//! Returns size of the stream
		virtual size_t size() const = 0;

		//! Jumps to next data in a stream
		virtual void next() = 0;

		//! Begins checksum of stream
		virtual void begin_checksum() = 0;

		//! Ends checksum of the stream
		virtual void end_checksum() = 0;

		//! Gets checksum of the stream
		virtual unsigned char get_checksum() const = 0;
	};

private:
	class StreamBase;
	class CharBufferStream;

	typedef bool (Parser::*ReportParser)(IStream& s) const;
	typedef std::map<std::string, ReportParser> ReportIdMap; // boost/unordered/unordered_map.hpp is missing?

	bool parseReport_HCHDM(IStream& s) const;
	bool parseReport_HCHDG(IStream& s) const;
	bool parseReport_PHTRO(IStream& s) const;
	bool parseReport_HCMTW(IStream& s) const;
	bool parseReport_PRDID(IStream& s) const;
	bool parseReport_PSONCMS(IStream& s) const;
	bool parseReport_TSS2(IStream& s) const;
	bool parseReport_EM1000(IStream& s) const;
	bool parseReport_HEHDT(IStream& s) const;
	bool parseReport_HEROT(IStream& s) const;

	bool parsePreamble(IStream& s) const;
	bool parseTss2Preamble(IStream& s) const;
	bool parseEm1000Preamble(IStream& s) const;
	bool parseChecksumMarker(IStream& s) const;
	bool parseMessageIdentifier(IStream& s, std::string& id) const;
	bool parseChecksum(IStream& s) const;
	bool parseCharacter(IStream& s, char const & character) const;
	bool parseAnyCharacter(IStream& s, char& character) const;
	bool parseNumber(IStream& s, DoubleValue& number) const;
	bool parseDigit(IStream& s, char& c, unsigned int& digit) const;
	bool parseCrLf(IStream& s) const;
	bool parseTrueFalse(IStream& s, char const & trueChar, char const & falseChar, BoolValue& trueVal) const;
	bool parseComma(IStream& s) const;
	bool parseInt16(IStream& s, double& number) const;
	bool parseUint16(IStream &s, double &number) const;
	bool parseWhiteSpace(IStream& s) const;

	inline bool isDigit(char c) const;
	inline bool isLetter(char c) const;
	inline bool hexDigitToDecimal(char hexDigit, unsigned char& decimal) const;

	ReportIdMap m_reportIdMap;
	mutable ReportIdMap::const_iterator m_reportIdMapIterator;
};

/*! \class Parser::StreamBase
	\brief A base class for the stream
*/
class Parser::StreamBase : public virtual IStream
{
public:
	StreamBase() : m_size(0), m_checksum(0), m_update_checksum(false) {}
	virtual void next() {assert(valid()); if (m_update_checksum) {m_checksum ^= peek();} ++m_size;}
	virtual size_t size() const {return m_size;}
	void begin_checksum() {m_update_checksum = true;}
	void end_checksum() {m_update_checksum = false;}
	unsigned char get_checksum() const {return m_checksum;}
private:
	size_t m_size;
	unsigned char m_checksum;
	bool m_update_checksum;
};

/*! \class Parser::CharBufferStream
	\brief A character buffer class for the stream
*/
class Parser::CharBufferStream : public StreamBase
{
public:
	//! A typedef for an iterator
	typedef char const * Iterator;

	//! Default constructor
	CharBufferStream(Iterator begin, Iterator end) :
		m_begin(begin),
		m_end(end),
		m_head(begin)
	{
		(void)m_begin; // m_begin is generally unused, but we'll leave it here anyway
	}

	bool valid() const {return m_head < m_end;}
	char peek() const {assert(valid()); return *m_head;}
	virtual void next() {StreamBase::next(); ++m_head;}

private:
	Iterator m_begin;
	Iterator m_end;
	Iterator m_head;
};



}

#endif
