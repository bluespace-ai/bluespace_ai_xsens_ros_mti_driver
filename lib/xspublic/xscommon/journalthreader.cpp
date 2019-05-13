
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

#include "journalthreader.h"
#include "journalfile.h"
#if JOURNALLER_WITH_THREAD_SUPPORT
#include "xsens_threadpool.h"
#include "xsens_mutex.h"
using namespace xsens;
#else
#ifdef XSENS_WINDOWS
#include <windows.h>
#endif
#endif

#ifndef XSENS_WINDOWS
static void OutputDebugStringA(const char *msg)
{
	(void)fprintf(stderr, "%s", msg);
}
#endif

/*! \brief Storage for logging queue of a specific file in a specific thread
*/
class ThreadLine {
public:
	std::string m_line;
	JournalLogLevel m_level;

	ThreadLine() : m_level(JLL_None) {}
};

#if JOURNALLER_WITH_THREAD_SUPPORT
#define LINEMAPLOCK()	Lock lock(&m_mutex)
#else
#define LINEMAPLOCK()	((void)0)
#endif

/*! \brief Class for making the LineMap access thread-safe
*/
class LineMap {
	typedef std::map<int, ThreadLine*> LineMapInternal;
	typedef LineMapInternal::iterator iterator;
	LineMapInternal m_map;
#if JOURNALLER_WITH_THREAD_SUPPORT
	mutable Mutex m_mutex;
#endif
public:
	ThreadLine* operator[](int item)
	{
		LINEMAPLOCK();
		iterator it = m_map.find(item);
		if (it != m_map.end())
			return it->second;
		ThreadLine* line = new ThreadLine;
		m_map[item] = line;
		return line;
	}

	void eraseAll()
	{
		LINEMAPLOCK();
		while (m_map.size())
		{
			LineMap::iterator it = m_map.begin();
			delete it->second;
			m_map.erase(it);
		}
	}
};

/*! \brief Class for storing information about a specific logfile.
	\details This class is only concerned about threading issues, not about file management.
*/
class FileInfo {
public:
	FileInfo()
		: m_file(0)
#if JOURNALLER_WITH_THREAD_SUPPORT
		, m_poolIndex(0)
#endif
	{}

	//! \brief Destructor, erases the LineMap items
	~FileInfo()
	{
#if JOURNALLER_WITH_THREAD_SUPPORT
		try {
			m_lines.eraseAll();
		} catch(...)
		{}
#endif
		m_file = 0;
	}

	JournalFile* m_file;

#if JOURNALLER_WITH_THREAD_SUPPORT
	LineMap m_lines;
	unsigned int m_poolIndex;
	Mutex m_mutex;
#else
	ThreadLine m_line;
#endif

	//! \brief Return the ThreadLine object associated with the given \a thread
	ThreadLine* threadLine(int thread)
	{
#if JOURNALLER_WITH_THREAD_SUPPORT
		return m_lines[thread];
#else
		return &m_line;
#endif
	}
};

/*! \brief Class for actual writing of data to a logfile
*/
class JournalTask
#if JOURNALLER_WITH_THREAD_SUPPORT
	: public ThreadPoolTask
#endif
{
public:
	//! \brief Write data to a logfile or debug output
	bool exec()
	{
		if (m_file->m_file)
			*(m_file->m_file) << m_line;
		else
			OutputDebugStringA(m_line.c_str());
		return true;
	}

	JournalTask(FileInfo* file, const std::string& line) :
#if JOURNALLER_WITH_THREAD_SUPPORT
		ThreadPoolTask(),
#endif
		m_file(file),
		m_line(line)
	{}
	virtual ~JournalTask() {}

	FileInfo* m_file;		//!< The file to write to or NULL to write to debug output
	std::string m_line;		//!< The text to write
};

/*! \class JournalThreader
	\brief Manages threaded writes for the Journaller objects
*/

/*! \brief Constructor, sets up the necessary structures for threaded logging */
JournalThreader::JournalThreader()
{}

/*! \brief Destructor, flushes all remaining data
*/
JournalThreader::~JournalThreader()
{
	try {
		while (m_files.size())
		{
			FileInfo* info = m_files.begin()->second;
			flushFile(info->m_file);
			m_files.erase(info->m_file);
			delete info;
		}
	} catch (...)
	{}
}

/*! \brief Flush all data for the file
	\details This function blocks any new logging from taking place on the file and returns
	when the current queue is empty for \a file
	\param file The file to flush
*/
void JournalThreader::flushFile(JournalFile* file)
{
	FileInfoMap::iterator item = m_files.find(file);
	if (item == m_files.end())
		return;

	FileInfo* info = item->second;
	if (info->m_file)
		info->m_file->flush();
}

/*! \brief Remove the file
	\details This function removes \a file from the known file list
	\param file The file to remove
*/
void JournalThreader::removeFile(JournalFile* file)
{
	FileInfoMap::iterator item = m_files.find(file);
	if (item != m_files.end())
	{
		delete item->second;
		m_files.erase(item);
	}
}

/*! \brief Write a message to \a file when possible.
	\details This function writes \a line to \a file when possible.
	In a threaded situation the writing is delegated to a threadpool, otherwise it is simply
	locked with a mutex. When no threading is available at all (JOURNALLER_WITH_THREAD_SUPPORT == 0)
	the write is done immediately.
	\param file The file to write to. Supply NULL to write to the debug output
	\param msg The string to write.
*/
void JournalThreader::flushMessage(JournalFile* file, const std::string& msg)
{
	FileInfo* info = fileInfo(file);
	if (!info)
		return;
	JournalTask task(info, msg);
	task.exec();
}

/*! \return the FileInfo object associated with the given \a file
	\param file The journal file
*/
FileInfo* JournalThreader::fileInfo(JournalFile* file)
{
	FileInfoMap::iterator item = m_files.find(file);
	if (item != m_files.end())
		return item->second;

	// add to map
	FileInfo* info = new FileInfo;
	info->m_file = file;
#if JOURNALLER_WITH_THREAD_SUPPORT
	info->m_poolIndex = 0;
#endif
	m_files[file] = info;
	return info;
}

/*! \brief Set the log level of the queued line
	\param file The file to configure
	\param thread The thread id associated with the line
	\param level The log level to set
	\returns The old log level
*/
JournalLogLevel JournalThreader::setLineLevel(JournalFile* file, int thread, JournalLogLevel level)
{
	FileInfo* info = fileInfo(file);
	ThreadLine* tline = info->threadLine(thread);
	JournalLogLevel old = tline->m_level;
	tline->m_level = level;
	return old;
}

/*! \returns The log level of the queued line
	\param file The file to get from
	\param thread The thread id associated with the line
*/
JournalLogLevel JournalThreader::lineLevel(JournalFile* file, int thread)
{
	FileInfo* info = fileInfo(file);
	ThreadLine* tline = info->threadLine(thread);
	return tline->m_level;
}

/*! \returns The reference to the line associated with the given file and thread
	\param file The associated log file
	\param thread The thread id associated with the line, ignored when JOURNALLER_WITH_THREAD_SUPPORT == 0
*/
std::string& JournalThreader::line(JournalFile* file, int thread)
{
	FileInfo* info = fileInfo(file);
	ThreadLine* tline = info->threadLine(thread);
	return tline->m_line;
}

/*! \brief Flush data for all known JournalFiles
*/
void JournalThreader::flushAll()
{
	for (FileInfoMap::iterator item = m_files.begin(); item != m_files.end(); ++item)
		flushFile(item->second->m_file);
}
