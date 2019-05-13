
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

#ifndef THREADING_H
#define THREADING_H

#include "xsens_mutex.h"

#ifndef __GNUC__
#pragma warning(disable: 4127)
#endif

namespace xsens {
	/*!	\class StandardThread
		\brief A class for a standard thread that has to perform the same action repeatedly.
		\details The class has three virtual functions, of which the innerFunction is the most important.
		innerFunction gets called repeatedly and is expected to return so that StandardThread can
		check for thread termination.
	*/
	class StandardThread {
	private:
		XsThread m_thread;
		XsThreadPriority m_priority;

	protected:
		volatile std::atomic_bool m_stop;	//!< Indicates that the thread should stop. Derived classes should check isTerminating() instead of directly polling this value when checking if the thread should stop. However, there are some cases (tests, SignallingThread) where direct access from within the class is desired, which is why the vlaue is protected instead of private.
#ifdef _WIN32
		HANDLE m_stopHandle;	//!< Duplicates m_stop functionality for external dependent classes such as Semaphore
		HANDLE m_running;		//!< Indicates that the thread is running
#else
		pthread_attr_t m_attr;	//!< Duplicates m_stop functionality for external dependent classes such as Semaphore
		bool m_running;			//!< Indicates that the thread is running
#endif
		volatile std::atomic_bool m_yieldOnZeroSleep;	//!< When true, a sleep value of 0 returned by innerFunction will trigger a thread yield operation. When false, the next cycle is started immediately.
	private:
		char* m_name;
		static XSENS_THREAD_RETURN threadInit(void *obj);
#ifndef _WIN32
		static void threadCleanup(void *obj);
#endif
		XsThreadId m_threadId;
		void threadMain(void);
	protected:
		//! Virtual initialization function
		virtual void initFunction(void) { }

		//! Virtual exit function
		virtual void exitFunction(void) { }

		//! Virtual inner function
		virtual int32_t innerFunction(void) { return 0; }
	public:
		StandardThread();
		virtual ~StandardThread();

		bool startThread(const char* name=NULL);
		void signalStopThread(void);
		void stopThread(void);
		bool isAlive(void) const;
		bool isRunning(void) const;
		bool setPriority(XsThreadPriority pri);
		bool isTerminating() const;

		//! \returns The thread ID
		XsThreadId getThreadId(void) const { return m_threadId; }
#ifdef _WIN32
		void terminateThread();
#endif
	};
	#define XSENS_THREAD_CHECK	if (isTerminating()) return 0;

#ifndef SWIG
	typedef void (__cdecl *WatchDogFunction)(void*);

	/*! \class WatchDogThread
		\brief A class that keeps an eye on a threads timer
	*/
	class WatchDogThread {
	private:
		XsThread m_thread;
#ifdef _WIN32
		HANDLE m_stop;
		HANDLE m_running;
		HANDLE m_reset;
#else
		pthread_attr_t m_attr;
		Mutex m_mutex;
		bool m_running;
		bool m_reset;
		bool m_stop;
#endif
		volatile std::atomic<std::uint32_t> m_timeout;
		WatchDogFunction m_func;
		void* m_param;
		char* m_name;
		static XSENS_THREAD_RETURN threadInit(void* obj);

		XsThreadId m_threadId;
		void threadMain(void);
		bool isAlive(void);
		bool isRunning(void);
	public:

		/*! \brief Constructor
		*/
		WatchDogThread(WatchDogFunction func, void* param = NULL);

		/*! \brief Destructor
		*/
		~WatchDogThread();

		bool resetTimer(uint32_t timeout = 0);
		bool startTimer(uint32_t timeout = 10000, const char* name=NULL);
		bool stopTimer(void);

		//! \returns The thread ID
		XsThreadId getThreadId(void) { return m_threadId; }
	};

	/*!	\class TaskThread
		\brief Class for handling small tasks
		\details Use this class if you have small tasks that need to be performed out of the main thread
		The thread uses tasks supplied in a TaskType struct.
	*/
	class TaskThread : public StandardThread {
	public:
		typedef void (__cdecl *TaskFunction)(void*); //!< A function prototype for a task
	private:
		struct TaskType {
			TaskFunction m_function;
			void*	m_param;
		};

		std::deque<TaskType> m_queue;
		Mutex m_safe;
		bool m_inFunc;
	protected:

		/*! \brief The inner function of the task thread.
			\details The function checks if there is a task in the queue and executes it.
			Then it returns to the StandardThread main loop to check for termination.
			If there are no tasks in the queue, the thread will terminate itself.
			\returns Not 0 if successful
		*/
		virtual int32_t innerFunction(void)
		{
			Lock safety(&m_safe);
			if (m_queue.size() > 0)
			{
				TaskType& task = m_queue.front();
				m_queue.pop_front();
				m_inFunc = true;
				safety.unlock();
				task.m_function(task.m_param);
				m_inFunc = false;
				return 0;	// no sleep
			}
			// notify thread to terminate when there are no more tasks
			stopThread();
			return 0;
		}
	public:
		TaskThread() : m_inFunc(false) {}
		~TaskThread()
		{
			while(isRunning() && getLength())
				XsTime::msleep(100);
		}

		/*! \brief Adds a task to a queue
			\param func The function to add
			\param param The parameters to add
		*/
		void addTask(TaskFunction func, void* param)
		{
			Lock safety(&m_safe);
			TaskType tmp = {func,param};
			m_queue.push_back(tmp);
		}

		//! \returns The length of a queue
		int32_t getLength(void) {
			Lock safety(&m_safe);
			return (int32_t) m_queue.size() + (m_inFunc?1:0);
		}

		//! \brief Clears a queue
		void clear(void)
		{
			Lock safety(&m_safe);
			m_queue.clear();
		}
	};
#endif
} // namespace xsens

#ifndef __GNUC__
#pragma warning(default: 4127)
#endif

#endif
