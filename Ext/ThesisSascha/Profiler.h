/*
 * Profiler.h
 *
 *  Created on: Jun 23, 2014
 *      Author: meisteryeti
 */

#ifndef PROFILER_H_
#define PROFILER_H_

#ifdef MINSG_PROFILING
#include <Util/StringIdentifier.h>
#include <Util/Timer.h>
#include <unordered_map>

class ProfilerEvent {
public:
	ProfilerEvent(const char *sName);
	void start();
	void stop();
	//static void start(const char *sName);
	//static void stop(const char *sName);
	static void init();
	static void close();
	static void print();
private:
	static std::unordered_map<Util::StringIdentifier, float> sumTime;
	static std::unordered_map<Util::StringIdentifier, uint32_t> callTimes;
	//static std::fstream names_;
	//static std::fstream file_;
	//char* event_name_;
	Util::Timer timer;
	Util::StringIdentifier event_id_;
};

class ProfilerScope {
public:
	ProfilerScope(ProfilerEvent* event);
	~ProfilerScope();
private:
	ProfilerEvent* event_;
};

#define PROFILING_INIT() ProfilerEvent::init();
#define PROFILING_CLOSE() ProfilerEvent::close();
#define PROFILING_PRINT() ProfilerEvent::print();

#define PROFILE_SCOPE(_name) static ProfilerEvent _profiler_event(_name); ProfilerScope _profiler_scope(&_profiler_event);
//#define PROFILE_EVENT_START(_name) ProfilerEvent::start(_name);
//#define PROFILE_EVENT_STOP(_name) ProfilerEvent::stop(_name);

#else

#define PROFILING_INIT()
#define PROFILING_CLOSE()
#define PROFILING_PRINT()

#define PROFILE_SCOPE(_name)
//#define PROFILE_EVENT_START(_name)
//#define PROFILE_EVENT_STOP(_name)

#endif

#endif /* PROFILER_H_ */
