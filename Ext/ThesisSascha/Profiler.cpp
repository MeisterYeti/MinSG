/*
 * Profiler.cpp
 *
 *  Created on: Jun 23, 2014
 *      Author: meisteryeti
 */

#ifdef MINSG_PROFILING

#include "Profiler.h"

#include <iostream>

ProfilerEvent::ProfilerEvent(const char *sName) {
	event_id_ = Util::StringIdentifier(sName);
}

void ProfilerEvent::start() {
	timer.reset();
}

std::unordered_map<Util::StringIdentifier, float> ProfilerEvent::sumTime;
std::unordered_map<Util::StringIdentifier, uint32_t> ProfilerEvent::callTimes;
void ProfilerEvent::stop() {
	if(sumTime.count(event_id_) > 0) {
		sumTime[event_id_] = sumTime[event_id_] + timer.getMilliseconds();
		callTimes[event_id_] = callTimes[event_id_]+1;
	} else {
		sumTime[event_id_] = timer.getMilliseconds();
		callTimes[event_id_] = 1;
	}
}

void ProfilerEvent::init() {
	sumTime.clear();
	callTimes.clear();
}

void ProfilerEvent::close() {

}

void ProfilerEvent::print() {
	for(auto entry : sumTime) {
		uint32_t calls = callTimes[entry.first];
		std::cout << entry.first.toString() << ": " << calls << " calls, " << entry.second << "ms (avg " << (entry.second/calls) << "ms)" << std::endl;
	}
}


ProfilerScope::ProfilerScope(ProfilerEvent* event) : event_(event) {
	event_->start();
}
ProfilerScope::~ProfilerScope() {
	event_->stop();
}

#endif
