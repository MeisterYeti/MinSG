#ifndef MINSG_THESISSASCHA_REQUESTQUEUE_H_
#define MINSG_THESISSASCHA_REQUESTQUEUE_H_

#include "Definitions.h"

#include <Util/Concurrency/Lock.h>
#include <Util/Concurrency/Mutex.h>
#include <Util/Concurrency/Semaphore.h>
#include <Util/Concurrency/Concurrency.h>

#include <set>
#include <functional>
#include <memory>
#include <deque>

namespace MinSG {
namespace ThesisSascha {

template<class CacheObject, uint32_t QueueCount>
class RequestQueue {
	public:
		typedef std::function<float(const CacheObject*)> PriorityFn_t;
	private:
		//typedef std::set<CacheObject*, std::function<bool(const CacheObject*, const CacheObject*)>> InternalQueue_t;
		typedef std::deque<CacheObject*> InternalQueue_t;

		std::array<InternalQueue_t, QueueCount> queues;
		std::unique_ptr<Util::Concurrency::Mutex> mutex;
		std::unique_ptr<Util::Concurrency::Semaphore> semaphore;

		float lastFramePriorityMin = 0;
		float lastFramePriorityMax = 0;
		float currentFramePriorityMin = std::numeric_limits<float>::max();
		float currentFramePriorityMax = 0;

		bool sortRequests = false;
		uint32_t maxSize = 0;
		PriorityFn_t priorityFn = [](const CacheObject* obj) { return 0; };
	public:

		RequestQueue() :
				mutex(Util::Concurrency::createMutex()), semaphore(Util::Concurrency::createSemaphore()) {
			for (uint32_t i = 0; i < QueueCount; ++i)
				queues[i] = InternalQueue_t();
				//queues[i] = InternalQueue_t([this](const CacheObject* oc1, const CacheObject* oc2) {return getPriority(oc1) < getPriority(oc2);});
		}

		bool push(CacheObject* obj) {
			LOCK(*mutex);
			if(pushInternal(obj)) {
				semaphore->post();
				return true;
			}
			return false;
		}

		CacheObject* tryPopFront() {
			if (!semaphore->tryWait())
				return nullptr;
			LOCK(*mutex);
			return popFrontInternal();
		}

		CacheObject* tryPopBack() {
			if (!semaphore->tryWait())
				return nullptr;
			LOCK(*mutex);
			return popBackInternal();
		}

		CacheObject* popFront() {
			semaphore->wait();
			LOCK(*mutex);
			return popFrontInternal();
		}

		CacheObject* popBack() {
			semaphore->wait();
			LOCK(*mutex);
			return popBackInternal();
		}

		bool empty() const {
			return size() <= 0;
		}

		void clear() {
			LOCK(*mutex);
			for (uint32_t i = 0; i < QueueCount; ++i) {
				queues[i].clear();
			}
			lastFramePriorityMax = 1;
			lastFramePriorityMin = 0;
			currentFramePriorityMin = std::numeric_limits<float>::max();
			currentFramePriorityMax = 0;
		}

		size_t size() const {
			LOCK(*mutex);
			return sizeInternal();
		}

		size_t size(uint32_t p) const {
			LOCK(*mutex);
			return queues[clamp < uint32_t > (p, 0, QueueCount - 1)].size();
		}

		void update() {
			LOCK(*mutex);
			// resort some elements to avoid starvation
			for (uint32_t i = 0; i < QueueCount; ++i) {
				if(!queues[i].empty()) {
					auto co = queues[i].back();
					queues[i].pop_back();
					uint32_t p = clamp < uint32_t > (getQueueIndexFromPriority(co), 0, QueueCount - 1);
					queues[p].push_front(co);
				}
			}
			if(sortRequests) {
				// fully sort the first queue with elements
				for (uint32_t i = 0; i < QueueCount; ++i) {
					if(!queues[i].empty()) {
						std::sort(queues[i].begin(), queues[i].end(), [this](const CacheObject* oc1, const CacheObject* oc2) {return getPriority(oc1) < getPriority(oc2);});
						break;
					}
				}
			}
			if (currentFramePriorityMax > currentFramePriorityMin) {
				lastFramePriorityMax = currentFramePriorityMax;
				lastFramePriorityMin = currentFramePriorityMin;
			}
			currentFramePriorityMin = std::numeric_limits<float>::max();
			currentFramePriorityMax = 0;
		}

		void setMaxSize(uint32_t value) {
			LOCK(*mutex);
			maxSize = value;
		}

		void setSortRequests(bool value) {
			LOCK(*mutex);
			sortRequests = value;
		}

		void setPriorityFunction(const PriorityFn_t& fn) {
			LOCK(*mutex);
			priorityFn = fn;
		}
	private:

		inline float getPriority(const CacheObject* obj) {
			if (obj == nullptr)
				return 0;
			return priorityFn(obj);
		}

		inline uint32_t getQueueIndexFromPriority(CacheObject* obj) {
			float priority = getPriority(obj);
			currentFramePriorityMin = std::min(currentFramePriorityMin, priority);
			currentFramePriorityMax = std::max(currentFramePriorityMax, priority);
			priority = clamp<float>(priority - lastFramePriorityMin, 0, lastFramePriorityMax - lastFramePriorityMin);
			float range = lastFramePriorityMax - lastFramePriorityMin;
			return range > 0 ? std::floor((priority / range) * (QueueCount - 1)) : 0;
		}

		inline size_t sizeInternal() const {
			uint32_t count = 0;
			for (uint32_t i = 0; i < QueueCount; ++i) {
				count += queues[i].size();
			}
			return count;
		}

		bool pushInternal(CacheObject* obj) {
			if (obj == nullptr || (maxSize > 0 && sizeInternal() >= maxSize))
				return false;
			uint32_t p = clamp < uint32_t > (getQueueIndexFromPriority(obj), 0, QueueCount - 1);
			queues[p].push_back(obj);
			//queues[p].insert(obj);
			return true;
		}

		CacheObject* popFrontInternal() {
			static uint32_t fairnessCounter = 0;
			++fairnessCounter;
			uint32_t start = 0;
			for (uint32_t i = 1; i < QueueCount; ++i) {
				if(fairnessCounter%(i*QueueCount)==i*QueueCount-1)
					start = i;
			}
			for (uint32_t i = start; i < QueueCount+start; ++i) {
				if (!queues[i%QueueCount].empty()) {
					auto co = queues[i%QueueCount].front();
					queues[i%QueueCount].pop_front();
					//auto it = queues[i].begin();
					//CacheObject* co = *it;
					//queues[i].erase(it);
					return co;
				}
			}
			return nullptr;
		}

		CacheObject* popBackInternal() {
			for (uint32_t i = QueueCount - 1; i >= 0; --i) {
				if (!queues[i].empty()) {
					auto co = queues[i].back();
					queues[i].pop_back();
					//auto it = --queues[i].end();
					//CacheObject* co = *it;
					//queues[i].erase(it);
					return co;
				}
			}
			return nullptr;
		}
};

}
}
#endif /* MINSG_THESISSASCHA_REQUESTQUEUE_H_ */
