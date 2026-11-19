#include "SparkRadStreamer.h"

#include "RadTypes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <fstream>
#include <limits>
#include <thread>

struct SparkRadStreamer::Impl
{
	SparkRadStreamer* parent = nullptr;
	static constexpr size_t maxWorkers = 3;
	std::array<std::thread, maxWorkers> threads;
	std::mutex mutex;
	std::condition_variable cv;
	std::atomic<bool> running{false};
	std::string filename;
	std::streamoff chunkDataBase = 0;
	std::vector<RadChunkInfo>* chunks = nullptr;
	std::vector<uint32_t> pendingChunks;
	std::vector<RadRawChunk> rawChunkQueue;
	SparkRadStreamer::PrepareChunk prepare;
	size_t maxReadyChunks = 8;
	size_t workerCount = 1;
	std::uint64_t configurationGeneration = 0;
	struct InFlight {
		bool active = false;
		uint32_t chunkIndex = 0;
		uint32_t generation = 0;
		std::uint64_t configuration = 0;
	};
	std::array<InFlight, maxWorkers> inFlight;
	std::vector<size_t> requiredRank;

	// Caller holds queue then state mutex. Canceled slots still belong to their
	// physical worker, but do not delay the new view's batching or consume its
	// reserved ready capacity. There can never be more than maxWorkers of them.
	bool current(const InFlight& flight) const
	{
		if (!flight.active || flight.configuration != configurationGeneration || !chunks ||
		    flight.chunkIndex >= chunks->size())
			return false;
		const auto& chunk = (*chunks)[flight.chunkIndex];
		return chunk.queued && !chunk.resident && chunk.loadGeneration == flight.generation;
	}

	size_t currentInFlightCount() const
	{
		return size_t(std::count_if(inFlight.begin(), inFlight.end(),
			[this](const InFlight& flight) { return current(flight); }));
	}

	bool readyBefore(const RadRawChunk& left, const RadRawChunk& right) const
	{
		const size_t absent = std::numeric_limits<size_t>::max();
		const size_t leftRank = left.chunkIndex < requiredRank.size() ? requiredRank[left.chunkIndex] : absent;
		const size_t rightRank = right.chunkIndex < requiredRank.size() ? requiredRank[right.chunkIndex] : absent;
		if (leftRank != rightRank)
			return leftRank < rightRank;
		const float leftPriority = chunks && left.chunkIndex < chunks->size() ? (*chunks)[left.chunkIndex].priority : 0.0f;
		const float rightPriority = chunks && right.chunkIndex < chunks->size() ? (*chunks)[right.chunkIndex].priority : 0.0f;
		return leftPriority > rightPriority;
	}

	void run(size_t workerIndex)
	{
		while (running) {
			uint32_t chunkIndex = 0;
			RadChunkInfo info;
			std::string readFilename;
			std::streamoff readBase = 0;
			std::uint64_t readConfiguration = 0;
			SparkRadStreamer::PrepareChunk readPrepare;
			{
				std::unique_lock<std::mutex> lock(mutex);
				cv.wait(lock, [this, workerIndex] {
					if (!running.load()) return true;
					if (workerIndex >= workerCount || pendingChunks.empty()) return false;
					// Never retain the state lock across the condition-variable wait.
					std::lock_guard<std::mutex> stateLock(parent->_stateMutex);
					return rawChunkQueue.size() + currentInFlightCount() < maxReadyChunks;
				});
				if (!running)
					break;
				chunkIndex = pendingChunks.front();
				pendingChunks.erase(pendingChunks.begin());
				
				// Must lock stateMutex AFTER waking up to avoid deadlock during cv.wait
				std::lock_guard<std::mutex> stateLock(parent->_stateMutex); 
				if (!chunks || chunkIndex >= chunks->size())
					continue;
				info = (*chunks)[chunkIndex];
				if (info.resident || !info.queued || filename.empty()) {
					(*chunks)[chunkIndex].queued = false;
					cv.notify_all();
					continue;
				}
				readFilename = filename;
				readBase = chunkDataBase;
				readConfiguration = configurationGeneration;
				readPrepare = prepare;
				inFlight[workerIndex] = {true, chunkIndex, info.loadGeneration, readConfiguration};
			}
			RadRawChunk raw;
			raw.chunkIndex = chunkIndex;
			raw.loadGeneration = info.loadGeneration;
			bool success = false;
			// Validate the seek arithmetic before allocating or reading. Completion
			// still checks the generation because the view can change during I/O.
			try {
				if (readBase >= 0 && info.offset <= uint64_t(std::numeric_limits<std::streamoff>::max() - readBase) &&
				    info.bytes <= uint64_t(std::numeric_limits<std::streamsize>::max()) &&
				    info.bytes <= uint64_t(std::numeric_limits<size_t>::max())) {
					std::ifstream file(readFilename, std::ios::binary);
					if (file) {
						file.seekg(readBase + (std::streamoff)info.offset);
						raw.bytes.resize((size_t)info.bytes);
						file.read(reinterpret_cast<char*>(raw.bytes.data()), (std::streamsize)raw.bytes.size());
						success = (size_t)file.gcount() == raw.bytes.size();
					}
				}
				if (success && readPrepare) {
					// Skip expensive preparation if this read has already been
					// canceled. A second check at publication covers cancellation
					// during the callback itself; inFlight stays set throughout.
					{
						std::lock_guard<std::mutex> lock(mutex);
						std::lock_guard<std::mutex> stateLock(parent->_stateMutex);
						success = running && readConfiguration == configurationGeneration && chunks &&
							chunkIndex < chunks->size() && (*chunks)[chunkIndex].loadGeneration == info.loadGeneration &&
							(*chunks)[chunkIndex].queued && !(*chunks)[chunkIndex].resident;
					}
					if (success)
						success = readPrepare(raw, info);
				}
			} catch (...) {
				// A malformed chunk or failed processor must release only its
				// own request, never terminate the persistent streaming thread.
				success = false;
			}
			{
				std::lock_guard<std::mutex> lock(mutex);
				std::lock_guard<std::mutex> stateLock(parent->_stateMutex);
				inFlight[workerIndex].active = false;
				if (readConfiguration == configurationGeneration && chunks && chunkIndex < chunks->size()) {
					RadChunkInfo& current = (*chunks)[chunkIndex];
					if (current.loadGeneration == info.loadGeneration && current.queued && !current.resident) {
						if (success && running) {
							// Completion order can differ from request order. Publish by
							// the latest view priorities without waiting for slower pages.
							auto position = std::find_if(rawChunkQueue.begin(), rawChunkQueue.end(),
								[&](const RadRawChunk& ready) { return readyBefore(raw, ready); });
							rawChunkQueue.insert(position, std::move(raw));
						}
						else
							current.queued = false;
					}
				}
				cv.notify_all();
			}
		}
	}
};

namespace
{
	void clearQueuedChunkState(std::vector<RadChunkInfo>* chunks)
	{
		if (!chunks)
			return;

		// queued also covers the chunk currently being read, which is absent
		// from both queues. Cancel it when changing files or stopping.
		for (RadChunkInfo& chunk : *chunks) {
			if (!chunk.resident && chunk.queued) {
				chunk.queued = false;
				++chunk.loadGeneration;
			}
		}
	}
}

SparkRadStreamer::SparkRadStreamer()
	: _impl(new Impl())
{
	_impl->parent = this;
}

SparkRadStreamer::~SparkRadStreamer()
{
	stop();
}

void SparkRadStreamer::configure(const std::string& filename, std::streamoff chunkDataBase, std::vector<RadChunkInfo>* chunks,
                                 PrepareChunk prepare, size_t maxReadyChunks, size_t workerCount)
{
	std::lock_guard<std::mutex> lock(_impl->mutex);
	std::lock_guard<std::mutex> stateLock(_stateMutex);
	clearQueuedChunkState(_impl->chunks);
	++_impl->configurationGeneration;
	_impl->filename = filename;
	_impl->chunkDataBase = chunkDataBase;
	_impl->chunks = chunks;
	_impl->prepare = std::move(prepare);
	_impl->maxReadyChunks = std::max(size_t(1), maxReadyChunks);
	_impl->workerCount = std::max(size_t(1), std::min(Impl::maxWorkers, workerCount));
	_impl->pendingChunks.clear();
	_impl->rawChunkQueue.clear();
	_impl->requiredRank.clear();
	_impl->cv.notify_all();
}

void SparkRadStreamer::start()
{
	if (_impl->running)
		return;
	_impl->running = true;
	try {
		// Park unused slots so live configure can change concurrency without
		// joining a callback or resizing storage referenced by running workers.
		for (size_t i = 0; i < Impl::maxWorkers; ++i)
			_impl->threads[i] = std::thread(&Impl::run, _impl.get(), i);
	} catch (...) {
		stop();
		throw;
	}
}

void SparkRadStreamer::stop()
{
	{
		// Use the wait mutex when changing its predicate, so a worker cannot
		// miss shutdown between checking running and entering cv.wait.
		std::lock_guard<std::mutex> lock(_impl->mutex);
		_impl->running = false;
	}
	_impl->cv.notify_all();
	for (std::thread& thread : _impl->threads)
		if (thread.joinable()) thread.join();
	std::lock_guard<std::mutex> lock(_impl->mutex);
	std::lock_guard<std::mutex> stateLock(_stateMutex);
	clearQueuedChunkState(_impl->chunks);
	_impl->pendingChunks.clear();
	_impl->rawChunkQueue.clear();
	_impl->requiredRank.clear();
}

void SparkRadStreamer::notify()
{
	_impl->cv.notify_all();
}

bool SparkRadStreamer::hasPendingWork()
{
	std::lock_guard<std::mutex> lock(_impl->mutex);
	std::lock_guard<std::mutex> stateLock(_stateMutex);
	if (!_impl->running || !_impl->chunks)
		return false;
	const auto& chunks = *_impl->chunks;
	for (uint32_t index : _impl->pendingChunks)
		if (index < chunks.size() && chunks[index].queued && !chunks[index].resident)
			return true;
	return _impl->currentInFlightCount() > 0;
}

int SparkRadStreamer::setRequiredChunks(const std::vector<std::pair<uint32_t, float>>& required)
{
	std::lock_guard<std::mutex> lock(_impl->mutex);
	std::lock_guard<std::mutex> stateLock(_stateMutex);
	if (!_impl->chunks) return 0;
	auto& chunks = *_impl->chunks;
	std::vector<unsigned char> wanted(chunks.size(), 0), ready(chunks.size(), 0);
	std::vector<uint32_t> order;
	order.reserve(std::min(required.size(), chunks.size()));
	for (RadChunkInfo& chunk : chunks) chunk.priority = 0.0f;
	for (const auto& entry : required) {
		const uint32_t index = entry.first;
		if (index >= chunks.size()) continue;
		if (!wanted[index]) {
			wanted[index] = 1;
			order.push_back(index);
		}
		if (std::isfinite(entry.second))
			chunks[index].priority = std::max(chunks[index].priority, entry.second);
	}
	for (size_t i = 0; i < chunks.size(); ++i) {
		RadChunkInfo& chunk = chunks[i];
		if (!wanted[i] && !chunk.resident && chunk.queued) {
			chunk.queued = false;
			++chunk.loadGeneration;
		}
	}
	_impl->rawChunkQueue.erase(std::remove_if(_impl->rawChunkQueue.begin(), _impl->rawChunkQueue.end(),
		[&](const RadRawChunk& raw) {
			if (raw.chunkIndex >= chunks.size()) return true;
			const auto& chunk = chunks[raw.chunkIndex];
			if (!wanted[raw.chunkIndex] || chunk.resident || !chunk.queued ||
			    chunk.loadGeneration != raw.loadGeneration || ready[raw.chunkIndex]) return true;
			ready[raw.chunkIndex] = 1;
			return false;
		}), _impl->rawChunkQueue.end());
	_impl->requiredRank.assign(chunks.size(), std::numeric_limits<size_t>::max());
	for (size_t i = 0; i < order.size(); ++i) _impl->requiredRank[order[i]] = i;
	std::stable_sort(_impl->rawChunkQueue.begin(), _impl->rawChunkQueue.end(),
		[&](const RadRawChunk& left, const RadRawChunk& right) {
			return _impl->readyBefore(left, right);
		});
	for (const auto& flight : _impl->inFlight)
		if (_impl->current(flight) && wanted[flight.chunkIndex])
			ready[flight.chunkIndex] = 1;
	std::vector<unsigned char> wasPending(chunks.size(), 0);
	for (uint32_t index : _impl->pendingChunks)
		if (index < chunks.size()) wasPending[index] = 1;
	_impl->pendingChunks.clear();
	int newlyQueued = 0;
	for (uint32_t index : order) {
		RadChunkInfo& chunk = chunks[index];
		if (chunk.resident || ready[index]) continue;
		if (!chunk.queued || !wasPending[index]) ++newlyQueued;
		chunk.queued = true;
		_impl->pendingChunks.push_back(index);
	}
	_impl->cv.notify_all();
	return newlyQueued;
}

std::mutex& SparkRadStreamer::mutex()
{
	return _impl->mutex;
}

std::mutex& SparkRadStreamer::stateMutex()
{
	return _stateMutex;
}

std::vector<uint32_t>& SparkRadStreamer::pendingChunks()
{
	return _impl->pendingChunks;
}

std::vector<RadRawChunk>& SparkRadStreamer::rawChunkQueue()
{
	return _impl->rawChunkQueue;
}

const std::vector<uint32_t>& SparkRadStreamer::pendingChunks() const
{
	return _impl->pendingChunks;
}

const std::vector<RadRawChunk>& SparkRadStreamer::rawChunkQueue() const
{
	return _impl->rawChunkQueue;
}
