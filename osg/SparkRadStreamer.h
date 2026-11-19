#ifndef SPARK_RAD_STREAMER_H
#define SPARK_RAD_STREAMER_H

#include <cstdint>
#include <cstddef>
#include <condition_variable>
#include <functional>
#include <iosfwd>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

struct RadChunkInfo;
struct RadRawChunk;

class SparkRadStreamer
{
public:
	SparkRadStreamer();
	~SparkRadStreamer();

	SparkRadStreamer(const SparkRadStreamer&) = delete;
	SparkRadStreamer& operator=(const SparkRadStreamer&) = delete;

	// Runs on streaming workers without either streamer lock. The callback
	// owns only its raw payload and the metadata snapshot; it must not mutate
	// chunkIndex/loadGeneration or access mutable renderer/page state. Multiple
	// workers may call it concurrently, so captured mutable state needs its own
	// synchronization. Worker count is clamped to 1..3 and may change live.
	using PrepareChunk = std::function<bool(RadRawChunk&, const RadChunkInfo&)>;
	// maxReadyChunks bounds ready payloads plus reserved slots for current valid
	// work. With one worker this preserves the original queue admission rule.
	// Canceled work retains its physical worker slot until it returns: across
	// reconfiguration, at most three stale payloads can exist in addition to
	// this bound, plus callback decode temporaries and the caller's applied page.
	void configure(const std::string& filename, std::streamoff chunkDataBase, std::vector<RadChunkInfo>* chunks,
	               PrepareChunk prepare = {}, size_t maxReadyChunks = 8, size_t workerCount = 1);
	void start();
	void stop();
	void notify();
	// Pending reads/preparation only, excluding already-ready results. Caller
	// must hold neither mutex. Canceled generations/configurations do not count.
	bool hasPendingWork();

	// Replace fetch priorities with the current traversal's ordered page set.
	// Includes resident pages; missing pages are queued once, while obsolete
	// pending, compressed and in-flight reads are canceled by generation.
	// Acquires queue/state mutexes internally; caller must hold neither.
	int setRequiredChunks(const std::vector<std::pair<uint32_t, float>>& required);

	std::mutex& mutex();
	std::mutex& stateMutex(); // Expose the internal state mutex for thread-safe access
	std::vector<uint32_t>& pendingChunks();
	std::vector<RadRawChunk>& rawChunkQueue();
	const std::vector<uint32_t>& pendingChunks() const;
	const std::vector<RadRawChunk>& rawChunkQueue() const;

private:
	void run();

	struct Impl;
	std::unique_ptr<Impl> _impl;
	std::mutex _stateMutex;
};

#endif
