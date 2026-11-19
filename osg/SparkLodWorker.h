#ifndef SPARK_LOD_WORKER_H
#define SPARK_LOD_WORKER_H

#include "RadTypes.h"
#include "SparkLodSelector.h"

#include <atomic>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

// Single-flight LOD traversal. Call submit/tryTake/stop from one owner thread.
// The worker copies chunk metadata during submit; callers must synchronize that
// copy with the streamer's chunk-state mutex. All other Input vectors are
// borrowed: keep them alive and do not modify/resize them from successful submit
// until tryTake succeeds (or throws), or stop returns. In particular, do not
// apply/evict streamed pages during this interval. File I/O can continue into
// separate raw-byte queues while traversal runs.
class SparkLodWorker {
public:
    SparkLodWorker() = default;
    ~SparkLodWorker();
    SparkLodWorker(const SparkLodWorker&) = delete;
    SparkLodWorker& operator=(const SparkLodWorker&) = delete;

    // The worker owns its cancellation token; use requestCancel() rather than
    // Input::cancelRequested for asynchronous work.
    bool submit(const SparkLodSelector::Input& input,
                SparkLodSelector::TraverseMode mode = SparkLodSelector::TraverseMode::Standard);
    bool hasInFlight() const { return _inFlight.load(); }

    // Nonblocking owner-thread invalidation. Borrowed inputs stay protected
    // until tryTake consumes a canceled result, or stop joins the worker.
    void requestCancel();

    // Never waits for traversal or a held mutex. A completed traversal exception
    // is propagated here after releasing the borrowed inputs.
    bool tryTake(SparkLodSelector::Result& result, osg::Matrix& selectedView, double& elapsedMs);

    // Waits for any traversal to finish, then discards its result and releases
    // borrowed inputs. A later submit lazily starts a new persistent thread.
    void stop();

private:
    void run();

    std::mutex _mutex;
    std::condition_variable _condition;
    std::thread _thread;
    std::atomic<bool> _inFlight{false};
    std::atomic<bool> _cancelRequested{false};
    bool _stopping = false;
    bool _hasTask = false;
    bool _complete = false;
    SparkLodSelector::Input _input;
    SparkLodSelector::TraverseMode _mode = SparkLodSelector::TraverseMode::Standard;
    std::vector<RadChunkInfo> _chunks;
    SparkLodSelector::Result _result;
    osg::Matrix _selectedView;
    double _elapsedMs = 0.0;
    std::exception_ptr _error;
};

#endif
