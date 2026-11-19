#include "SparkLodWorker.h"

#include <chrono>
#include <utility>

SparkLodWorker::~SparkLodWorker()
{
    stop();
}

bool SparkLodWorker::submit(const SparkLodSelector::Input& input, SparkLodSelector::TraverseMode mode)
{
    std::lock_guard<std::mutex> lock(_mutex);
    if (_inFlight || _stopping)
        return false;

    if (input.chunks)
        _chunks = *input.chunks;
    else
        _chunks.clear();
    _input = input;
    _input.chunks = input.chunks ? &_chunks : nullptr;
    _cancelRequested.store(false, std::memory_order_relaxed);
    _input.cancelRequested = &_cancelRequested;
    _mode = mode;
    _selectedView = input.viewMatrix;
    _complete = false;
    _error = nullptr;
    _hasTask = true;
    _inFlight = true;
    if (!_thread.joinable()) {
        try {
            _thread = std::thread(&SparkLodWorker::run, this);
        } catch (...) {
            _hasTask = false;
            _inFlight = false;
            _input = SparkLodSelector::Input{};
            throw;
        }
    }
    _condition.notify_one();
    return true;
}

void SparkLodWorker::requestCancel()
{
    if (_inFlight.load())
        _cancelRequested.store(true, std::memory_order_relaxed);
}

bool SparkLodWorker::tryTake(SparkLodSelector::Result& result, osg::Matrix& selectedView, double& elapsedMs)
{
    std::unique_lock<std::mutex> lock(_mutex, std::try_to_lock);
    if (!lock.owns_lock() || !_complete)
        return false;

    if (_cancelRequested.load(std::memory_order_relaxed)) {
        // Also discard a completed result invalidated before owner handoff.
        _result = SparkLodSelector::Result{};
        _result.candidateComplete = false;
        _result.canceled = true;
    }
    result = std::move(_result);
    selectedView = _selectedView;
    elapsedMs = _elapsedMs;
    auto error = _error;
    _error = nullptr;
    _complete = false;
    _input = SparkLodSelector::Input{};
    _inFlight = false;
    lock.unlock();
    if (error)
        std::rethrow_exception(error);
    return true;
}

void SparkLodWorker::stop()
{
    {
        std::lock_guard<std::mutex> lock(_mutex);
        _stopping = true;
        _cancelRequested.store(true, std::memory_order_relaxed);
    }
    _condition.notify_one();
    if (_thread.joinable())
        _thread.join();

    std::lock_guard<std::mutex> lock(_mutex);
    _input = SparkLodSelector::Input{};
    _result = SparkLodSelector::Result{};
    _chunks.clear();
    _error = nullptr;
    _hasTask = false;
    _complete = false;
    _stopping = false;
    _inFlight = false;
}

void SparkLodWorker::run()
{
    SparkLodSelector selector;
    std::unique_lock<std::mutex> lock(_mutex);
    for (;;) {
        _condition.wait(lock, [this] { return _hasTask || _stopping; });
        if (_stopping)
            return;

        const auto input = _input;
        const auto mode = _mode;
        _hasTask = false;
        lock.unlock();
        const auto start = std::chrono::steady_clock::now();
        SparkLodSelector::Result result;
        std::exception_ptr error;
        try {
            result = selector.select(input, mode);
        } catch (...) {
            error = std::current_exception();
        }
        const double elapsedMs = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();

        lock.lock();
        if (_stopping)
            return;
        _result = std::move(result);
        _elapsedMs = elapsedMs;
        _error = error;
        _complete = true;
        // Keep _inFlight set until the owner consumes this result. In
        // particular, completion alone does not permit page mutation.
    }
}
