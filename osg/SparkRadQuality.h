#ifndef SPARK_RAD_QUALITY_H
#define SPARK_RAD_QUALITY_H

#include <osg/Matrix>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <string>

// Preserve detail while moving by default. An explicit smaller moving budget
// can trade interaction quality for less selection/sorting work; allocation
// still includes the full idle target.
struct SparkRadQualitySettings {
    int movingSplats = 2500000;
    int idleSplats = 2500000;
    int idleDelayMs = 350;

    int capacity() const { return std::max(movingSplats, idleSplats); }

    static std::string environment(const char* name)
    {
#ifdef _MSC_VER
        char* value = nullptr;
        size_t length = 0;
        if (_dupenv_s(&value, &length, name) != 0 || !value)
            return std::string();
        const std::string result(value);
        std::free(value);
        return result;
#else
        const char* value = std::getenv(name);
        return value ? std::string(value) : std::string();
#endif
    }

    static int boundedInteger(const std::string& text, int fallback, int low, int high)
    {
        if (text.empty()) return fallback;
        char* end = nullptr;
        const long value = std::strtol(text.c_str(), &end, 10);
        if (end == text.c_str() || *end != '\0') return fallback;
        return static_cast<int>(std::max<long>(low, std::min<long>(high, value)));
    }

    static SparkRadQualitySettings fromEnvironment()
    {
        SparkRadQualitySettings settings;
        const std::string fixed = environment("OSG_3DGS_RAD_MAX_ACTIVE");
        if (!fixed.empty()) {
            // An explicit budget is a fixed budget, including for benchmarks.
            settings.movingSplats = boundedInteger(fixed, settings.movingSplats, 10000, 4000000);
            settings.idleSplats = settings.movingSplats;
        } else {
            settings.idleSplats = boundedInteger(environment("OSG_3DGS_RAD_IDLE_MAX_ACTIVE"),
                settings.idleSplats, 600000, 4000000);
            settings.movingSplats = boundedInteger(environment("OSG_3DGS_RAD_MOVING_MAX_ACTIVE"),
                std::min(settings.movingSplats, settings.idleSplats), 10000, settings.idleSplats);
        }
        settings.idleDelayMs = boundedInteger(environment("OSG_3DGS_RAD_IDLE_DELAY_MS"),
            settings.idleDelayMs, 0, 5000);
        return settings;
    }
};

// Time is supplied by the caller so a stationary camera, resumed movement and
// stale asynchronous selections can be tested without sleeps or a GL context.
class SparkRadQualityPolicy {
public:
    void reset(const SparkRadQualitySettings& settings)
    {
        _settings = settings;
        _initialized = false;
        _refiningIdle = false;
        _budget = settings.movingSplats;
        _generation = 0;
    }

    int update(const osg::Matrix& view, double nowMs)
    {
        bool moving = !_initialized;
        for (int row = 0; row < 4 && !moving; ++row)
            for (int column = 0; column < 4; ++column)
                if (std::abs(view(row, column) - _motionView(row, column)) > 1.0e-5) {
                    moving = true;
                    break;
                }
        if (moving) {
            _motionView = view;
            _lastMotionMs = nowMs;
            _initialized = true;
        }
        _refiningIdle = !moving && nowMs - _lastMotionMs >= _settings.idleDelayMs;
        const int budget = _refiningIdle
            ? _settings.idleSplats : _settings.movingSplats;
        if (budget != _budget) {
            _budget = budget;
            ++_generation;
        }
        return _budget;
    }

    int budget() const { return _budget; }
    // Page batching follows camera inactivity even when an explicit fixed
    // budget makes moving/idle selections the same size.
    bool refiningIdle() const { return _refiningIdle; }
    std::uint64_t generation() const { return _generation; }
    bool accepts(std::uint64_t submittedGeneration) const
    {
        // With different moving/idle budgets, the generation also rejects
        // idle -> moving -> idle results whose numeric budget happens to match
        // again when the worker completes. Equal budgets need no invalidation.
        return submittedGeneration == _generation;
    }

private:
    SparkRadQualitySettings _settings;
    osg::Matrix _motionView;
    double _lastMotionMs = 0.0;
    bool _initialized = false;
    bool _refiningIdle = false;
    int _budget = 2500000;
    std::uint64_t _generation = 0;
};

// A moving camera may consume useful older LOD results. Once a pose has stayed
// still for 80 ms, however, one obsolete task can be canceled to start selecting
// that stopped view sooner. This is independent of the quality/projection
// generations, and never permits borrowed worker inputs to be released early.
class SparkRadLodRetargetPolicy {
public:
    void reset()
    {
        _initialized = false;
        _settled = false;
        _canceledForStop = false;
    }

    void update(const osg::Matrix& currentView, double nowMs)
    {
        _currentView = currentView;
        const bool moving = !_initialized || !sameView(currentView, _motionView);
        if (moving) {
            _motionView = currentView;
            _lastMotionMs = nowMs;
            _initialized = true;
            _canceledForStop = false;
        }
        _settled = !moving && std::isfinite(nowMs) && std::isfinite(_lastMotionMs) &&
            nowMs - _lastMotionMs >= 80.0;
    }

    // Owner-thread call after update(). Returning true consumes this stop's
    // cancellation opportunity; idle polling cannot repeatedly cancel work.
    // A matching or absent task leaves it available for a later stale task.
    bool shouldCancel(bool workerInFlight, const osg::Matrix& submittedView)
    {
        if (!_initialized || !_settled || _canceledForStop || !workerInFlight ||
            sameView(submittedView, _currentView))
            return false;
        _canceledForStop = true;
        return true;
    }

private:
    static bool sameView(const osg::Matrix& a, const osg::Matrix& b)
    {
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column) {
                const double delta = a(row, column) - b(row, column);
                if (!std::isfinite(delta) || std::abs(delta) > 1.0e-5)
                    return false;
            }
        return true;
    }

    osg::Matrix _currentView;
    osg::Matrix _motionView;
    double _lastMotionMs = 0.0;
    bool _initialized = false;
    bool _settled = false;
    bool _canceledForStop = false;
};

// A partially uploaded batch may be blocked only by the old sorted display.
// Waiting for the already requested replacement frees those pages sooner than
// starting another LOD traversal, which would freeze all page application.
struct SparkRadSortHandoffWait {
    struct State {
        bool capacityBlocked = false;
        int loadedSinceSelection = 0;
        bool gpuSort = false;
        bool sortDisabled = false;
        bool sortPending = false;
        unsigned int activeVersion = 0;
        unsigned int displayVersion = 0;
        bool qualityChanged = false;
        bool projectionChanged = false;
    };

    static bool shouldWait(const osg::Matrix& currentView, const osg::Matrix& selectedView,
                           const State& state)
    {
        if (!state.capacityBlocked || state.loadedSinceSelection <= 0 ||
            state.gpuSort || state.sortDisabled || !state.sortPending ||
            state.activeVersion == state.displayVersion || state.qualityChanged || state.projectionChanged)
            return false;
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column) {
                const double delta = currentView(row, column) - selectedView(row, column);
                if (!std::isfinite(delta) || std::abs(delta) > 1.0e-5)
                    return false;
            }
        return true;
    }
};

// Projection changes invalidate selections independently of camera motion.
// The generation rejects A -> B -> A changes while a task still uses old A;
// the pose is deliberately excluded so moving cameras can consume useful work.
class SparkRadProjectionState {
public:
    struct Stamp {
        osg::Matrix matrix;
        double x = 0.0, y = 0.0, width = 0.0, height = 0.0;
        bool valid = false;
    };

    void reset() { _initialized = false; _generation = 0; }
    bool update(const Stamp& stamp)
    {
        bool changed = !_initialized || stamp.valid != _stamp.valid ||
            stamp.x != _stamp.x || stamp.y != _stamp.y ||
            stamp.width != _stamp.width || stamp.height != _stamp.height;
        for (int row = 0; row < 4 && !changed; ++row)
            for (int column = 0; column < 4; ++column)
                if (stamp.matrix(row, column) != _stamp.matrix(row, column)) {
                    changed = true;
                    break;
                }
        if (changed) {
            _stamp = stamp;
            _initialized = true;
            ++_generation;
        }
        return changed;
    }
    std::uint64_t generation() const { return _generation; }
    bool accepts(std::uint64_t generation) const { return generation == _generation; }

private:
    Stamp _stamp;
    bool _initialized = false;
    std::uint64_t _generation = 0;
};

// A full pool whose pages are all protected cannot progress just because more
// compressed bytes arrived. Retry only after something that can change page
// protection changes; never drop the currently displayed complete frontier.
class SparkRadStreamWait {
public:
    struct State {
        std::uint64_t quality = 0;
        unsigned int active = 0;
        std::uint64_t display = 0;
        std::uint64_t residency = 0;
    };

    void clear() { _blocked = false; }
    void park(const osg::Matrix& view, const State& state)
    {
        _view = view;
        _state = state;
        _blocked = true;
    }
    bool blockedFor(const osg::Matrix& view, const State& state) const
    {
        if (!_blocked || state.quality != _state.quality || state.active != _state.active ||
            state.display != _state.display || state.residency != _state.residency)
            return false;
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column)
                if (std::abs(view(row, column) - _view(row, column)) > 1.0e-5)
                    return false;
        return true;
    }

private:
    bool _blocked = false;
    osg::Matrix _view;
    State _state;
};

#endif
