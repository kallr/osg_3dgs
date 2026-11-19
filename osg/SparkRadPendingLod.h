#ifndef SPARK_RAD_PENDING_LOD_H
#define SPARK_RAD_PENDING_LOD_H

#include "SparkLodSelector.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>

// A small complete LOD change waits for actual rendering frames, retaining
// the finished traversal instead of rerunning a large traversal every time.
class SparkRadPendingLod {
public:
    struct State {
        std::uint64_t quality = 0;
        std::uint64_t projection = 0;
        std::uint64_t residency = 0;
    };
    struct Candidate {
        SparkLodSelector::Result result;
        osg::Matrix view;
        float pixelScaleLimit = 0.0f;
        float viewportOutsideWeight = 1.0f;
    };
    enum class Status { None, Waiting, Ready, Invalidated };

    bool defer(Candidate&& candidate, const osg::Matrix& currentView,
               const State& state, unsigned int frame, int stableFrames)
    {
        clear();
        // Pose-only worker results can be useful while moving, but a small
        // deferred change must still describe the camera that is stabilizing.
        if (!sameView(candidate.view, currentView))
            return false;
        _candidate = std::move(candidate);
        _state = state;
        _firstFrame = frame;
        _stableFrames = unsigned(std::max(1, stableFrames));
        _pending = true;
        return true;
    }

    Status poll(const osg::Matrix& currentView, const State& state,
                unsigned int frame, Candidate& out)
    {
        if (!_pending)
            return Status::None;
        if (state.quality != _state.quality || state.projection != _state.projection ||
            state.residency != _state.residency || !sameView(_candidate.view, currentView)) {
            clear();
            return Status::Invalidated;
        }
        // The frame that completed the traversal is the first stable frame.
        // Unsigned subtraction also handles the frame counter wrapping.
        if (frame - _firstFrame < _stableFrames - 1)
            return Status::Waiting;
        out = std::move(_candidate);
        _pending = false;
        return Status::Ready;
    }

    bool pending() const { return _pending; }
    void clear()
    {
        _pending = false;
        _candidate = Candidate();
    }

private:
    static bool sameView(const osg::Matrix& a, const osg::Matrix& b)
    {
        for (int row = 0; row < 4; ++row)
            for (int column = 0; column < 4; ++column) {
                const double delta = a(row, column) - b(row, column);
                if (!std::isfinite(delta) || std::abs(delta) > 1.0e-3)
                    return false;
            }
        return true;
    }

    Candidate _candidate;
    State _state;
    unsigned int _firstFrame = 0;
    unsigned int _stableFrames = 1;
    bool _pending = false;
};

#endif
