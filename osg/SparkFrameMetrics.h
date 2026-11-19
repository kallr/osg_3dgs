#ifndef SPARK_FRAME_METRICS_H
#define SPARK_FRAME_METRICS_H

#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>

// Opt-in CPU timings. These measure work submitted by the render thread, not
// completed GPU execution; frame intervals also include Qt scheduling/present.
namespace SparkFrameMetrics {
enum Stage { Frame, Protect, Stream, Select, Residency, SortRequest, SortApply, LodWorker, LodCommit, DrawStage,
             Decode, Convert, Pack, PageApply, PrepareWorker,
             PageAllocate, PageImages, PageCpu, PageSortPositions, Count };
using Clock = std::chrono::steady_clock;

inline bool enabled()
{
    static const bool value = [] {
        const char* setting = std::getenv("OSG_3DGS_PROFILE");
        return setting && setting[0] && !(setting[0] == '0' && setting[1] == '\0');
    }();
    return value;
}

struct FrameState {
    std::array<double, Count> milliseconds{};
    Clock::time_point previousStart{};
    unsigned long long number = 0;
    double intervalMilliseconds = 0.0;
    bool inFrame = false;
};

inline FrameState& state()
{
    static thread_local FrameState current;
    return current;
}

inline void beginFrame()
{
    if (!enabled()) return;
    auto& current = state();
    auto now = Clock::now();
    current.intervalMilliseconds = current.number
        ? std::chrono::duration<double, std::milli>(now - current.previousStart).count() : 0.0;
    current.previousStart = now;
    ++current.number;
    current.milliseconds.fill(0.0);
    current.inFrame = true;
}

class Scope {
public:
    explicit Scope(Stage stage) : _stage(stage), _active(enabled() && state().inFrame)
    {
        if (_active) _start = Clock::now();
    }
    ~Scope()
    {
        if (_active) {
            state().milliseconds[_stage] +=
                std::chrono::duration<double, std::milli>(Clock::now() - _start).count();
        }
    }
    Scope(const Scope&) = delete;
    Scope& operator=(const Scope&) = delete;
private:
    Stage _stage;
    bool _active;
    Clock::time_point _start;
};

inline void endFrame(double elapsedSeconds, bool moving)
{
    if (!enabled()) return;
    auto& current = state();
    current.inFrame = false;
    const auto& ms = current.milliseconds;
    // Buffered output avoids a console/file flush on every frame.
    std::printf("FRAME_PROFILE,frame=%llu,elapsed=%.6f,intervalMs=%.3f,moving=%d,"
                "frameMs=%.3f,protectMs=%.3f,streamMs=%.3f,selectMs=%.3f,"
                "residencyMs=%.3f,sortRequestMs=%.3f,sortApplyMs=%.3f,lodWorkerMs=%.3f,"
                "lodCommitMs=%.3f,drawStageMs=%.3f,decodeMs=%.3f,convertMs=%.3f,packMs=%.3f,"
                "pageApplyMs=%.3f,prepareWorkerMs=%.3f,pageAllocateMs=%.3f,pageImagesMs=%.3f,"
                "pageCpuMs=%.3f,pageSortPositionsMs=%.3f\n",
                current.number, elapsedSeconds, current.intervalMilliseconds, moving ? 1 : 0,
                ms[Frame], ms[Protect], ms[Stream], ms[Select], ms[Residency], ms[SortRequest], ms[SortApply], ms[LodWorker],
                ms[LodCommit], ms[DrawStage], ms[Decode], ms[Convert], ms[Pack], ms[PageApply], ms[PrepareWorker],
                ms[PageAllocate], ms[PageImages], ms[PageCpu], ms[PageSortPositions]);
}
} // namespace SparkFrameMetrics

#endif
