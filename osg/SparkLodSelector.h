#ifndef SPARK_LOD_SELECTOR_H
#define SPARK_LOD_SELECTOR_H

#include "osg/Matrix"
#include "osg/Vec3f"

#include <atomic>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

struct MI_GaussianPoint;
struct RadChunkInfo;

class SparkLodSelector
{
public:
	// The LOD scorer needs only this compact immutable subset of a Gaussian.
	// RAD keeps it alongside the full render record so traversal does not pull
	// SH and rotation data through the CPU cache for every visited node.
	struct LodNode
	{
		osg::Vec3f position;
		osg::Vec3f scale;
		float alpha = 0.0f;
		float size = 0.0f;
	};
	static_assert(sizeof(LodNode) == 32, "Compact RAD LOD records must remain 32 bytes");

	struct Options
	{
		int maxActiveSplats = 0;
		int totalPointCount = 0;
		int chunkSize = 65536;
		int totalChunkCount = 0;
		// Standard traversal counts missing dependencies as well as resident
		// pages. Zero leaves the historical unlimited residency behavior.
		int maxResidentChunks = 0;
		float pixelScaleLimit = 0.0f;
	};

	struct Input
	{
		const std::vector<MI_GaussianPoint>* points = nullptr;
		const std::vector<LodNode>* lodNodes = nullptr;
		bool useCompactLodRecords = true;
		const std::vector<int>* globalToLocal = nullptr;
		const std::vector<int>* slotToChunk = nullptr;
		const std::vector<RadChunkInfo>* chunks = nullptr;
		const std::vector<uint16_t>* childCounts = nullptr;
		const std::vector<uint32_t>* childStarts = nullptr;
		const std::vector<float>* lodSizes = nullptr;
		// Nonzero certifies contiguous resident mappings at pageIndex * pageSize.
		// The producer freezes these mappings for the entire selection. Zero
		// retains the general globalToLocal lookup for legacy/scattered storage.
		int pagedPageSize = 0;
		osg::Matrix viewMatrix;
		Options options;
		// Foveated LOD parameters (Spark-style)
		float coneFov0 = 0.0f;       // inner cone full-width angle (degrees), 0=disabled
		float coneFov = 0.0f;        // outer cone full-width angle (degrees), 0=disabled
		float coneFoveate = 1.0f;    // foveation scale at outer cone edge (0-1)
		float behindFoveate = 1.0f;  // foveation scale behind camera (0-1)
		// Symmetric perspective viewport. Invalid/nonpositive tangents disable
		// viewport weighting; a weight of 1 preserves historical priorities.
		float tanHalfFovX = 0.0f;
		float tanHalfFovY = 0.0f;
		float viewportOutsideWeight = 1.0f;
		bool collectViewportStats = false;
		// Standard traversal can collect the exact final active page set while
		// appending indices, avoiding a second full frontier scan on the owner.
		bool collectActiveChunks = false;
		// Optional cooperative cancellation. Keep this token alive and latched
		// once requested; do not reuse input storage until select() returns.
		const std::atomic<bool>* cancelRequested = nullptr;
		// Opt in only for immutable resident pages: changing page contents must
		// change its loadGeneration, and dataset/reset changes dataEpoch. The
		// selector retains cached priorities between calls on one owner thread.
		bool cachePagedPriorities = false;
		// The same immutable-page contract permits lazily keeping 20-byte scoring
		// inputs plus one validity byte per visited page slot. Camera changes reuse
		// these inputs while recomputing priorities, avoiding larger SH records.
		bool cachePagedScoringInputs = false;
		// Standard traversal only. Equal priorities have no stable ordering in
		// either heap; this can choose different tied nodes at a budget boundary.
		bool useFourAryHeap = false;
		// Standard traversal only. Already covered terminal priorities can go
		// straight to the active frontier without any further heap operations.
		bool bypassTerminalHeap = false;
		// Reuse canonical child-range bounds and page priorities for terminal
		// bypass. This only removes repeated lookups; traversal order is unchanged.
		bool useCanonicalTerminalRanges = false;
		// Standard traversal only; the immutable-page contract above also
		// applies to child metadata and mappings. Reuse a sufficiently long
		// prefix before its first missing dependency when its pages and every
		// selection parameter are unchanged. Unrelated pages may be replaced.
		bool resumePagedTraversal = false;
		std::uint64_t dataEpoch = 0;
	};

	struct ChunkPriority
	{
		uint32_t chunkIndex = 0;
		float priority = 0.0f;
	};

	struct Result
	{
		std::vector<int> activeIndices;
		// Unique logical chunks in active-index first-occurrence order. Valid
		// only for complete standard results when collection was requested.
		std::vector<int> activeChunks;
		bool activeChunksValid = false;
		std::vector<ChunkPriority> touchedChunks;
		std::vector<ChunkPriority> missingChunks;
		// Standard traversal's complete admitted dependency set, including
		// ancestors and missing children, ordered by current-view priority.
		std::vector<ChunkPriority> requiredChunks;
		bool candidateComplete = true;
		// Canceled traversals never expose a partial frontier or page requests.
		bool canceled = false;
		int stopActiveCap = 0;
		int stopMissingChildren = 0;
		int stopPixelLimit = 0;
		int stopResidentCap = 0;
		float maxFrontierPixelScale = 0.0f;
		int missingCount = 0;
		// Optional center classification of the final active frontier; these
		// counts do not cull splats or alter the active/residency budgets.
		int viewportInsideCount = 0;
		int viewportOutsideCount = 0;
		int viewportBehindCount = 0;
		std::uint64_t priorityCacheHits = 0;
		std::uint64_t priorityCacheMisses = 0;
		std::uint64_t scoringCachePointsBuilt = 0;
		// Allocated compact-input/validity capacities, excluding priority cache.
		std::uint64_t scoringCacheBytes = 0;
		std::uint64_t bypassedTerminalCount = 0;
		// Per-call work saved by certified terminal ranges; restored checkpoint
		// work is excluded. Leaves avoid two dependent required-priority lookups.
		std::uint64_t canonicalTerminalRangeNodes = 0;
		std::uint64_t canonicalTerminalRangeLeaves = 0;
		// Work items are popped heap entries, not scored children. The prefix
		// counter is observed even when too short or ineligible to be saved.
		std::uint64_t checkpointPrefixItems = 0;
		std::uint64_t checkpointSavedItems = 0;
		std::uint64_t checkpointResumedItems = 0;
		std::uint64_t checkpointBytes = 0;
	};

	enum class TraverseMode
	{
		Standard,
		Dynamic
	};

	Result select(const Input& input, TraverseMode mode = TraverseMode::Standard) const;

private:
	struct PriorityCachePage;
	struct ScoringInput;
	struct RadItem {
		float pixelScale = 0.0f;
		int localIndex = -1;
		bool operator<(const RadItem& rhs) const { return pixelScale < rhs.pixelScale; }
	};
	struct PixelScaleParams {
		bool foveated = false;
		float coneDot0 = 1.0f;
		float coneDot = 1.0f;
		bool viewportValid = false;
		bool viewportWeighted = false;
		float viewportInvNormalX = 1.0f;
		float viewportInvNormalY = 1.0f;
	};
	Result selectStandard(const Input& input, const PixelScaleParams& params) const;
	Result selectDynamic(const Input& input, const PixelScaleParams& params) const;
	bool isResidentLocal(const Input& input, int localIndex) const;
	float computePixelScale(const Input& input, int localIndex, const PixelScaleParams& params) const;
	float computeCachedPixelScale(const Input& input, int localIndex, const PixelScaleParams& params,
		PriorityCachePage& page, int offset) const;
	float computeUncachedPixelScale(const Input& input, int localIndex, const PixelScaleParams& params) const;
	float computePixelScaleValues(const Input& input, const osg::Vec3f& position,
		const osg::Vec3f& scale, float alpha, float size, const PixelScaleParams& params) const;
	float computeCompactPixelScale(const Input& input, const ScoringInput& point,
		const PixelScaleParams& params) const;
	void collectViewportStats(const Input& input, const PixelScaleParams& params, Result& result) const;
	void preparePriorityCache(const Input& input) const;
	static bool sameTraversalInput(const Input& a, const Input& b);
	struct ScoringInput {
		osg::Vec3f position;
		float size = 0.0f;
		// NaN means the original static support checks disabled weighting.
		float viewportRadius = 0.0f;
	};
	struct PriorityCachePage {
		int localBase = -1;
		int count = 0;
		std::uint32_t generation = 0;
		std::vector<float> priorities;
		// Separate validity preserves every float value, including NaN.
		std::vector<unsigned char> valid;
		std::vector<ScoringInput> scoringInputs;
		// Independent of view-dependent priority validity; publish only after
		// all immutable inputs for this slot have been written.
		std::vector<unsigned char> scoringValid;
	};
	struct CheckpointPage {
		std::uint32_t chunkIndex = 0;
		int base = 0;
		int count = 0;
		int pageIndex = -1;
		std::uint32_t generation = 0;
	};
	struct TraversalCheckpoint {
		Input input;
		size_t chunkCount = 0;
		int rootLocal = -1;
		std::uint64_t processedItems = 0;
		std::uint64_t bytes = 0;
		std::vector<RadItem> frontier;
		Result prefix;
		std::vector<int> touchedPositions;
		std::vector<int> requiredPositions;
		std::vector<CheckpointPage> dependencies;
	};
	mutable std::unique_ptr<TraversalCheckpoint> _traversalCheckpoint;
	mutable bool _priorityCacheActive = false;
	mutable bool _scoringCacheActive = false;
	mutable Input _priorityCacheInput;
	mutable std::vector<PriorityCachePage> _priorityCachePages;
	mutable std::uint64_t _priorityCacheHits = 0;
	mutable std::uint64_t _priorityCacheMisses = 0;
	mutable std::uint64_t _scoringCachePointsBuilt = 0;
};

#endif
