#include "SparkLodSelector.h"

#include "GaussianDrawObj.h"

#include "osg/Vec4f"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

namespace {
SparkLodSelector::Result canceledSelection()
{
	SparkLodSelector::Result result;
	result.candidateComplete = false;
	result.canceled = true;
	return result;
}

bool cancellationRequested(const SparkLodSelector::Input& input)
{
	return input.cancelRequested && input.cancelRequested->load(std::memory_order_relaxed);
}

// Checking once per small group bounds wasted traversal without an atomic
// load for every node. The first check also handles pre-canceled tasks.
class CancellationCheck {
public:
	explicit CancellationCheck(const SparkLodSelector::Input& input) : _input(input) {}
	bool operator()() { return _input.cancelRequested && ((_count++ & 1023u) == 0) && cancellationRequested(_input); }
private:
	const SparkLodSelector::Input& _input;
	unsigned int _count = 0;
};

// Four children halve the height of the large traversal frontier. Keep the
// binary implementation available as the default oracle and for runtime A/B.
// Both compare priority only; ties intentionally retain unspecified ordering.
template<typename T>
void pushFourAryHeap(std::vector<T>& heap)
{
	size_t hole = heap.size() - 1;
	const T value = heap[hole];
	while (hole > 0) {
		const size_t parent = (hole - 1) / 4;
		if (!(heap[parent] < value))
			break;
		heap[hole] = heap[parent];
		hole = parent;
	}
	heap[hole] = value;
}

template<typename T>
void popFourAryHeap(std::vector<T>& heap)
{
	const T value = heap.back();
	heap.pop_back();
	if (heap.empty())
		return;
	const size_t count = heap.size();
	size_t hole = 0;
	for (size_t first = 1; first < count; first = 4 * hole + 1) {
		size_t largest = first;
		const size_t end = std::min(first + 4, count);
		for (size_t child = first + 1; child < end; ++child)
			if (heap[largest] < heap[child])
				largest = child;
		heap[hole] = heap[largest];
		hole = largest;
	}
	while (hole > 0) {
		const size_t parent = (hole - 1) / 4;
		if (!(heap[parent] < value))
			break;
		heap[hole] = heap[parent];
		hole = parent;
	}
	heap[hole] = value;
}

template<typename T>
bool copyCheckpointVector(const std::vector<T>& source, std::vector<T>& destination,
	const SparkLodSelector::Input& input)
{
	destination.clear();
	destination.reserve(source.size());
	constexpr size_t batch = 16384;
	for (size_t begin = 0; begin < source.size(); begin += batch) {
		if (cancellationRequested(input))
			return false;
		const size_t end = std::min(begin + batch, source.size());
		destination.insert(destination.end(), source.begin() + begin, source.begin() + end);
	}
	return !cancellationRequested(input);
}

// Copy the large active list in bounded batches; the page lists and scalar
// diagnostics are small. A partially copied snapshot is never published.
bool copyCheckpointResult(const SparkLodSelector::Result& source, SparkLodSelector::Result& destination,
	const SparkLodSelector::Input& input)
{
	if (!copyCheckpointVector(source.activeIndices, destination.activeIndices, input))
		return false;
	// Avoid copying activeIndices a second time through Result assignment.
	destination.activeChunks = source.activeChunks;
	destination.activeChunksValid = false; // Prefix summaries are internal until traversal completes.
	destination.touchedChunks = source.touchedChunks;
	destination.missingChunks = source.missingChunks;
	destination.requiredChunks = source.requiredChunks;
	destination.candidateComplete = source.candidateComplete;
	destination.canceled = source.canceled;
	destination.stopActiveCap = source.stopActiveCap;
	destination.stopMissingChildren = source.stopMissingChildren;
	destination.stopPixelLimit = source.stopPixelLimit;
	destination.stopResidentCap = source.stopResidentCap;
	destination.maxFrontierPixelScale = source.maxFrontierPixelScale;
	destination.missingCount = source.missingCount;
	destination.viewportInsideCount = source.viewportInsideCount;
	destination.viewportOutsideCount = source.viewportOutsideCount;
	destination.viewportBehindCount = source.viewportBehindCount;
	destination.bypassedTerminalCount = source.bypassedTerminalCount;
	return !cancellationRequested(input);
}
}

bool SparkLodSelector::sameTraversalInput(const Input& a, const Input& b)
{
	return a.resumePagedTraversal && b.resumePagedTraversal && a.dataEpoch == b.dataEpoch &&
		a.points == b.points && a.lodNodes == b.lodNodes && a.useCompactLodRecords == b.useCompactLodRecords &&
		a.globalToLocal == b.globalToLocal && a.slotToChunk == b.slotToChunk &&
		a.chunks == b.chunks && a.childCounts == b.childCounts && a.childStarts == b.childStarts &&
		a.lodSizes == b.lodSizes && a.pagedPageSize == b.pagedPageSize && a.viewMatrix == b.viewMatrix &&
		a.options.maxActiveSplats == b.options.maxActiveSplats && a.options.totalPointCount == b.options.totalPointCount &&
		a.options.chunkSize == b.options.chunkSize && a.options.totalChunkCount == b.options.totalChunkCount &&
		a.options.maxResidentChunks == b.options.maxResidentChunks && a.options.pixelScaleLimit == b.options.pixelScaleLimit &&
		a.coneFov0 == b.coneFov0 && a.coneFov == b.coneFov && a.coneFoveate == b.coneFoveate &&
		a.behindFoveate == b.behindFoveate && a.tanHalfFovX == b.tanHalfFovX && a.tanHalfFovY == b.tanHalfFovY &&
		a.viewportOutsideWeight == b.viewportOutsideWeight && a.collectViewportStats == b.collectViewportStats &&
		a.collectActiveChunks == b.collectActiveChunks &&
		a.cachePagedPriorities == b.cachePagedPriorities && a.cachePagedScoringInputs == b.cachePagedScoringInputs &&
		a.useFourAryHeap == b.useFourAryHeap && a.bypassTerminalHeap == b.bypassTerminalHeap &&
		a.useCanonicalTerminalRanges == b.useCanonicalTerminalRanges;
}

bool SparkLodSelector::isResidentLocal(const Input& input, int localIndex) const
{
	if (!input.points || !input.slotToChunk || !input.chunks)
		return false;
	if (localIndex < 0 || localIndex >= (int)input.points->size())
		return false;
	if (localIndex >= (int)input.slotToChunk->size())
		return false;
	int chunkIndex = (*input.slotToChunk)[localIndex];
	if (chunkIndex < 0 || chunkIndex >= (int)input.chunks->size())
		return false;
	return (*input.chunks)[chunkIndex].resident;
}

float SparkLodSelector::computePixelScale(const Input& input, int localIndex, const PixelScaleParams& params) const
{
	if ((_priorityCacheActive || _scoringCacheActive) && localIndex >= 0 && size_t(localIndex) < input.slotToChunk->size()) {
		const int chunk = (*input.slotToChunk)[localIndex];
		if (chunk >= 0 && size_t(chunk) < _priorityCachePages.size()) {
			auto& page = _priorityCachePages[chunk];
			const int offset = localIndex - page.localBase;
			if (page.localBase >= 0 && offset >= 0 && offset < page.count)
				return computeCachedPixelScale(input, localIndex, params, page, offset);
		}
	}
	return computeUncachedPixelScale(input, localIndex, params);
}

float SparkLodSelector::computeCachedPixelScale(const Input& input, int localIndex, const PixelScaleParams& params,
	PriorityCachePage& page, int offset) const
{
	if (_priorityCacheActive && page.valid[offset]) {
		++_priorityCacheHits;
		return page.priorities[offset];
	}
	float priority;
	if (_scoringCacheActive) {
		// Allocate a page only when a priority miss first needs its source data.
		// A priority hit above must not force construction of scoring inputs.
		if (page.scoringInputs.size() != size_t(page.count) || page.scoringValid.size() != size_t(page.count)) {
			page.scoringInputs.resize(size_t(page.count));
			page.scoringValid.assign(size_t(page.count), 0);
		}
		if (!page.scoringValid[offset]) {
			ScoringInput target;
			osg::Vec3f position;
			osg::Vec3f scale;
			float alpha = 0.0f;
			if (input.useCompactLodRecords && input.lodNodes && localIndex >= 0 && localIndex < (int)input.lodNodes->size()) {
				const LodNode& node = (*input.lodNodes)[localIndex];
				position = node.position;
				scale = node.scale;
				alpha = node.alpha;
				target.size = node.size;
			} else {
				const auto& source = (*input.points)[localIndex];
				position = source.position;
				scale = source.scale;
				alpha = source.color.a();
				target.size = input.lodSizes && size_t(localIndex) < input.lodSizes->size()
					? (*input.lodSizes)[localIndex]
					: std::max(scale.x(), std::max(scale.y(), scale.z()));
			}
			target.position = position;
			target.viewportRadius = std::numeric_limits<float>::quiet_NaN();
			if (std::isfinite(target.size) && std::isfinite(scale.x()) && std::isfinite(scale.y()) &&
			    std::isfinite(scale.z()) && std::isfinite(alpha)) {
				// Preserve the raw path's float expression order, including signed
				// maxima, alpha expansion and overflow before the finite guard.
				const float maxScale = std::max(std::abs(scale.x()),
					std::max(std::abs(scale.y()), std::abs(scale.z())));
				const float opacity = alpha > 1.0f ? std::min(4.0f * alpha - 3.0f, 5.0f) : 1.0f;
				const float support = 2.8284271f + 0.7f * (opacity - 1.0f);
				const float radius = 1.1f * std::max(std::max(0.0f, target.size), support * maxScale);
				if (std::isfinite(radius))
					target.viewportRadius = radius;
			}
			page.scoringInputs[offset] = target;
			page.scoringValid[offset] = 1;
			++_scoringCachePointsBuilt;
		}
		const auto& point = page.scoringInputs[offset];
		priority = computeCompactPixelScale(input, point, params);
	} else {
		priority = computeUncachedPixelScale(input, localIndex, params);
	}
	if (_priorityCacheActive) {
		++_priorityCacheMisses;
		page.priorities[offset] = priority;
		page.valid[offset] = 1;
	}
	return priority;
}

void SparkLodSelector::preparePriorityCache(const Input& input) const
{
	static_assert(sizeof(ScoringInput) == 20, "Compact scoring inputs must remain 20 bytes per splat");
	_priorityCacheHits = _priorityCacheMisses = 0;
	_scoringCachePointsBuilt = 0;
	const bool enabled = (input.cachePagedPriorities || input.cachePagedScoringInputs) && input.pagedPageSize > 0 &&
		input.options.chunkSize > 0 && input.options.totalPointCount > 0 &&
		input.points && input.slotToChunk && input.globalToLocal && input.chunks;
	const auto& old = _priorityCacheInput;
	const bool sameSource = (_priorityCacheActive || _scoringCacheActive) && enabled && input.dataEpoch == old.dataEpoch &&
		input.points == old.points && input.lodNodes == old.lodNodes &&
		input.useCompactLodRecords == old.useCompactLodRecords && input.lodSizes == old.lodSizes &&
		input.slotToChunk == old.slotToChunk && input.globalToLocal == old.globalToLocal && input.chunks == old.chunks &&
		input.pagedPageSize == old.pagedPageSize &&
		input.options.chunkSize == old.options.chunkSize && input.options.totalPointCount == old.options.totalPointCount;
	const bool sameKey = sameSource && _priorityCacheActive && input.cachePagedPriorities &&
		input.viewMatrix == old.viewMatrix && input.coneFov0 == old.coneFov0 && input.coneFov == old.coneFov &&
		input.coneFoveate == old.coneFoveate && input.behindFoveate == old.behindFoveate &&
		input.tanHalfFovX == old.tanHalfFovX && input.tanHalfFovY == old.tanHalfFovY &&
		input.viewportOutsideWeight == old.viewportOutsideWeight;
	if (!enabled)
		_priorityCachePages.clear();
	else if (!sameKey) {
		// Camera changes invalidate scores, but need not allocate the resident
		// cache again on every moving-camera traversal.
		for (auto& page : _priorityCachePages) {
			std::fill(page.valid.begin(), page.valid.end(), 0);
			if (!sameSource)
				std::fill(page.scoringValid.begin(), page.scoringValid.end(), 0);
		}
	}
	_priorityCacheActive = enabled && input.cachePagedPriorities;
	_scoringCacheActive = enabled && input.cachePagedScoringInputs;
	_priorityCacheInput = enabled ? input : Input{};
	if (!enabled)
		return;
	_priorityCachePages.resize(input.chunks->size());
	for (size_t chunkIndex = 0; chunkIndex < input.chunks->size(); ++chunkIndex) {
		const auto& chunk = (*input.chunks)[chunkIndex];
		auto& page = _priorityCachePages[chunkIndex];
		const uint64_t globalBase = uint64_t(chunkIndex) * uint64_t(input.options.chunkSize);
		const uint64_t localBase = chunk.pageIndex >= 0 ? uint64_t(chunk.pageIndex) * uint64_t(input.pagedPageSize) : 0;
		const uint64_t localEnd = localBase + uint64_t(std::max(0, chunk.count));
		const uint64_t globalEnd = globalBase + uint64_t(std::max(0, chunk.count));
		const bool canonical = chunk.resident && chunk.pageIndex >= 0 && chunk.base >= 0 &&
			uint64_t(chunk.base) == globalBase && globalBase < uint64_t(input.options.totalPointCount) &&
			chunk.count == int(std::min(uint64_t(input.options.chunkSize), uint64_t(input.options.totalPointCount) - globalBase)) &&
			chunk.count > 0 && chunk.count <= input.pagedPageSize && localEnd <= input.points->size() &&
			localEnd <= input.slotToChunk->size() && localEnd <= uint64_t(std::numeric_limits<int>::max()) &&
			globalEnd <= input.globalToLocal->size() &&
			(*input.slotToChunk)[size_t(localBase)] == int(chunkIndex) &&
			(*input.slotToChunk)[size_t(localEnd - 1)] == int(chunkIndex) &&
			(*input.globalToLocal)[size_t(globalBase)] == int(localBase) &&
			(*input.globalToLocal)[size_t(globalEnd - 1)] == int(localEnd - 1);
		if (!canonical) {
			page = PriorityCachePage{}; // Do not retain allocations for evicted pages.
			continue;
		}
		if (page.localBase != int(localBase) || page.count != chunk.count || page.generation != chunk.loadGeneration) {
			page.localBase = int(localBase);
			page.count = chunk.count;
			page.generation = chunk.loadGeneration;
			page.priorities.resize(size_t(chunk.count));
			page.valid.assign(size_t(chunk.count), 0);
			std::fill(page.scoringValid.begin(), page.scoringValid.end(), 0);
		}
		if (!_scoringCacheActive && page.scoringInputs.capacity() != 0) {
			std::vector<ScoringInput>().swap(page.scoringInputs);
			std::vector<unsigned char>().swap(page.scoringValid);
		}
	}
}

float SparkLodSelector::computeUncachedPixelScale(const Input& input, int localIndex, const PixelScaleParams& params) const
{
	if (input.useCompactLodRecords && input.lodNodes && localIndex >= 0 && localIndex < (int)input.lodNodes->size()) {
		const LodNode& node = (*input.lodNodes)[localIndex];
		return computePixelScaleValues(input, node.position, node.scale, node.alpha, node.size, params);
	}
	const MI_GaussianPoint& g = (*input.points)[localIndex];
	float size = (input.lodSizes && localIndex >= 0 && localIndex < (int)input.lodSizes->size())
		? (*input.lodSizes)[localIndex]
		: std::max(g.scale.x(), std::max(g.scale.y(), g.scale.z()));
	return computePixelScaleValues(input, g.position, g.scale, g.color.a(), size, params);
}

float SparkLodSelector::computePixelScaleValues(const Input& input, const osg::Vec3f& position,
	const osg::Vec3f& scale, float alpha, float size, const PixelScaleParams& params) const
{
	osg::Vec4f eyePos = osg::Vec4f(position.x(), position.y(), position.z(), 1.0f) * input.viewMatrix;
	float distance = std::max(1.0e-6f,
		std::sqrt(eyePos.x() * eyePos.x() + eyePos.y() * eyePos.y() + eyePos.z() * eyePos.z()));
	float pixelScale = size / distance;
	float foveate = 1.0f;

	// Foveated LOD: scale pixel_scale by foveation factor
	if (params.foveated) {
		// Compute view-space direction from camera to splat
		float invDistance = 1.0f / distance;

		// Camera forward direction in view space is (0, 0, -1)
		float forwardDot = -eyePos.z();  // delta.dot((0,0,-1))

		const float coneDot0 = params.coneDot0;
		const float coneDot = params.coneDot;

		if (forwardDot <= 0.0f) {
			// Behind camera
			foveate = input.behindFoveate;
		} else {
			float dot = forwardDot * invDistance;
			if (dot >= coneDot0) {
				foveate = 1.0f;
			} else if (dot >= coneDot) {
				float t = (dot - coneDot) / (coneDot0 - coneDot + 1.0e-12f);
				foveate = input.coneFoveate + (1.0f - input.coneFoveate) * t;
			} else {
				float t = dot / (coneDot + 1.0e-12f);
				foveate = input.behindFoveate + (input.coneFoveate - input.behindFoveate) * t;
			}
		}
	}
	if (params.viewportWeighted && std::isfinite(eyePos.x()) && std::isfinite(eyePos.y()) &&
	    std::isfinite(eyePos.z()) && std::isfinite(distance) && std::isfinite(size) &&
	    std::isfinite(scale.x()) && std::isfinite(scale.y()) && std::isfinite(scale.z()) &&
	    std::isfinite(alpha)) {
		// The average-scale LOD size is not a bound for an anisotropic parent.
		// Use its largest Gaussian axis and the renderer's alpha-expanded
		// support, with an extra guard for edge/quantization differences.
		const float maxScale = std::max(std::abs(scale.x()),
			std::max(std::abs(scale.y()), std::abs(scale.z())));
		const float opacity = alpha > 1.0f ? std::min(4.0f * alpha - 3.0f, 5.0f) : 1.0f;
		const float support = 2.8284271f + 0.7f * (opacity - 1.0f);
		const float radius = 1.1f * std::max(std::max(0.0f, size), support * maxScale);
		const float depth = -eyePos.z();
		// A support volume containing the camera keeps its original priority.
		// Side-plane overlap also leaves large parents near the viewport intact.
		if (std::isfinite(radius) && distance > radius) {
			const float outsideX = (std::abs(eyePos.x()) - depth * input.tanHalfFovX) * params.viewportInvNormalX;
			const float outsideY = (std::abs(eyePos.y()) - depth * input.tanHalfFovY) * params.viewportInvNormalY;
			const float clearance = std::max(std::max(outsideX, outsideY), -depth) - radius;
			if (std::isfinite(clearance) && clearance > 0.0f) {
				const float t = std::min(1.0f, clearance / std::max(1.0e-6f, 0.1f * distance));
				const float smooth = t * t * (3.0f - 2.0f * t);
				const float viewportWeight = 1.0f + (input.viewportOutsideWeight - 1.0f) * smooth;
				// Do not multiply two penalties together, especially behind the
				// camera. A positive floor retains a complete coarse frontier.
				foveate = std::min(foveate, viewportWeight);
			}
		}
	}

	return pixelScale * foveate;
}

float SparkLodSelector::computeCompactPixelScale(const Input& input, const ScoringInput& point,
	const PixelScaleParams& params) const
{
	// Keep the raw scorer above as an independent oracle. Only the static
	// Gaussian support calculation is replaced; all camera-dependent float
	// operations and the viewport guards retain their original order.
	const auto& position = point.position;
	osg::Vec4f eyePos = osg::Vec4f(position.x(), position.y(), position.z(), 1.0f) * input.viewMatrix;
	float distance = std::max(1.0e-6f,
		std::sqrt(eyePos.x() * eyePos.x() + eyePos.y() * eyePos.y() + eyePos.z() * eyePos.z()));
	float pixelScale = point.size / distance;
	float foveate = 1.0f;
	if (params.foveated) {
		float invDistance = 1.0f / distance;
		float forwardDot = -eyePos.z();
		const float coneDot0 = params.coneDot0;
		const float coneDot = params.coneDot;
		if (forwardDot <= 0.0f) {
			foveate = input.behindFoveate;
		} else {
			float dot = forwardDot * invDistance;
			if (dot >= coneDot0) {
				foveate = 1.0f;
			} else if (dot >= coneDot) {
				float t = (dot - coneDot) / (coneDot0 - coneDot + 1.0e-12f);
				foveate = input.coneFoveate + (1.0f - input.coneFoveate) * t;
			} else {
				float t = dot / (coneDot + 1.0e-12f);
				foveate = input.behindFoveate + (input.coneFoveate - input.behindFoveate) * t;
			}
		}
	}
	if (params.viewportWeighted && std::isfinite(eyePos.x()) && std::isfinite(eyePos.y()) &&
	    std::isfinite(eyePos.z()) && std::isfinite(distance) && std::isfinite(point.viewportRadius)) {
		const float radius = point.viewportRadius;
		const float depth = -eyePos.z();
		if (distance > radius) {
			const float outsideX = (std::abs(eyePos.x()) - depth * input.tanHalfFovX) * params.viewportInvNormalX;
			const float outsideY = (std::abs(eyePos.y()) - depth * input.tanHalfFovY) * params.viewportInvNormalY;
			const float clearance = std::max(std::max(outsideX, outsideY), -depth) - radius;
			if (std::isfinite(clearance) && clearance > 0.0f) {
				const float t = std::min(1.0f, clearance / std::max(1.0e-6f, 0.1f * distance));
				const float smooth = t * t * (3.0f - 2.0f * t);
				const float viewportWeight = 1.0f + (input.viewportOutsideWeight - 1.0f) * smooth;
				foveate = std::min(foveate, viewportWeight);
			}
		}
	}
	return pixelScale * foveate;
}

void SparkLodSelector::collectViewportStats(const Input& input, const PixelScaleParams& params, Result& result) const
{
	if (!input.collectViewportStats || !params.viewportValid || !input.points)
		return;
	CancellationCheck canceled(input);
	for (int localIndex : result.activeIndices) {
		if (canceled())
			return;
		if (localIndex < 0 || size_t(localIndex) >= input.points->size())
			continue;
		const osg::Vec3f& position = (*input.points)[localIndex].position;
		const osg::Vec4f eye = osg::Vec4f(position.x(), position.y(), position.z(), 1.0f) * input.viewMatrix;
		const float depth = -eye.z();
		if (depth <= 0.0f)
			++result.viewportBehindCount;
		else if (std::isfinite(depth) && std::isfinite(eye.x()) && std::isfinite(eye.y()) &&
		         std::abs(eye.x()) <= depth * input.tanHalfFovX &&
		         std::abs(eye.y()) <= depth * input.tanHalfFovY)
			++result.viewportInsideCount;
		else
			++result.viewportOutsideCount;
	}
}

SparkLodSelector::Result SparkLodSelector::select(const Input& input, TraverseMode mode) const
{
	if (!input.resumePagedTraversal || mode != TraverseMode::Standard || input.pagedPageSize <= 0)
		_traversalCheckpoint.reset();
	if (cancellationRequested(input)) {
		_traversalCheckpoint.reset();
		return canceledSelection();
	}
	preparePriorityCache(input);
	// These camera constants are shared by every node in a traversal.
	PixelScaleParams params;
	params.foveated = input.coneFov0 > 0.0f || input.coneFov > 0.0f;
	if (input.coneFov0 > 0.0f)
		params.coneDot0 = std::cos(0.5f * input.coneFov0 * 0.0174532925f);
	if (input.coneFov > 0.0f)
		params.coneDot = std::cos(0.5f * input.coneFov * 0.0174532925f);
	params.coneDot = std::min(params.coneDot, params.coneDot0);
	params.viewportValid = std::isfinite(input.tanHalfFovX) && input.tanHalfFovX > 0.0f &&
		std::isfinite(input.tanHalfFovY) && input.tanHalfFovY > 0.0f;
	params.viewportWeighted = params.viewportValid && std::isfinite(input.viewportOutsideWeight) &&
		input.viewportOutsideWeight > 0.0f && input.viewportOutsideWeight < 1.0f;
	if (params.viewportWeighted) {
		// The radius is in model units. Camera view matrices preserve lengths;
		// unsupported projective/scaled/sheared inputs keep the old priorities.
		for (int row = 0; row < 4 && params.viewportWeighted; ++row)
			for (int column = 0; column < 4; ++column)
				if (!std::isfinite(input.viewMatrix(row, column)))
					params.viewportWeighted = false;
		for (int row = 0; row < 3 && params.viewportWeighted; ++row) {
			if (std::abs(input.viewMatrix(row, 3)) > 1.0e-6)
				params.viewportWeighted = false;
			for (int other = row; other < 3; ++other) {
				double dot = 0.0;
				for (int column = 0; column < 3; ++column)
					dot += input.viewMatrix(row, column) * input.viewMatrix(other, column);
				if (std::abs(dot - (row == other ? 1.0 : 0.0)) > 1.0e-5)
					params.viewportWeighted = false;
			}
		}
		if (std::abs(input.viewMatrix(3, 3) - 1.0) > 1.0e-6)
			params.viewportWeighted = false;
	}
	if (params.viewportWeighted) {
		params.viewportInvNormalX = float(1.0 / std::sqrt(1.0 + double(input.tanHalfFovX) * input.tanHalfFovX));
		params.viewportInvNormalY = float(1.0 / std::sqrt(1.0 + double(input.tanHalfFovY) * input.tanHalfFovY));
	}
	Result result = mode == TraverseMode::Dynamic ? selectDynamic(input, params) : selectStandard(input, params);
	if (result.canceled || cancellationRequested(input)) {
		_traversalCheckpoint.reset();
		return canceledSelection();
	}
	collectViewportStats(input, params, result);
	if (cancellationRequested(input)) {
		_traversalCheckpoint.reset();
		return canceledSelection();
	}
	result.priorityCacheHits = _priorityCacheHits;
	result.priorityCacheMisses = _priorityCacheMisses;
	result.scoringCachePointsBuilt = _scoringCachePointsBuilt;
	for (const auto& page : _priorityCachePages)
		result.scoringCacheBytes += page.scoringInputs.capacity() * sizeof(ScoringInput) +
			page.scoringValid.capacity() * sizeof(unsigned char);
	result.activeChunksValid = input.collectActiveChunks && mode == TraverseMode::Standard && result.candidateComplete;
	if (!result.activeChunksValid)
		result.activeChunks.clear();
	return result;
}

SparkLodSelector::Result SparkLodSelector::selectStandard(const Input& input, const PixelScaleParams& params) const
{
	Result result;
	CancellationCheck canceled(input);
	if (!input.points || !input.globalToLocal || !input.childCounts || !input.childStarts || !input.chunks ||
	    input.options.maxActiveSplats <= 0 || input.options.totalPointCount <= 0 || input.options.chunkSize <= 0) {
		_traversalCheckpoint.reset();
		result.candidateComplete = false;
		return result;
	}

	std::vector<RadItem> frontier;
	std::uint64_t processedItems = 0;
	const size_t capacity = std::min((size_t)input.options.maxActiveSplats, input.points->size());
	frontier.reserve(capacity);
	result.activeIndices.reserve(capacity);
	auto pushFrontier = [&](RadItem item) {
		frontier.push_back(item);
		if (input.useFourAryHeap)
			pushFourAryHeap(frontier);
		else
			std::push_heap(frontier.begin(), frontier.end());
	};
	auto popFrontier = [&]() {
		++processedItems;
		if (input.useFourAryHeap)
			popFourAryHeap(frontier);
		else {
			std::pop_heap(frontier.begin(), frontier.end());
			frontier.pop_back();
		}
	};

	int rootLocal = (!input.globalToLocal->empty() && (*input.globalToLocal)[0] >= 0)
	                ? (*input.globalToLocal)[0] : -1;
	if (!isResidentLocal(input, rootLocal)) {
		_traversalCheckpoint.reset();
		result.candidateComplete = false;
		result.missingChunks.push_back({ 0, std::numeric_limits<float>::max() });
		result.requiredChunks.push_back({ 0, std::numeric_limits<float>::max() });
		return result;
	}
	pushFrontier({ computePixelScale(input, rootLocal, params), rootLocal });

	// Canonical RAD chunks have contiguous global IDs and contiguous physical
	// slots. Validate their page metadata once, rather than chasing the large
	// globalToLocal array and rechecking residency for every admitted child.
	// Invalid, partial-preview or noncanonical chunks retain the general path.
	std::vector<int> pagedChunkBases;
	if (input.pagedPageSize > 0) {
		pagedChunkBases.resize(input.chunks->size(), -1);
		for (size_t chunkIndex = 0; chunkIndex < input.chunks->size(); ++chunkIndex) {
			const RadChunkInfo& chunk = (*input.chunks)[chunkIndex];
			const uint64_t globalBase = uint64_t(chunkIndex) * uint64_t(input.options.chunkSize);
			if (!chunk.resident || chunk.pageIndex < 0 ||
			    globalBase >= uint64_t(input.options.totalPointCount) ||
			    chunk.base < 0 || uint64_t(chunk.base) != globalBase)
				continue;
			const int expectedCount = int(std::min(uint64_t(input.options.chunkSize),
				uint64_t(input.options.totalPointCount) - globalBase));
			if (chunk.count != expectedCount || chunk.count > input.pagedPageSize)
				continue;
			const uint64_t localBase = uint64_t(chunk.pageIndex) * uint64_t(input.pagedPageSize);
			const uint64_t localEnd = localBase + uint64_t(chunk.count);
			const uint64_t globalEnd = globalBase + uint64_t(chunk.count);
			if (localEnd > input.points->size() || localEnd > input.slotToChunk->size() ||
			    localEnd > uint64_t(std::numeric_limits<int>::max()) ||
			    globalEnd > input.globalToLocal->size())
				continue;
			if ((*input.slotToChunk)[size_t(localBase)] != int(chunkIndex) ||
			    (*input.slotToChunk)[size_t(localEnd - 1)] != int(chunkIndex) ||
			    (*input.globalToLocal)[size_t(globalBase)] != int(localBase) ||
			    (*input.globalToLocal)[size_t(globalEnd - 1)] != int(localEnd - 1))
				continue;
			pagedChunkBases[chunkIndex] = int(localBase);
		}
	}

	// Spark tracks priorities per chunk, not per visited node. Keeping the
	// maximum here also bounds downstream cache locks and queue updates.
	std::vector<int> touchedPositions(input.chunks->size(), -1);
	std::vector<int> missingPositions(input.chunks->size(), -1);
	std::vector<int> requiredPositions(input.chunks->size(), -1);
	std::vector<unsigned char> activeChunkSeen(input.collectActiveChunks ? input.chunks->size() : 0, 0);
	auto recordChunk = [](std::vector<ChunkPriority>& priorities, std::vector<int>& positions,
	                      uint32_t chunkIndex, float priority) {
		if (chunkIndex >= positions.size())
			return;
		int& position = positions[chunkIndex];
		if (position < 0) {
			position = (int)priorities.size();
			priorities.push_back({ chunkIndex, priority });
		} else {
			priorities[position].priority = std::max(priorities[position].priority, priority);
		}
	};
	recordChunk(result.requiredChunks, requiredPositions,
	            (uint32_t)(*input.slotToChunk)[rootLocal], frontier.front().pixelScale);
	struct ResidentChild { int localIndex; int chunkIndex; };
	std::vector<ResidentChild> childLocals;

	bool checkpointEligible = input.resumePagedTraversal && !pagedChunkBases.empty();
	bool firstMissingObserved = false;
	bool checkpointReusedAtSameItem = false;
	std::vector<unsigned char> prefixPages(checkpointEligible ? input.chunks->size() : 0, 0);
	std::unique_ptr<TraversalCheckpoint> nextCheckpoint;
	// Dependencies include every scored/enqueued/active resident node, not
	// merely the popped parents. Invalid/scattered pages keep normal selection
	// semantics but cannot certify a reusable prefix.
	auto rememberPrefixPage = [&](int localIndex) {
		if (!checkpointEligible || firstMissingObserved)
			return;
		const int chunk = (*input.slotToChunk)[localIndex];
		if (chunk < 0 || size_t(chunk) >= pagedChunkBases.size() || pagedChunkBases[chunk] < 0 ||
		    localIndex < pagedChunkBases[chunk] ||
		    localIndex - pagedChunkBases[chunk] >= (*input.chunks)[chunk].count) {
			checkpointEligible = false;
			return;
		}
		prefixPages[chunk] = 1;
	};
	rememberPrefixPage(rootLocal);
	if (_traversalCheckpoint) {
		const auto& checkpoint = *_traversalCheckpoint;
		bool valid = checkpointEligible && sameTraversalInput(input, checkpoint.input) &&
			checkpoint.chunkCount == input.chunks->size() && checkpoint.rootLocal == rootLocal;
		for (const auto& dependency : checkpoint.dependencies) {
			if (!valid)
				break;
			if (dependency.chunkIndex >= input.chunks->size()) {
				valid = false;
				break;
			}
			const auto& chunk = (*input.chunks)[dependency.chunkIndex];
			valid = pagedChunkBases[dependency.chunkIndex] >= 0 && chunk.resident &&
				chunk.base == dependency.base && chunk.count == dependency.count &&
				chunk.pageIndex == dependency.pageIndex && chunk.loadGeneration == dependency.generation;
		}
		if (valid) {
			if (!copyCheckpointVector(checkpoint.frontier, frontier, input) ||
			    !copyCheckpointResult(checkpoint.prefix, result, input))
				return canceledSelection();
			touchedPositions = checkpoint.touchedPositions;
			requiredPositions = checkpoint.requiredPositions;
			for (int chunk : result.activeChunks)
				activeChunkSeen[chunk] = 1;
			for (const auto& dependency : checkpoint.dependencies)
				prefixPages[dependency.chunkIndex] = 1;
			processedItems = checkpoint.processedItems;
			result.checkpointResumedItems = processedItems;
			result.checkpointBytes = checkpoint.bytes;
		} else {
			_traversalCheckpoint.reset();
		}
	}
	auto saveFirstMissingCheckpoint = [&]() {
		if (firstMissingObserved)
			return true;
		firstMissingObserved = true;
		if (input.resumePagedTraversal)
			result.checkpointPrefixItems = processedItems;
		constexpr std::uint64_t minimumPrefixItems = 65536;
		if (!checkpointEligible || !result.candidateComplete || processedItems < minimumPrefixItems)
			return true;
		// If the original missing parent still cannot split, the exact same
		// prefix is already stored. Reuse it without copying millions of items.
		if (_traversalCheckpoint && processedItems == _traversalCheckpoint->processedItems) {
			checkpointReusedAtSameItem = true;
			return true;
		}
		auto snapshot = std::make_unique<TraversalCheckpoint>();
		snapshot->input = input;
		snapshot->input.cancelRequested = nullptr;
		snapshot->chunkCount = input.chunks->size();
		snapshot->rootLocal = rootLocal;
		snapshot->processedItems = processedItems;
		if (!copyCheckpointVector(frontier, snapshot->frontier, input) ||
		    !copyCheckpointResult(result, snapshot->prefix, input))
			return false;
		// This is the first missing parent. It is still at the heap root; its
		// touched/required/maxFrontier updates are idempotent and preserve their
		// first-insertion order when replayed. No missing-stop counter has been
		// incremented yet. Remove just this parent's missing discoveries so the
		// resumed iteration tests current residency and records them exactly once.
		snapshot->prefix.missingChunks.clear();
		snapshot->touchedPositions = touchedPositions;
		snapshot->requiredPositions = requiredPositions;
		for (size_t chunkIndex = 0; chunkIndex < prefixPages.size(); ++chunkIndex) {
			if (!prefixPages[chunkIndex])
				continue;
			const auto& chunk = (*input.chunks)[chunkIndex];
			snapshot->dependencies.push_back({uint32_t(chunkIndex), chunk.base, chunk.count,
				chunk.pageIndex, chunk.loadGeneration});
		}
		snapshot->bytes = snapshot->frontier.size() * sizeof(RadItem) +
			(snapshot->prefix.activeIndices.size() + snapshot->prefix.activeChunks.size()) * sizeof(int) +
			(snapshot->prefix.touchedChunks.size() + snapshot->prefix.requiredChunks.size()) * sizeof(ChunkPriority) +
			(snapshot->touchedPositions.size() + snapshot->requiredPositions.size()) * sizeof(int) +
			snapshot->dependencies.size() * sizeof(CheckpointPage);
		if (cancellationRequested(input))
			return false;
		result.checkpointSavedItems = processedItems;
		result.checkpointBytes = snapshot->bytes;
		nextCheckpoint = std::move(snapshot);
		return true;
	};

	auto totalCandidateCount = [&]() {
		return (int)result.activeIndices.size() + (int)frontier.size();
	};
	auto appendCandidateLocal = [&](int localIndex, int chunkIndex) {
		// Every insertion is resident: the root is checked above, general
		// children are checked before visitChild, and canonical children belong
		// to a validated immutable page. Residency cannot change during select.
		// This also covers parents retained after a failed or rejected split.
		if ((int)result.activeIndices.size() >= input.options.maxActiveSplats) {
			result.candidateComplete = false;
			++result.stopActiveCap;
			return false;
		}
		result.activeIndices.push_back(localIndex);
		if (input.collectActiveChunks && !activeChunkSeen[chunkIndex]) {
			activeChunkSeen[chunkIndex] = 1;
			result.activeChunks.push_back(chunkIndex);
		}
		return true;
	};

	while (!frontier.empty()) {
		if (canceled())
			return canceledSelection();
		if (totalCandidateCount() > input.options.maxActiveSplats) {
			result.candidateComplete = false;
			++result.stopActiveCap;
			break;
		}
		RadItem item = frontier.front();
		if (item.pixelScale <= input.options.pixelScaleLimit) {
			++result.stopPixelLimit;
			break;
		}
		result.maxFrontierPixelScale = std::max(result.maxFrontierPixelScale, item.pixelScale);
		int localIndex = item.localIndex;
		// The same insertion invariant as appendCandidateLocal makes this
		// resident check redundant; retain all child-metadata bounds below.
		int chunkIndex = (*input.slotToChunk)[localIndex];
		recordChunk(result.touchedChunks, touchedPositions, (uint32_t)chunkIndex, item.pixelScale);
		recordChunk(result.requiredChunks, requiredPositions, (uint32_t)chunkIndex, item.pixelScale);
		if (localIndex < 0 || localIndex >= (int)input.childCounts->size() || localIndex >= (int)input.childStarts->size()) {
			result.candidateComplete = false;
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}
		uint16_t childCount = (*input.childCounts)[localIndex];
		uint32_t childStart = (*input.childStarts)[localIndex];
		if (childCount == 0) {
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}
		int newTotalCandidateCount = totalCandidateCount() - 1 + (int)childCount;
		if (newTotalCandidateCount > input.options.maxActiveSplats) {
			// As in Spark, retain the complete coarse frontier once the highest
			// priority expansion exceeds the budget. Sorting it is done later.
			++result.stopActiveCap;
			break;
		}

		uint64_t childEndExclusive = (uint64_t)childStart + (uint64_t)childCount;
		if (childEndExclusive > input.globalToLocal->size() || childEndExclusive > (uint64_t)input.options.totalPointCount) {
			result.candidateComplete = false;
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}
		uint32_t lastChunk = (uint32_t)((childEndExclusive - 1) / (uint32_t)input.options.chunkSize);
		if (lastChunk >= (uint32_t)input.options.totalChunkCount || lastChunk >= input.chunks->size()) {
			result.candidateComplete = false;
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}
		const uint32_t firstChunk = childStart / (uint32_t)input.options.chunkSize;
		int additionalChunks = 0;
		for (uint32_t chunk = firstChunk; chunk <= lastChunk; ++chunk)
			if (requiredPositions[chunk] < 0)
				++additionalChunks;
		if (input.options.maxResidentChunks > 0 &&
		    result.requiredChunks.size() + additionalChunks > (size_t)input.options.maxResidentChunks) {
			// Reserve every page needed by a split together, even if it is
			// missing. Otherwise existing low-priority pages fill the pool and
			// prevent higher-priority children from ever being loaded. Keeping
			// this parent also removes old low-priority descendants from the
			// next draw frontier, making their pages safe to retire afterward.
			++result.stopResidentCap;
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}
		for (uint32_t chunk = firstChunk; chunk <= lastChunk; ++chunk)
			recordChunk(result.requiredChunks, requiredPositions, chunk,
			            item.pixelScale + input.options.pixelScaleLimit);
		bool pagedChildren = !pagedChunkBases.empty();
		for (uint32_t chunk = firstChunk; pagedChildren && chunk <= lastChunk; ++chunk)
			pagedChildren = pagedChunkBases[chunk] >= 0;
		bool childrenResident = true;
		bool canonicalMissingRange = false;
		if (!pagedChildren) {
			// A canonical paged RAD range can be rejected at page granularity.
			// If any page covering this split is unavailable, the complete
			// frontier keeps the parent and does not need to inspect every child
			// through globalToLocal. This is the same coarse-parent decision as
			// the per-child fallback below, while avoiding up to 65,536 indirect
			// lookups and temporary ResidentChild entries for one missing page.
			canonicalMissingRange = input.pagedPageSize > 0 &&
				!pagedChunkBases.empty();
			bool hasResidentChunk = false;
			if (canonicalMissingRange) {
				for (uint32_t chunk = firstChunk; chunk <= lastChunk; ++chunk) {
					if (pagedChunkBases[chunk] >= 0) {
						hasResidentChunk = true;
						continue;
					}
					const auto& descriptor = (*input.chunks)[chunk];
					const uint64_t globalBase = uint64_t(chunk) * uint64_t(input.options.chunkSize);
					const uint64_t globalEnd = globalBase + uint64_t(std::max(0, descriptor.count));
					const bool canonicalDescriptor = !descriptor.resident && descriptor.pageIndex < 0 &&
						descriptor.base >= 0 &&
						uint64_t(descriptor.base) == globalBase && descriptor.count > 0 &&
						descriptor.count <= input.pagedPageSize &&
						globalEnd <= uint64_t(input.options.totalPointCount);
					if (!canonicalDescriptor) {
						canonicalMissingRange = false;
						break;
					}
				}
				canonicalMissingRange = canonicalMissingRange && !hasResidentChunk;
			}
			if (canonicalMissingRange) {
				for (uint32_t chunk = firstChunk; chunk <= lastChunk; ++chunk) {
					if (pagedChunkBases[chunk] >= 0)
						continue;
					recordChunk(result.missingChunks, missingPositions, chunk,
						item.pixelScale + input.options.pixelScaleLimit);
				}
				if (!saveFirstMissingCheckpoint())
					return canceledSelection();
				childrenResident = false;
			}
		}
		if (!pagedChildren && !canonicalMissingRange) {
			// The canonical fast path above has already established the same
			// coarse-parent result. General/scattered mappings retain the exact
			// per-child residency checks, including resident siblings of a
			// missing page.
			childLocals.clear();
			childLocals.reserve(childCount);
			for (uint16_t c = 0; c < childCount; ++c) {
				if (canceled())
					return canceledSelection();
				uint32_t childGlobal = childStart + c;
				int childLocal = (*input.globalToLocal)[childGlobal];
				if (childLocal < 0 || !isResidentLocal(input, childLocal)) {
					childrenResident = false;
					if (childLocal >= 0)
						result.candidateComplete = false;
					recordChunk(result.missingChunks, missingPositions,
					            childGlobal / (uint32_t)input.options.chunkSize,
					            item.pixelScale + input.options.pixelScaleLimit);
				} else {
					rememberPrefixPage(childLocal);
					childLocals.push_back({childLocal, (*input.slotToChunk)[childLocal]});
				}
			}
		}
		if (!childrenResident) {
			if (!saveFirstMissingCheckpoint())
				return canceledSelection();
			++result.stopMissingChildren;
			popFrontier();
			appendCandidateLocal(localIndex, chunkIndex);
			continue;
		}

		popFrontier();
		struct TerminalRange {
			int required = -1;
			float requiredPriority = 0.0f;
			float maxFrontierPriority = 0.0f;
		};
		auto visitChild = [&](int childLocal, int childChunk, PriorityCachePage* priorityPage, const TerminalRange* terminalRange) {
			float pixelScale = priorityPage
				? computeCachedPixelScale(input, childLocal, params, *priorityPage, childLocal - priorityPage->localBase)
				: computePixelScale(input, childLocal, params);
			if (pixelScale <= input.options.pixelScaleLimit)
				appendCandidateLocal(childLocal, childChunk);
			else {
				if (input.bypassTerminalHeap && std::isfinite(pixelScale)) {
					if (terminalRange)
						++result.canonicalTerminalRangeNodes;
					const bool validMetadata = terminalRange ||
						(size_t(childLocal) < input.childCounts->size() && size_t(childLocal) < input.childStarts->size());
					if (validMetadata && (*input.childCounts)[childLocal] == 0) {
						const int chunkIndex = childChunk;
						const int required = terminalRange ? terminalRange->required : requiredPositions[chunkIndex];
						if (terminalRange)
							++result.canonicalTerminalRangeLeaves;
						// A leaf already consumes one candidate slot and can never
						// request children. Removing it from the heap therefore keeps
						// active + frontier, every split budget and all page admission
						// decisions unchanged (apart from unspecified heap ties).
						//
						// Larger-priority children must keep the historical path:
						// an active-cap stop might leave them unpopped, in which case
						// the original traversal would not raise required priorities
						// or maxFrontier. Only bypass when both values already cover
						// this leaf, so downstream page loading sees the same scores.
						if (required >= 0 &&
						    (terminalRange ? terminalRange->requiredPriority : result.requiredChunks[required].priority) >= pixelScale &&
						    (terminalRange ? terminalRange->maxFrontierPriority : result.maxFrontierPixelScale) >= pixelScale) {
							if (appendCandidateLocal(childLocal, childChunk)) {
								recordChunk(result.touchedChunks, touchedPositions, (uint32_t)chunkIndex, pixelScale);
								++result.bypassedTerminalCount;
							}
							return;
						}
					}
				}
				pushFrontier({ pixelScale, childLocal });
			}
		};
		if (pagedChildren) {
			// Keep global child order, including across nonadjacent physical
			// pages, so equal-priority heap entries behave exactly as before.
			for (uint32_t chunk = firstChunk; chunk <= lastChunk; ++chunk) {
				const uint64_t chunkBase = uint64_t(chunk) * uint64_t(input.options.chunkSize);
				const int begin = int(std::max(uint64_t(childStart), chunkBase) - chunkBase);
				const int end = int(std::min(childEndExclusive,
					chunkBase + uint64_t(input.options.chunkSize)) - chunkBase);
				// All nodes in this nonempty range belong to its certified page.
				// Record the prefix dependency once; general children were already
				// recorded individually during their residency validation. No
				// checkpoint can be saved inside visitChild, and cancellation drops
				// the entire partial traversal, so this preserves dependency coverage.
				if (checkpointEligible && !firstMissingObserved)
					prefixPages[chunk] = 1;
				// The canonical child loop already knows its logical chunk and
				// physical offsets. Validate the cached page once per range and
				// avoid chasing slotToChunk for every score hit or recomputation.
				PriorityCachePage* priorityPage = nullptr;
				if ((_priorityCacheActive || _scoringCacheActive) && chunk < _priorityCachePages.size()) {
					auto& page = _priorityCachePages[chunk];
					if (page.localBase == pagedChunkBases[chunk] && page.count >= end)
						priorityPage = &page;
				}
				TerminalRange terminalRange;
				const TerminalRange* terminal = nullptr;
				const int localEnd = pagedChunkBases[chunk] + end;
				if (input.useCanonicalTerminalRanges && input.bypassTerminalHeap &&
				    size_t(localEnd) <= input.childCounts->size() && size_t(localEnd) <= input.childStarts->size()) {
					// This entire physical range has certified chunk membership and
					// valid metadata. requiredChunks and maxFrontier cannot change
					// while visitChild only appends output, touches pages or pushes
					// heap items. Preserve the original comparisons for every leaf.
					terminalRange.required = requiredPositions[chunk];
					if (terminalRange.required >= 0)
						terminalRange.requiredPriority = result.requiredChunks[terminalRange.required].priority;
					terminalRange.maxFrontierPriority = result.maxFrontierPixelScale;
					terminal = &terminalRange;
				}
				for (int local = pagedChunkBases[chunk] + begin; local < localEnd; ++local) {
					if (canceled())
						return canceledSelection();
					visitChild(local, int(chunk), priorityPage, terminal);
				}
			}
		} else {
			for (const ResidentChild& child : childLocals) {
				if (canceled())
					return canceledSelection();
				visitChild(child.localIndex, child.chunkIndex, nullptr, nullptr);
			}
		}
	}

	// The draw order comes from the depth sorter, so drain the heap linearly.
	for (const RadItem& item : frontier) {
		if (canceled())
			return canceledSelection();
		const int chunkIndex = (*input.slotToChunk)[item.localIndex];
		if (!appendCandidateLocal(item.localIndex, chunkIndex)) {
			result.candidateComplete = false;
			break;
		}
		recordChunk(result.touchedChunks, touchedPositions,
		            (uint32_t)chunkIndex, item.pixelScale);
	}
	std::stable_sort(result.requiredChunks.begin(), result.requiredChunks.end(),
		[](const ChunkPriority& a, const ChunkPriority& b) { return a.priority > b.priority; });
	if (!result.candidateComplete || cancellationRequested(input)) {
		_traversalCheckpoint.reset();
		result.checkpointSavedItems = 0;
		result.checkpointBytes = 0;
	} else if (nextCheckpoint) {
		_traversalCheckpoint = std::move(nextCheckpoint);
	} else if (!checkpointReusedAtSameItem) {
		// A fully resident result needs no pending continuation. Also release
		// snapshots whose prefix became short, uncertified or otherwise invalid.
		_traversalCheckpoint.reset();
		result.checkpointBytes = 0;
	}
	return result;
}

SparkLodSelector::Result SparkLodSelector::selectDynamic(const Input& input, const PixelScaleParams& params) const
{
	Result result;
	CancellationCheck canceled(input);
	if (!input.points || !input.globalToLocal || !input.childCounts || !input.childStarts ||
	    input.options.maxActiveSplats <= 0 || input.options.totalPointCount <= 0 || input.options.chunkSize <= 0) {
		result.candidateComplete = false;
		return result;
	}

	const int maxSplats = input.options.maxActiveSplats;
	const float pixelScaleLimit = input.options.pixelScaleLimit;

	// Root check
	int rootLocal = (!input.globalToLocal->empty() && (*input.globalToLocal)[0] >= 0)
	                ? (*input.globalToLocal)[0] : -1;
	if (!isResidentLocal(input, rootLocal)) {
		result.candidateComplete = false;
		result.missingChunks.push_back({ 0, std::numeric_limits<float>::max() });
		return result;
	}

	// Stack and frontier for single-instance dynamic traversal
	// stack: nodes to expand
	// Note: In a fully Spark-aligned implementation, this should be a priority queue 
	// ordered by pixelScale to ensure breadth-first/priority-first expansion.
	// For now, we use a simple vector but sort it periodically to approximate priority queue.
	struct StackItem { 
		int localIndex; 
		float pixelScale; 
		bool operator<(const StackItem& other) const { return pixelScale < other.pixelScale; }
	};
	std::vector<StackItem> stack;
	std::vector<StackItem> frontier;
	std::vector<StackItem> output;  // final output (leaves + below-limit nodes)

	stack.reserve(1024);
	frontier.reserve(1024);
	output.reserve(maxSplats);

	// chunk_max: tracks max pixelScale per chunk for priority ordering
	std::unordered_map<uint32_t, float> chunkMax;

	// Initialize with root
	float rootScale = computePixelScale(input, rootLocal, params);
	stack.push_back({ rootLocal, rootScale });
	chunkMax[0] = std::numeric_limits<float>::max();

	float currentScale = pixelScaleLimit * 100.0f;
	int missingCount = 0;

	// Multi-round iteration
	for (;;) {
		if (cancellationRequested(input))
			return canceledSelection();
		frontier.clear();
		int outputCount = (int)output.size();

		// Sort stack to pop the largest pixelScale first (approximating priority queue)
		std::sort(stack.begin(), stack.end());

		while (!stack.empty()) {
			if (canceled())
				return canceledSelection();
			StackItem item = stack.back();
			stack.pop_back();

			if (item.pixelScale <= currentScale) {
				frontier.push_back(item);
				continue;
			}

			result.maxFrontierPixelScale = std::max(result.maxFrontierPixelScale, item.pixelScale);

			int localIndex = item.localIndex;

			// Track chunk priority
			if (localIndex >= 0 && localIndex < (int)input.slotToChunk->size()) {
				uint32_t chunkIndex = (uint32_t)(*input.slotToChunk)[localIndex];
				auto it = chunkMax.find(chunkIndex);
				if (it == chunkMax.end() || it->second < item.pixelScale)
					chunkMax[chunkIndex] = item.pixelScale;
			}

			// Check if leaf
			if (localIndex < 0 || localIndex >= (int)input.childCounts->size() ||
			    localIndex >= (int)input.childStarts->size()) {
				output.push_back(item);
				continue;
			}
			uint16_t childCount = (*input.childCounts)[localIndex];
			uint32_t childStart = (*input.childStarts)[localIndex];

			if (childCount == 0) {
				output.push_back(item);
				continue;
			}

			// Check children residency
			bool childrenResident = true;
			for (uint16_t c = 0; c < childCount; ++c) {
				if (canceled())
					return canceledSelection();
				uint32_t childGlobal = childStart + c;
				if (childGlobal >= input.globalToLocal->size()) {
					childrenResident = false;
					break;
				}
				int childLocal = (*input.globalToLocal)[childGlobal];
				if (childLocal < 0 || !isResidentLocal(input, childLocal)) {
					childrenResident = false;
					break;
				}
			}

			if (!childrenResident) {
				// Keep parent as output, record missing chunks
				output.push_back(item);
				++missingCount;
				++result.stopMissingChildren;
				result.candidateComplete = false;

				for (uint16_t c = 0; c < childCount; ++c) {
					if (canceled())
						return canceledSelection();
					uint32_t childGlobal = childStart + c;
					if (childGlobal >= input.globalToLocal->size()) continue;
					int childLocal = (*input.globalToLocal)[childGlobal];
					if (!isResidentLocal(input, childLocal)) {
						uint32_t missingChunk = childGlobal / (uint32_t)input.options.chunkSize;
						result.missingChunks.push_back({ missingChunk, item.pixelScale + pixelScaleLimit });
						auto it = chunkMax.find(missingChunk);
						if (it == chunkMax.end() || it->second < item.pixelScale)
							chunkMax[missingChunk] = item.pixelScale;
					}
				}
				continue;
			}

			// Expand children
			for (uint16_t c = 0; c < childCount; ++c) {
				if (canceled())
					return canceledSelection();
				uint32_t childGlobal = childStart + c;
				int childLocal = (*input.globalToLocal)[childGlobal];
				float childPixelScale = computePixelScale(input, childLocal, params);

				if (childPixelScale <= currentScale) {
					if (childPixelScale <= pixelScaleLimit) {
						output.push_back({ childLocal, childPixelScale });
					} else {
						frontier.push_back({ childLocal, childPixelScale });
					}
				} else {
					stack.push_back({ childLocal, childPixelScale });
				}
			}
		}

		outputCount = (int)output.size() + (int)frontier.size();

		// Dynamic scale adjustment (matching Spark's formula)
		float ratio = (float)outputCount / (float)maxSplats;
		float nextScale = 0.99f * currentScale * std::pow(ratio, 0.5f);
		nextScale = std::max(nextScale, 0.5f * currentScale);
		nextScale = std::max(nextScale, pixelScaleLimit);

		bool noFrontier = frontier.empty();

		if (noFrontier || (nextScale == currentScale) || outputCount >= maxSplats) {
			// Final round: add frontier to output
			for (auto& item : frontier) {
				if (canceled())
					return canceledSelection();
				output.push_back(item);
			}
			break;
		}

		// Move frontier back to stack for next round with lower scale
		currentScale = nextScale;
		stack = std::move(frontier);
		frontier.clear();
	}

	// Build result from output — sort by pixelScale descending so truncation
	// keeps the most visible splats instead of arbitrary traversal order
	if (cancellationRequested(input))
		return canceledSelection();
	std::sort(output.begin(), output.end(), [](const auto& a, const auto& b) {
		return a.pixelScale > b.pixelScale;
	});
	int outputSize = std::min((int)output.size(), maxSplats);
	result.activeIndices.reserve(outputSize);
	for (int i = 0; i < outputSize; ++i) {
		if (canceled())
			return canceledSelection();
		result.activeIndices.push_back(output[i].localIndex);
	}

	if ((int)output.size() > maxSplats) {
		result.candidateComplete = false;
		++result.stopActiveCap;
	}

	// Build touched chunks from chunkMax (sorted by priority descending)
	result.touchedChunks.reserve(chunkMax.size());
	for (auto& [chunkIndex, priority] : chunkMax) {
		if (priority > 0.0f)
			result.touchedChunks.push_back({ chunkIndex, priority });
	}
	std::sort(result.touchedChunks.begin(), result.touchedChunks.end(),
	          [](const ChunkPriority& a, const ChunkPriority& b) { return a.priority > b.priority; });

	// Deduplicate missing chunks, keeping highest priority per chunkIndex
	if (cancellationRequested(input))
		return canceledSelection();
	std::sort(result.missingChunks.begin(), result.missingChunks.end(),
	          [](const ChunkPriority& a, const ChunkPriority& b) {
	              return a.chunkIndex < b.chunkIndex ||
	                     (a.chunkIndex == b.chunkIndex && a.priority > b.priority);
	          });
	result.missingChunks.erase(
		std::unique(result.missingChunks.begin(), result.missingChunks.end(),
		            [](const ChunkPriority& a, const ChunkPriority& b) { return a.chunkIndex == b.chunkIndex; }),
		result.missingChunks.end());

	result.missingCount = missingCount;
	return result;
}
