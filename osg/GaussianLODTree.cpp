#include "GaussianLODTree.h"
#include "GaussianDrawObj.h"

#include "osg/Vec4f"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace
{
	float clamp01(float v)
	{
		return std::max(0.0f, std::min(1.0f, v));
	}

	float maxScale(const MI_GaussianPoint& g)
	{
		return std::max(g.scale.x(), std::max(g.scale.y(), g.scale.z()));
	}
}

void GaussianLODTree::build(std::vector<MI_GaussianPoint>& points, int originalPointCount)
{
	_nodes.clear();
	_indices.clear();
	_root = -1;
	_originalPointCount = std::max(0, std::min(originalPointCount, (int)points.size()));

	if (_originalPointCount <= 0)
		return;

	_indices.resize(_originalPointCount);
	std::iota(_indices.begin(), _indices.end(), 0);
	_nodes.reserve(_originalPointCount * 2 - 1);
	points.reserve(points.size() + std::max(0, _originalPointCount - 1));
	_root = buildRecursive(points, 0, _originalPointCount, 0);
}

int GaussianLODTree::buildRecursive(std::vector<MI_GaussianPoint>& points, int begin, int count, int depth)
{
	Node node;
	node.begin = begin;
	node.count = count;

	for (int i = 0; i < count; ++i) {
		const MI_GaussianPoint& g = points[_indices[begin + i]];
		float extent = std::max(1.0e-4f, maxScale(g));
		node.bounds.expandBy(g.position - osg::Vec3f(extent, extent, extent));
		node.bounds.expandBy(g.position + osg::Vec3f(extent, extent, extent));
	}
	node.center = node.bounds.center();
	node.radius = node.bounds.radius();

	if (count <= 1 || depth >= 24) {
		if (count == 1) {
			node.renderIndex = _indices[begin];
		} else {
			MI_GaussianPoint merged = mergePoints(points, begin, count, node);
			node.renderIndex = (int)points.size();
			points.push_back(merged);
		}
		_nodes.push_back(node);
		return (int)_nodes.size() - 1;
	}

	osg::Vec3f size = node.bounds._max - node.bounds._min;
	int axis = 0;
	if (size.y() > size.x() && size.y() >= size.z()) axis = 1;
	else if (size.z() > size.x() && size.z() >= size.y()) axis = 2;

	std::sort(_indices.begin() + begin, _indices.begin() + begin + count,
		[&points, axis](int a, int b) {
			return points[a].position[axis] < points[b].position[axis];
		});

	int nodeIndex = (int)_nodes.size();
	_nodes.push_back(node);

	int half = count / 2;
	int left = buildRecursive(points, begin, half, depth + 1);
	int right = buildRecursive(points, begin + half, count - half, depth + 1);

	MI_GaussianPoint merged = mergePoints(points, begin, count, node);
	int renderIndex = (int)points.size();
	points.push_back(merged);

	Node& out = _nodes[nodeIndex];
	out.left = left;
	out.right = right;
	out.renderIndex = renderIndex;
	return nodeIndex;
}

MI_GaussianPoint GaussianLODTree::mergePoints(const std::vector<MI_GaussianPoint>& points, int begin, int count,
                                              const Node& node) const
{
	MI_GaussianPoint merged;
	merged.position.set(0.0f, 0.0f, 0.0f);
	merged.color.set(0.0f, 0.0f, 0.0f, 0.0f);
	merged.scale.set(0.0f, 0.0f, 0.0f);
	merged.rotation.set(0.0f, 0.0f, 0.0f, 1.0f);
	for (int k = 0; k < 45; ++k)
		merged.sh[k] = 0.0f;
	float weightSum = 0.0f;
	float colorWeightSum = 0.0f;
	float alphaProduct = 1.0f;
	float bestWeight = -1.0f;
	osg::Vec4f bestRotation(0.0f, 0.0f, 0.0f, 1.0f);

	for (int i = 0; i < count; ++i) {
		const MI_GaussianPoint& g = points[_indices[begin + i]];
		float extent = std::max(1.0e-4f, maxScale(g));
		float alpha = clamp01(g.color.a());
		float weight = std::max(1.0e-4f, alpha * extent * extent);
		merged.position += g.position * weight;
		merged.color.r() += g.color.r() * alpha * weight;
		merged.color.g() += g.color.g() * alpha * weight;
		merged.color.b() += g.color.b() * alpha * weight;
		for (int k = 0; k < 45; ++k)
			merged.sh[k] += g.sh[k] * weight;
		weightSum += weight;
		colorWeightSum += alpha * weight;
		if (weight > bestWeight) {
			bestWeight = weight;
			bestRotation = g.rotation;
		}
		alphaProduct *= (1.0f - alpha);
	}

	if (weightSum > 0.0f) {
		merged.position /= weightSum;
		float premulDenom = std::max(1.0e-4f, colorWeightSum);
		merged.color.r() /= premulDenom;
		merged.color.g() /= premulDenom;
		merged.color.b() /= premulDenom;
		for (int k = 0; k < 45; ++k)
			merged.sh[k] /= weightSum;
	} else {
		merged.position = node.bounds.center();
	}

	osg::Vec3f ext = node.bounds._max - node.bounds._min;
	float radius = std::max(1.0e-4f, node.radius);
	merged.scale.set(std::max(1.0e-4f, ext.x() * 0.5f),
	                 std::max(1.0e-4f, ext.y() * 0.5f),
	                 std::max(1.0e-4f, ext.z() * 0.5f));
	if (merged.scale.length2() <= 1.0e-8f)
		merged.scale.set(radius, radius, radius);

	float combinedAlpha = clamp01(1.0f - alphaProduct);
	merged.color.a() = combinedAlpha;
	float qLen = std::sqrt(bestRotation.x() * bestRotation.x() + bestRotation.y() * bestRotation.y() +
	                       bestRotation.z() * bestRotation.z() + bestRotation.w() * bestRotation.w());
	if (qLen > 1.0e-6f)
		merged.rotation.set(bestRotation.x() / qLen, bestRotation.y() / qLen,
		                    bestRotation.z() / qLen, bestRotation.w() / qLen);
	else
		merged.rotation.set(0.0f, 0.0f, 0.0f, 1.0f);
	return merged;
}

void GaussianLODTree::traverse(const osg::Matrix& viewMatrix, const osg::Matrix& projMatrix,
                               const osg::Viewport* viewport, std::vector<int>& outIndices) const
{
	outIndices.clear();
	if (_root < 0)
		return;

	if (!viewport || viewport->height() <= 0.0) {
		for (int i = 0; i < _originalPointCount; ++i)
			outIndices.push_back(i);
		return;
	}

	float focalPixels = (float)(projMatrix(1, 1) * viewport->height() * 0.5);

	std::vector<TraversalItem> frontier;
	frontier.reserve(_originalPointCount);
	if (!isOutsideGuardBand(_root, viewMatrix, projMatrix, viewport->width(), viewport->height())) {
		frontier.push_back({computeScreenRadius(_root, viewMatrix, focalPixels), _root});
		std::push_heap(frontier.begin(), frontier.end());
	}

	int activeCount = (int)frontier.size();
	while (!frontier.empty()) {
		TraversalItem item = frontier.front();
		const Node& node = _nodes[item.nodeIndex];
		bool canSplit = node.left >= 0 && node.right >= 0;
		if (!canSplit || item.screenRadius <= _options.splitPixelSize)
			break;

		bool leftVisible = !isOutsideGuardBand(node.left, viewMatrix, projMatrix, viewport->width(), viewport->height());
		bool rightVisible = !isOutsideGuardBand(node.right, viewMatrix, projMatrix, viewport->width(), viewport->height());
		int visibleChildren = (leftVisible ? 1 : 0) + (rightVisible ? 1 : 0);
		int newActiveCount = activeCount - 1 + visibleChildren;
		if (_options.maxActiveCount > 0 && newActiveCount > _options.maxActiveCount)
			break;

		std::pop_heap(frontier.begin(), frontier.end());
		frontier.pop_back();
		if (leftVisible) {
			frontier.push_back({computeScreenRadius(node.left, viewMatrix, focalPixels), node.left});
			std::push_heap(frontier.begin(), frontier.end());
		}
		if (rightVisible) {
			frontier.push_back({computeScreenRadius(node.right, viewMatrix, focalPixels), node.right});
			std::push_heap(frontier.begin(), frontier.end());
		}
		activeCount = newActiveCount;
	}

	outIndices.reserve(frontier.size());
	while (!frontier.empty()) {
		std::pop_heap(frontier.begin(), frontier.end());
		const Node& node = _nodes[frontier.back().nodeIndex];
		if (node.renderIndex >= 0)
			outIndices.push_back(node.renderIndex);
		frontier.pop_back();
	}
}

float GaussianLODTree::computeScreenRadius(int nodeIndex, const osg::Matrix& viewMatrix, float focalPixels) const
{
	const Node& node = _nodes[nodeIndex];
	osg::Vec4f eyePos = osg::Vec4f(node.center.x(), node.center.y(), node.center.z(), 1.0f) * viewMatrix;
	float depth = std::max(1.0e-4f, std::abs(eyePos.z()));
	return node.radius * focalPixels * _options.lodScale / depth;
}

bool GaussianLODTree::isOutsideGuardBand(int nodeIndex, const osg::Matrix& viewMatrix,
                                         const osg::Matrix& projMatrix,
                                         double viewportWidth, double viewportHeight) const
{
	if (viewportWidth <= 0.0 || viewportHeight <= 0.0)
		return false;

	const Node& node = _nodes[nodeIndex];
	const float radius = std::max(1.0e-4f, node.radius);
	osg::Vec4f eyePos = osg::Vec4f(node.center.x(), node.center.y(), node.center.z(), 1.0f) * viewMatrix;

	// OpenGL eye space looks down -Z. Cull only if the whole node is behind the eye;
	// keep near-plane intersections conservative to avoid dropping visible children.
	if (eyePos.z() > radius)
		return true;
	if (eyePos.z() > -radius)
		return false;

	osg::Vec4f clip = eyePos * projMatrix;
	if (clip.w() <= 0.0f)
		return false;

	float invW = 1.0f / clip.w();
	float ndcX = clip.x() * invW;
	float ndcY = clip.y() * invW;
	if (!std::isfinite(ndcX) || !std::isfinite(ndcY))
		return false;

	float depth = std::max(1.0e-4f, -eyePos.z());
	float radiusNdcX = radius * std::abs((float)projMatrix(0, 0)) / depth;
	float radiusNdcY = radius * std::abs((float)projMatrix(1, 1)) / depth;
	const float guard = 2.0f;

	return (ndcX + radiusNdcX < -guard || ndcX - radiusNdcX > guard ||
	        ndcY + radiusNdcY < -guard || ndcY - radiusNdcY > guard);
}
