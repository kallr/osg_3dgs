#ifndef GAUSSIAN_LOD_TREE_H
#define GAUSSIAN_LOD_TREE_H

#include "osg/BoundingBox"
#include "osg/Matrixf"
#include "osg/Vec3f"
#include "osg/Viewport"

#include <utility>
#include <vector>

struct MI_GaussianPoint;

class GaussianLODTree
{
public:
	struct Options
	{
		float splitPixelSize = 2.0f;
		float lodScale = 1.0f;
		int maxActiveCount = 0;
	};

	void build(std::vector<MI_GaussianPoint>& points, int originalPointCount);
	void traverse(const osg::Matrix& viewMatrix, const osg::Matrix& projMatrix,
	              const osg::Viewport* viewport, std::vector<int>& outIndices) const;

	void setOptions(const Options& options) { _options = options; }
	const Options& getOptions() const { return _options; }

	bool empty() const { return _root < 0; }
	int getOriginalPointCount() const { return _originalPointCount; }
	int getRootIndex() const { return _root; }

private:
	struct Node
	{
		osg::BoundingBox bounds;
		osg::Vec3f center;
		float radius = 0.0f;
		int left = -1;
		int right = -1;
		int begin = 0;
		int count = 0;
		int renderIndex = -1;
	};

	struct TraversalItem
	{
		float screenRadius = 0.0f;
		int nodeIndex = -1;
		bool operator<(const TraversalItem& rhs) const { return screenRadius < rhs.screenRadius; }
	};

	int buildRecursive(std::vector<MI_GaussianPoint>& points, int begin, int count, int depth);
	MI_GaussianPoint mergePoints(const std::vector<MI_GaussianPoint>& points, int begin, int count,
	                             const Node& node) const;
	float computeScreenRadius(int nodeIndex, const osg::Matrix& viewMatrix, float focalPixels) const;
	bool isOutsideGuardBand(int nodeIndex, const osg::Matrix& viewMatrix, const osg::Matrix& projMatrix,
	                        double viewportWidth, double viewportHeight) const;

		std::vector<Node> _nodes;
	std::vector<int> _indices;
	Options _options;
	int _root = -1;
	int _originalPointCount = 0;
};

#endif
