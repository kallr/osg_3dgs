#ifndef GAUSSIAN_DRAW_OBJ_H
#define GAUSSIAN_DRAW_OBJ_H

#include "osg/Matrixf"
#include "osg/BoundingBox"
#include "osg/Node"
#include "osg/Group"
#include "osg/MatrixTransform"
#include "osg/Image"

#include <vector>
#include <string>
#include <thread>
#include <mutex>
#include <atomic>

struct MI_GaussianPoint {
	osg::Vec3f position;
	osg::Vec4f color;      // DC color (rgba), alpha = sigmoid(opacity)
	osg::Vec3f scale;      // log-space scale (already exponentiated for .splat)
	osg::Vec4f rotation;   // quaternion (x, y, z, w)
	float sh[45] = {};     // 1~3 order SH coefficients
};

class GaussianSortThread
{
public:
	GaussianSortThread();
	~GaussianSortThread();

	void start();
	void stop();

	void setPositions(const std::vector<MI_GaussianPoint>* points, int numPoints);
	void requestSort(const osg::Matrix& modelView);
	bool fetchResult(std::vector<int>& outIndices);

private:
	void run();

	std::thread _thread;
	std::mutex _taskMutex;
	std::mutex _resultMutex;
	std::atomic<bool> _running{false};
	std::atomic<bool> _hasTask{false};
	std::atomic<bool> _hasResult{false};

	const std::vector<MI_GaussianPoint>* _points = nullptr;
	int _numPoints = 0;
	osg::Matrix _taskModelView;

	std::vector<int> _resultIndices;
};

class GaussianDrawObj
{
public:
	GaussianDrawObj(const std::string& name);
	~GaussianDrawObj();

	osg::ref_ptr<osg::Node> getNode();

	void requestSort();
	bool applySortResult(osg::Image* indexImage);
	bool shouldSort(const osg::Matrix& currentView);

	int getNumPoints() const { return nNum; }
	const std::vector<MI_GaussianPoint>& getPoints() const { return gaussianPoints; }
	const osg::BoundingBox& getBounds() const { return bounds; }

private:
	void updateDataImage(osg::Image* paramsImage);
	void updateSHImage(osg::Image* paramsImage);
	void updateIndexImage(osg::Image* paramsImage);
	void loadShader(osg::StateSet* ss);

	std::vector<MI_GaussianPoint> readSplatFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readPlyFile(const std::string& filename);
	osg::ref_ptr<osg::Geometry> createQuadGeometry();

private:
	int nNum = 0;
	bool hasSH = false;
	std::vector<MI_GaussianPoint> gaussianPoints;
	std::vector<int> depthIndex;
	osg::BoundingBox bounds;

	osg::Matrix _lastViewMatrix;
	bool _firstSort = true;

	GaussianSortThread _sortThread;
};

void setMainCamera(osg::Camera* pC);
osg::Camera* getCamera();

#endif
