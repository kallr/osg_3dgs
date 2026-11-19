#ifndef GAUSSIAN_DRAW_OBJ_H
#define GAUSSIAN_DRAW_OBJ_H

#include "osg/Matrixf"
#include "osg/BoundingBox"
#include "osg/Camera"
#include "osg/Node"
#include "osg/Group"
#include "osg/MatrixTransform"
#include "osg/Image"
#include "osg/BufferObject"
#include "osg/Array"
#include "osg/DispatchCompute"
#include "osg/Program"
#include "osg/TextureBuffer"
#include "GaussianLODTree.h"
#include "SparkGpuResources.h"
#include "SparkSortPass.h"
#include "SparkTileBinner.h"
#include "SparkTileRasterPass.h"
#include "SparkCompositePass.h"
#include "RadTypes.h"
#include "SparkOsgDrawable.h"
#include "SparkPortal.h"
#include "SparkActiveSet.h"

#include <cstdint>
#include <vector>
#include <string>
#include <mutex>

class GaussianDrawObj
{
public:
	GaussianDrawObj(const std::string& name, const std::string& shaderDirectory);
	~GaussianDrawObj();
	GaussianDrawObj(const GaussianDrawObj&) = delete;
	GaussianDrawObj& operator=(const GaussianDrawObj&) = delete;
	GaussianDrawObj(GaussianDrawObj&&) = delete;
	GaussianDrawObj& operator=(GaussianDrawObj&&) = delete;

	osg::ref_ptr<osg::Node> createNode(osg::Camera& camera);
	osg::Camera* camera() const { return _camera.get(); }

	void requestSort();
	bool applySortResult(osg::Image* indexImage);
	bool shouldSort(const osg::Matrix& currentView);

	int getNumPoints() const { return _originalPointCount; }
	int getOriginalPointCount() const { return _originalPointCount; }
	int getRenderablePointCount() const { return _renderablePointCount; }
	const std::vector<MI_GaussianPoint>& getPoints() const { return gaussianPoints; }
	const osg::BoundingBox& getBounds() const { return bounds; }
	const osg::BoundingBox& getFocusBounds() const { return _focusBounds.valid() ? _focusBounds : bounds; }

	// LOD
	int getActiveCount() const { return _sparkDrawable.isRadFile() ? _sparkDrawable.drawActiveCount() : _activeSet.getDrawActiveCount(); }
	int getGpuSortDrawCount() const;
	int getCurrentLOD() const { return _sparkDrawable.isRadFile() ? _sparkDrawable.currentLOD() : _activeSet.getLODLevel(); }
	SparkSortPass& sortPass() { return _sparkDrawable.isRadFile() ? _sparkDrawable.sortPass() : _sortPass; }
	const SparkSortPass& sortPass() const { return _sparkDrawable.isRadFile() ? _sparkDrawable.sortPass() : _sortPass; }
	SparkOsgDrawable& sparkDrawable() { return _sparkDrawable; }
	const SparkOsgDrawable& sparkDrawable() const { return _sparkDrawable; }
	bool isRadFile() const { return _sparkDrawable.isRadFile(); }
	bool isLODEnabled() const { return _sparkDrawable.isRadFile() || !_lodTree.empty(); }
	const std::vector<int>& editableIndices() const { return _activeSet.getActiveIndices(); }
	bool supportsPointEditing() const { return !isLODEnabled() && _originalPointCount > 0; }
	void setDeletedPoints(const std::vector<unsigned char>& deleted);
	int getSHOriginalPointCount() const { return _sparkDrawable.isRadFile() ? _sparkDrawable.getSHOriginalPointCount() : _renderablePointCount; }
	bool updateLOD(const osg::Matrix& viewMatrix);
	bool updateRadNativeLOD(const osg::Matrix& viewMatrix);

	// Portal
	SparkPortal* portal() { return _portal.get(); }
	const SparkPortal* portal() const { return _portal.get(); }

private:
	void updateDataImage(osg::Image* paramsImage);
	void updateDataImageRange(osg::Image* paramsImage, int firstSlot, int endSlot);
	void updateSHImage(osg::Image* paramsImage);
	void updateSHImageRange(osg::Image* paramsImage, int firstSlot, int endSlot);
	void updateIndexImage(osg::Image* paramsImage);
	void updateIndexImageRange(osg::Image* paramsImage, int firstIndex, int endIndex);
	void updateIndexImageChangedRanges(osg::Image* paramsImage, const std::vector<int>& previousIndices);
	void ensureIndexTextureCapacity(int requiredIndices);
	void loadShader(osg::StateSet* ss);
	void loadComputeShader();
	void createGpuSortBuffers();
	void updateActiveIndexBuffer();
	osg::ref_ptr<osg::Node> createComputeDispatchNode(bool enableCovSplats = false);

	std::vector<MI_GaussianPoint> readSplatFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readRadFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readPlyFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readWithGaussForge(const std::string& filename, const std::string& ext);
	osg::ref_ptr<osg::Geometry> createQuadGeometry(int instanceCount);

private:
	int nNum = 0;
	int _originalPointCount = 0;
	int _renderablePointCount = 0;
	bool hasSH = false;
	int _globalShOrder = 0;
	std::vector<MI_GaussianPoint> gaussianPoints;
	SparkActiveSet _activeSet;
	osg::BoundingBox bounds;
	osg::BoundingBox _focusBounds;
	osg::ref_ptr<osg::Group> _editableNode;
	osg::observer_ptr<osg::Geometry> _editableQuad;
	osg::observer_ptr<osg::Camera> _camera;
	osg::ref_ptr<osg::Camera::DrawCallback> _sortCallback;
	const std::string _shaderDirectory;

	osg::ref_ptr<osg::Program> _depthComputeProgram;
	osg::ref_ptr<osg::Program> _sortComputeProgram;
	SparkGpuResources _gpuResources;
	osg::ref_ptr<osg::DispatchCompute> _depthComputeDispatch;
	SparkTileBinner _tileBinner;
	SparkTileRasterPass _tileRasterPass;
	SparkCompositePass _compositePass;

	SparkSortPass _sortPass;
	SparkOsgDrawable _sparkDrawable;
	GaussianLODTree _lodTree;

	// RAD state is now owned by _sparkDrawable; no duplicate members needed

	// Portal
	osg::ref_ptr<SparkPortal> _portal = new SparkPortal;
};

#endif
