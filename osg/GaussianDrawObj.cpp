#include "GaussianDrawObj.h"
#include "SparkRadFile.h"
#include "SparkRadQuality.h"
#include "SparkLodSelector.h"
#include "SparkSplatEdit.h"
#include "SparkSplatSkinning.h"
#include "SparkShaderLibrary.h"
#include "SparkOsgCallbacks.h"
#include "SparkRenderTypes.h"
#include "SparkDatasetLoader.h"
#include "SparkRadDatasetLoader.h"

#include "osg/Material"
#include "osg/Geode"
#include "osg/Geometry"
#include "osg/TextureBuffer"
#include "osg/Texture2D"
#include "osg/Uniform"
#include "osg/MatrixTransform"
#include "osgUtil/CullVisitor"
#include "osg/ComputeBoundsVisitor"
#include "osg/BufferIndexBinding"
#include "osg/DispatchCompute"
#include "osg/GLExtensions"
#include "osg/State"
#include "osgDB/ReadFile"
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include "tools/miniply.h"

#include "gf/core/gauss_ir.h"
#include "gf/io/registry.h"
#include "gf/io/reader.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <cfloat>
#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <cctype>
#include <cstdlib>
#include <unordered_set>
#include <utility>
#ifdef HAVE_ZLIB
#include <zlib.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef GL_RED_INTEGER
#define GL_RED_INTEGER 0x8D94
#endif
#ifndef GL_R32I
#define GL_R32I 0x8235
#endif
#ifndef GL_TEXTURE_FETCH_BARRIER_BIT
#define GL_TEXTURE_FETCH_BARRIER_BIT 0x00000008
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
#endif
#ifndef GL_RGBA_INTEGER
#define GL_RGBA_INTEGER 0x8D99
#endif
#ifndef GL_FRAMEBUFFER_SRGB
#define GL_FRAMEBUFFER_SRGB 0x8DB9
#endif

namespace
{
	const int DEFAULT_RAD_MAX_RESIDENT = 6291456;
	const float DEFAULT_RAD_LOD_LEVEL = 0.786f;
	const float DEFAULT_RAD_LOD_QUALITY_AT_LEVEL = 0.08f;

	bool isEnvEnabled(const char* name)
	{
		std::string text;
#ifdef _MSC_VER
		char* value = nullptr;
		size_t length = 0;
		if (_dupenv_s(&value, &length, name) != 0 || !value)
			return false;
		text.assign(value);
		std::free(value);
#else
		const char* value = std::getenv(name);
		if (!value)
			return false;
		text.assign(value);
#endif
		if (text.empty())
			return false;
		std::transform(text.begin(), text.end(), text.begin(),
			[](unsigned char ch) { return (char)std::tolower(ch); });
		return text != "0" && text != "false" && text != "off" && text != "no";
	}

	int getEnvInt(const char* name, int fallback, int minValue, int maxValue)
	{
		std::string text;
#ifdef _MSC_VER
		char* value = nullptr;
		size_t length = 0;
		if (_dupenv_s(&value, &length, name) != 0 || !value)
			return fallback;
		text.assign(value);
		std::free(value);
#else
		const char* value = std::getenv(name);
		if (!value)
			return fallback;
		text.assign(value);
#endif
		if (text.empty())
			return fallback;
		int parsed = std::atoi(text.c_str());
		return std::max(minValue, std::min(maxValue, parsed));
	}

	float getEnvFloat(const char* name, float fallback, float minValue, float maxValue)
	{
		std::string text;
#ifdef _MSC_VER
		char* value = nullptr;
		size_t length = 0;
		if (_dupenv_s(&value, &length, name) != 0 || !value)
			return fallback;
		text.assign(value);
		std::free(value);
#else
		const char* value = std::getenv(name);
		if (!value)
			return fallback;
		text.assign(value);
#endif
		if (text.empty())
			return fallback;
		float parsed = (float)std::atof(text.c_str());
		return std::max(minValue, std::min(maxValue, parsed));
	}

	bool isEnvSet(const char* name)
	{
#ifdef _MSC_VER
		char* value = nullptr;
		size_t length = 0;
		bool exists = _dupenv_s(&value, &length, name) == 0 && value;
		std::free(value);
		return exists;
#else
		return std::getenv(name) != nullptr;
#endif
	}

	bool radDefaultBool(const char* enableName, const char* disableName, bool defaultValue)
	{
		if (isEnvSet(disableName))
			return !isEnvEnabled(disableName);
		if (isEnvSet(enableName))
			return isEnvEnabled(enableName);
		return defaultValue;
	}

	bool useRadSparkPath(bool isRadFile)
	{
		return radDefaultBool("OSG_3DGS_RAD_SPARK_PATH", "OSG_3DGS_DISABLE_RAD_SPARK_PATH", isRadFile);
	}

	float radAwareDefault(bool isRadFile, float radValue, float defaultValue)
	{
		return isRadFile ? radValue : defaultValue;
	}

	int getPackedEncodingMode(bool requirePacked)
	{
		// RAD stores F32 centers; preserve them through GPU upload for fine detail.
		int packedMode = getEnvInt("OSG_3DGS_PACKED_ENCODING", requirePacked ? 2 : 0, 0, 2);
		if (requirePacked && packedMode == 0) {
			std::cout << "Spark Paged Pool requires packed encoding; forcing OSG_3DGS_PACKED_ENCODING=1" << std::endl;
			packedMode = 1;
		}
		return packedMode;
	}

	void logRadRenderControls(bool enableBlurCompensation, bool sparkRadiusClamp, bool lodInflate,
	                          float supportRadiusScale, float maxScreenRadius,
	                          float preBlurAmount, float blurAmount)
	{
		static bool logged = false;
		if (logged)
			return;
		logged = true;
		std::cout << "RAD render controls: blurCompensation=" << (enableBlurCompensation ? "on" : "off")
		          << ", sparkRadiusClamp=" << (sparkRadiusClamp ? "on" : "off")
		          << ", lodInflate=" << (lodInflate ? "on" : "off")
		          << ", supportRadiusScale=" << supportRadiusScale
		          << ", maxScreenRadius=" << maxScreenRadius
		          << ", preBlurAmount=" << preBlurAmount
		          << ", blurAmount=" << blurAmount
		          << " (override with OSG_3DGS_ENABLE_*/OSG_3DGS_DISABLE_* and numeric OSG_3DGS_* vars)" << std::endl;
	}

	bool isLODEnabled()
	{
		return isEnvEnabled("OSG_3DGS_ENABLE_LOD");
	}

	int nextPowerOfTwo(int v)
	{
		int p = 1;
		while (p < v)
			p <<= 1;
		return p;
	}

}

// ======================== Callbacks ========================

class GaussianSortCallback : public osg::Camera::DrawCallback
{
public:
	GaussianSortCallback(GaussianDrawObj* obj) : _obj(obj) {}

	virtual void operator()(osg::RenderInfo& renderInfo) const
	{
		if (!_obj || !renderInfo.getCurrentCamera()) return;
		osg::Matrix viewMatrix = renderInfo.getCurrentCamera()->getViewMatrix();

		bool lodChanged = _obj->isLODEnabled() ? _obj->updateLOD(viewMatrix) : false;
		bool shouldSort = false;
		if (!lodChanged)
			shouldSort = _obj->shouldSort(viewMatrix);

		if (isEnvEnabled("OSG_3DGS_RAD_LOD_TRACE")) {
			std::cout << "RAD LOD trace: callback"
				<< ", eyeT=(" << viewMatrix(3, 0) << "," << viewMatrix(3, 1) << "," << viewMatrix(3, 2) << ")"
				<< ", lodEnabled=" << (_obj->isLODEnabled() ? 1 : 0)
				<< ", lodChanged=" << (lodChanged ? 1 : 0)
				<< ", shouldSort=" << (shouldSort ? 1 : 0)
				<< std::endl;
		}

		if (lodChanged || shouldSort)
			_obj->requestSort();

        // Flush any pending Paged Pool partial updates to GPU
        if (_obj->sparkDrawable().isRadFile()) {
            osg::State* state = renderInfo.getState();
            if (state) {
                SparkGpuResources& gpu = _obj->sparkDrawable().gpuResources();
                gpu.flushPendingPageUpdates(*state);

            }
        }
	}

private:
	GaussianDrawObj* _obj;
};

class SSCallback : public osg::StateSet::Callback
{
public:
	SSCallback(GaussianDrawObj* obj) : _obj(obj) {}

	virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
	{
		if (!ss) return;
		osg::Camera* camera = _obj ? _obj->camera() : nullptr;
		if (!camera) return;

		if (!_viewUniform) {
			_viewUniform = ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4);
			_projUniform = ss->getOrCreateUniform("proj", osg::Uniform::FLOAT_MAT4);
			_viewportUniform = ss->getOrCreateUniform("viewport_size", osg::Uniform::FLOAT_VEC2);
		}

		osg::Matrixf viewMatrix = camera->getViewMatrix();
		_viewUniform->set(viewMatrix);


		osg::Matrixf projMatrix = camera->getProjectionMatrix();
		_projUniform->set(projMatrix);

		osg::Viewport* viewport = camera->getViewport();
		if (viewport) {
			osg::Vec2 vs;
			vs[0] = viewport->width();
			vs[1] = viewport->height();
			_viewportUniform->set(vs);
		}

		// Update portal uniforms if portal is enabled
		if (_obj && _obj->portal() && _obj->portal()->enablePortal()) {
			if (!_enablePortalUniform) {
				_enablePortalUniform = ss->getOrCreateUniform("enablePortal", osg::Uniform::BOOL);
				_diskCenterUniform = ss->getOrCreateUniform("diskCenter", osg::Uniform::FLOAT_VEC3);
				_diskNormalUniform = ss->getOrCreateUniform("diskNormal", osg::Uniform::FLOAT_VEC3);
				_diskRadiusUniform = ss->getOrCreateUniform("diskRadius", osg::Uniform::FLOAT);
				_diskTwoSidedUniform = ss->getOrCreateUniform("diskTwoSided", osg::Uniform::BOOL);
			}

			SparkPortal* portal = _obj->portal();
			osg::Vec2f viewportSize;
			if (viewport) {
				viewportSize.set(viewport->width(), viewport->height());
			}
			portal->update(viewMatrix, projMatrix, viewportSize);

			_enablePortalUniform->set(portal->enablePortal());
			_diskCenterUniform->set(portal->diskCenter());
			_diskNormalUniform->set(portal->diskNormal());
			_diskRadiusUniform->set(portal->diskRadius());
			_diskTwoSidedUniform->set(portal->diskTwoSided());
		}
	}

private:
	GaussianDrawObj* _obj;
	osg::Uniform* _viewUniform = nullptr;
	osg::Uniform* _projUniform = nullptr;
	osg::Uniform* _viewportUniform = nullptr;
	osg::Uniform* _enablePortalUniform = nullptr;
	osg::Uniform* _diskCenterUniform = nullptr;
	osg::Uniform* _diskNormalUniform = nullptr;
	osg::Uniform* _diskRadiusUniform = nullptr;
	osg::Uniform* _diskTwoSidedUniform = nullptr;
};

class ComputeSSCallback : public osg::StateSet::Callback
{
public:
	ComputeSSCallback(GaussianDrawObj* obj, osg::DispatchCompute* dispatch)
		: _obj(obj), _dispatch(dispatch) {}

	virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
	{
		if (!ss || !_obj) return;
		osg::Camera* camera = _obj->camera();
		if (!camera) return;

		if (!_viewUniform) {
			_viewUniform = ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4);
			_activeCountUniform = ss->getOrCreateUniform("activeCount", osg::Uniform::INT);
			_sortCountUniform = ss->getOrCreateUniform("sortCount", osg::Uniform::INT);
		}

		int activeCount = _obj->getGpuSortDrawCount();
		int sortCount = _obj->sortPass().gpuSortCapacity();
		osg::Matrixf viewMatrix = camera->getViewMatrix();
		_viewUniform->set(viewMatrix);
		_activeCountUniform->set(activeCount);
		_sortCountUniform->set(sortCount);

		if (_dispatch) {
			int groupsX = std::max(1, (sortCount + 255) / 256);
			_dispatch->setComputeGroups(groupsX, 1, 1);
		}
	}

private:
	GaussianDrawObj* _obj;
	osg::DispatchCompute* _dispatch;
	osg::Uniform* _viewUniform = nullptr;
	osg::Uniform* _activeCountUniform = nullptr;
	osg::Uniform* _sortCountUniform = nullptr;
};

class IndexBufferUpdateCallback : public osg::StateAttributeCallback
{
public:
	IndexBufferUpdateCallback(GaussianDrawObj* obj, osg::Geometry* geom)
		: _obj(obj)
	{
		_drawElements = geom ? dynamic_cast<osg::DrawElementsUShort*>(geom->getPrimitiveSet(0)) : nullptr;
	}

	virtual void operator()(osg::StateAttribute* sa, osg::NodeVisitor* nv)
	{
		if (!_obj) return;
		osg::TextureBuffer* tbo = dynamic_cast<osg::TextureBuffer*>(sa);
		if (!tbo) return;
		osg::Image* indexImage = tbo->getImage();
		if (!indexImage) return;

		_obj->applySortResult(indexImage);

		// Dynamically update instance count for LOD/full-resolution drawing
		if (_drawElements) {
			int activeCount = _obj->getGpuSortDrawCount();
			if (activeCount != _lastInstanceCount) {
				_drawElements->setNumInstances(activeCount);
				_lastInstanceCount = activeCount;
			}
		}
	}

private:
	GaussianDrawObj* _obj;
	osg::DrawElementsUShort* _drawElements = nullptr;
	int _lastInstanceCount = -1;
};

// ======================== GaussianDrawObj ========================

int GaussianDrawObj::getGpuSortDrawCount() const
{
	const SparkSortPass& sort = sortPass();
	int activeCount = getActiveCount();
	return sort.useGpuSort() ? std::min(activeCount, sort.gpuSortCapacity()) : activeCount;
}

void GaussianDrawObj::setDeletedPoints(const std::vector<unsigned char>& deleted)
{
	if (!supportsPointEditing() || deleted.size() != size_t(_originalPointCount))
		return;

	auto& active = _activeSet.getActiveIndices();
	active.clear();
	bounds.init();
	for (int i = 0; i < _originalPointCount; ++i) {
		if (deleted[i]) continue;
		active.push_back(i);
		bounds.expandBy(gaussianPoints[i].position);
	}
	if (_editableQuad.valid()) _editableQuad->setInitialBound(bounds);

	_sortPass.noteActiveSetChanged();
	_activeSet.setActiveSetVersion(_activeSet.getActiveSetVersion() + 1);
	_activeSet.setDrawActiveCount(int(active.size()));
	// Limit the published order immediately so an old CPU sort cannot redraw deleted points.
	auto& order = _activeSet.getDepthIndex();
	order.erase(std::remove_if(order.begin(), order.end(), [&](int i) { return deleted[i] != 0; }), order.end());
	if (order.size() != active.size()) order = active;
	updateIndexImage(nullptr);
	// OSG treats zero instances as a regular (non-instanced) draw.
	if (_editableNode) _editableNode->setNodeMask(active.empty() ? 0u : ~0u);
	if (!active.empty()) requestSort();
}

GaussianDrawObj::GaussianDrawObj(const std::string& name, const std::string& shaderDirectory)
	: _shaderDirectory(shaderDirectory + "/")
{
	_tileBinner.setEnabled(isEnvEnabled("OSG_3DGS_ENABLE_TILE_BINNING"));

	// Determine file extension
	std::string ext;
	size_t dotPos = name.rfind('.');
	if (dotPos != std::string::npos) {
		ext = name.substr(dotPos + 1);
		std::transform(ext.begin(), ext.end(), ext.begin(),
			[](unsigned char ch) { return (char)std::tolower(ch); });
	}

	// Use GaussForge for all supported formats
	if (ext == "spz" || ext == "ksplat")
	{
		gaussianPoints = readWithGaussForge(name, ext);
	}
	else if (ext == "ply")
	{
		// Try GaussForge first, fall back to miniply
		gaussianPoints = readWithGaussForge(name, ext);
		if (gaussianPoints.empty())
			gaussianPoints = readPlyFile(name);
	}
	else if (ext == "splat")
	{
		gaussianPoints = readWithGaussForge(name, ext);
		if (gaussianPoints.empty())
			gaussianPoints = readSplatFile(name);
	}
	else if (ext == "rad")
	{
		gaussianPoints = readRadFile(name);
	}
	else
	{
		gaussianPoints = readWithGaussForge(name, ext);
	}

	if (_sparkDrawable.isRadFile()) {
		_originalPointCount = _sparkDrawable.radTotalPointCount() > 0 ? _sparkDrawable.radTotalPointCount() : (int)gaussianPoints.size();
	} else {
		_originalPointCount = (int)gaussianPoints.size();
	}
	nNum = _originalPointCount;

	if (!isLODEnabled()) {
		_renderablePointCount = _originalPointCount;
		gaussianPoints.resize(_originalPointCount);
	} else if (!_sparkDrawable.isRadFile()) {
		_lodTree.build(gaussianPoints, _originalPointCount);
		_renderablePointCount = (int)gaussianPoints.size();
	} else {
		_renderablePointCount = (int)gaussianPoints.size();
	}
	if (_sparkDrawable.isRadFile()) {
		int rootLocal = (!_sparkDrawable._radGlobalToLocal.empty() && _sparkDrawable._radGlobalToLocal[0] >= 0)
		                ? _sparkDrawable._radGlobalToLocal[0] : -1;
		int requestedBootstrapActive = getEnvInt("OSG_3DGS_RAD_BOOTSTRAP_ACTIVE", 65536, 1, 1200000);
		int bootstrapActive = std::min(requestedBootstrapActive, 65536);
		if (requestedBootstrapActive != bootstrapActive) {
			std::cout << "RAD bootstrap active requested=" << requestedBootstrapActive
			          << ", clamped=" << bootstrapActive
			          << " for first-frame stability" << std::endl;
		}
		_sparkDrawable._depthIndex.reserve(std::min(bootstrapActive, (int)gaussianPoints.size()));
		if (rootLocal >= 0 && rootLocal < (int)gaussianPoints.size())
			_sparkDrawable._depthIndex.push_back(rootLocal);
		else if (!gaussianPoints.empty())
			std::cerr << "RAD root node is not resident after initial load; bootstrapping resident slots" << std::endl;
		for (int i = 0; i < (int)gaussianPoints.size() && (int)_sparkDrawable._depthIndex.size() < bootstrapActive; ++i) {
			if (i != rootLocal)
				_sparkDrawable._depthIndex.push_back(i);
		}
		int maxInitialActive = SparkRadQualitySettings::fromEnvironment().movingSplats;
		if (_sparkDrawable._depthIndex.size() > (size_t)maxInitialActive) {
			_sparkDrawable._depthIndex.resize(maxInitialActive);
		}
		std::cout << "RAD initial active=" << _sparkDrawable._depthIndex.size()
		          << ", resident=" << gaussianPoints.size()
		          << ", total=" << _originalPointCount << std::endl;
		_sparkDrawable._activeIndices = _sparkDrawable._depthIndex;
		_sparkDrawable._drawActiveCount = (int)_sparkDrawable._activeIndices.size();
	} else {
		_activeSet.initializeSequential(_originalPointCount);
	}
	int reserveCount = _sparkDrawable.isRadFile()
		? std::min(SparkRadQualitySettings::fromEnvironment().capacity(),
		           std::max(1, _sparkDrawable.radResidentCapacity()))
		: _originalPointCount;
	_activeSet.reserveScratch(reserveCount);

	_sortPass.setPositions(&gaussianPoints, (int)gaussianPoints.size());
	loadComputeShader();

	if (_sparkDrawable.isRadFile()) {
		// Initialize SparkOsgDrawable for RAD files
		_sparkDrawable.initializeRad(_sparkDrawable._radFilename, _sparkDrawable._radChunkDataBase, _sparkDrawable._radTotalPointCount, _sparkDrawable._radResidentCapacity);
		_sparkDrawable.setPoints(&gaussianPoints);
		_sparkDrawable.setHasSH(hasSH);

		// Initialize Phase 1 Spark Paged GPU Resource Pool if requested
		const bool useSparkPagedPool = useRadSparkPath(_sparkDrawable.isRadFile());
		const int packedMode = getPackedEncodingMode(useSparkPagedPool);
		_gpuResources.setSplatEncoding(static_cast<SplatEncoding>(packedMode));
		if (useSparkPagedPool) {
			const int maxSupportedPageSize = 262144;
			int pageSize = getEnvInt("OSG_3DGS_RAD_SPARK_PAGE_SIZE", 65536, 1024, maxSupportedPageSize);
			int maxChunkCount = 1;
			for (const RadChunkInfo& chunkInfo : _sparkDrawable.radChunks())
				maxChunkCount = std::max(maxChunkCount, chunkInfo.count);
			if (pageSize < maxChunkCount) {
				if (maxChunkCount <= maxSupportedPageSize) {
					std::cout << "RAD Spark page size raised from " << pageSize
					          << " to " << maxChunkCount
					          << " to fit the largest RAD chunk" << std::endl;
					pageSize = maxChunkCount;
				} else {
					std::cerr << "RAD largest chunk exceeds Spark page size limit: chunkCount=" << maxChunkCount
					          << ", limit=" << maxSupportedPageSize
					          << ". Oversized chunks will be rejected by the paged loader." << std::endl;
				}
			}
			int derivedPages = std::max(1, (_sparkDrawable._radResidentCapacity + pageSize - 1) / pageSize);
			int defaultPages = std::max(16, derivedPages);
			int maxPages = getEnvInt("OSG_3DGS_RAD_SPARK_PAGES", defaultPages, 16, 2048);
			const int activeCapacity = SparkRadQualitySettings::fromEnvironment().capacity();
			_gpuResources.initPagedPool(maxPages, pageSize, activeCapacity);

			for (size_t chunkIndex = 0; chunkIndex < _sparkDrawable.radChunks().size(); ++chunkIndex) {
				RadChunkInfo& chunkInfo = _sparkDrawable.radChunks()[chunkIndex];
				if (!chunkInfo.resident || chunkIndex >= _sparkDrawable._radChunkSlots.size() ||
				    _sparkDrawable._radChunkSlots[chunkIndex].empty()) {
					continue;
				}
				int firstSlot = _sparkDrawable._radChunkSlots[chunkIndex].front();
				int pageIndex = firstSlot / pageSize;
				if (pageIndex >= 0 && pageIndex < _gpuResources.maxPages()) {
					_gpuResources.setPageOccupied(pageIndex, true);
					chunkInfo.pageIndex = pageIndex;
				}
			}
		}

		// Wire SparkOsgDrawable to shared GPU resources and sort pass
		_sparkDrawable.setSharedResources(&_gpuResources, &_sortPass);

		// Configure SparkOsgDrawable's sort pass and GPU resources
		int radRequestedActiveCapacity = SparkRadQualitySettings::fromEnvironment().capacity();
		_sparkDrawable.sortPass().setPositions(&gaussianPoints, (int)gaussianPoints.size());
		_sparkDrawable.sortPass().configureSort(radRequestedActiveCapacity, true);
		int radCapacity = _sparkDrawable.sortPass().gpuSortCapacity();
		_sparkDrawable.gpuResources().setRadMode(true);
		_sparkDrawable.gpuResources().createSortBuffers(radCapacity, _sparkDrawable.activeIndices());

		if (_tileBinner.isEnabled()) {
			_tileBinner.initialize(_sparkDrawable.gpuResources());
		}

		_sparkDrawable.startRadLoader();
		_sparkDrawable.sortPass().start();
	} else {
		createGpuSortBuffers();
		_sortPass.start();
	}
}

GaussianDrawObj::~GaussianDrawObj()
{
	if (_camera.valid() && _sortCallback) _camera->removePreDrawCallback(_sortCallback);
	if (_sparkDrawable.isRadFile()) {
		_sparkDrawable.sortPass().stop();
		_sparkDrawable.stopRadLoader();
	} else {
		_sortPass.stop();
	}
}

bool GaussianDrawObj::shouldSort(const osg::Matrix& currentView)
{
	if (_sparkDrawable.isRadFile())
		return _sparkDrawable.shouldSort(currentView, _sparkDrawable._firstSort);
	float epsilon = 1.0e-5f;
	return _sortPass.shouldSort(currentView, _activeSet.isFirstSort(), epsilon);
}

void GaussianDrawObj::requestSort()
{
	osg::Camera* cam = camera();
	if (!cam) return;

	osg::Matrix viewMatrix = cam->getViewMatrix();

	if (_sparkDrawable.isRadFile()) {
		_sparkDrawable.requestSort(viewMatrix);
		return;
	}

	if (_sortPass.sortDisabled()) {
		_sortPass.setLastViewMatrix(viewMatrix);
		_activeSet.commitSortDisabled();
		return;
	}

	if (_sortPass.useGpuSort()) {
		_sortPass.noteGpuSortDispatched(viewMatrix);
		// Update the SSBO active index buffer so the depth compute
		// shader reads the current active set.  The compute dispatch
		// node in the scene graph will execute depth + sort passes
		// automatically during the next render traversal.
		updateActiveIndexBuffer();
		_activeSet.commitGpuSortDispatched();
		return;
	}

	_activeSet.noteCpuSortRequested();

	_sortPass.requestCpuSort(viewMatrix, _activeSet.getActiveIndices(), _activeSet.getActiveSetVersion());
}

bool GaussianDrawObj::updateLOD(const osg::Matrix& viewMatrix)
{
	if (_sparkDrawable.isRadFile()) {
		return _sparkDrawable.updateRadNativeLOD(viewMatrix, camera());
	}

	osg::Camera* cam = camera();
	if (!cam) return false;

	if (_lodTree.empty())
		return false;

	auto& lodScratchIndices = _activeSet.getLodScratchIndices();
	lodScratchIndices.clear();
	_lodTree.traverse(viewMatrix, cam->getProjectionMatrix(), cam->getViewport(), lodScratchIndices);

	const auto& activeIndices = _activeSet.getActiveIndices();
	bool changed = lodScratchIndices != activeIndices;
	if (changed) {
		const int LOD_STABLE_FRAME_COUNT = 2;
		if (!_activeSet.stageLOD(lodScratchIndices, LOD_STABLE_FRAME_COUNT))
			return false;

		std::vector<int> previousDepthIndex = _activeSet.commitPendingLOD(_originalPointCount);
		updateActiveIndexBuffer();
		if (_gpuResources.indexTexture() && _gpuResources.indexTexture()->getImage())
			updateIndexImageChangedRanges(_gpuResources.indexTexture()->getImage(), previousDepthIndex);
		return true;
	} else {
		_activeSet.clearPendingLOD();
	}
	return false;
}

bool GaussianDrawObj::updateRadNativeLOD(const osg::Matrix& viewMatrix)
{
	if (!_sparkDrawable.isRadFile())
		return false;

	return _sparkDrawable.updateRadNativeLOD(viewMatrix, camera());
}

bool GaussianDrawObj::applySortResult(osg::Image* indexImage)
{
	if (_sparkDrawable.isRadFile()) {
		return _sparkDrawable.applySortResult(indexImage);
	}

	if (_sortPass.useGpuSort() || _sortPass.sortDisabled())
		return false;

	double sortMilliseconds = 0.0;
	SparkSortPass::CpuResultStatus resultStatus = _sortPass.fetchCpuResult(_activeSet.getSortResultScratch(), _activeSet.getActiveSetVersion(), sortMilliseconds);
	if (resultStatus == SparkSortPass::CpuResultStatus::None)
		return false;
	if (resultStatus == SparkSortPass::CpuResultStatus::Stale) {
		return false;
	}

	std::vector<int> previousDepthIndex = _activeSet.applyCpuSortResultWithPrevious(sortMilliseconds);
	updateIndexImageChangedRanges(indexImage, previousDepthIndex);
	return true;
}

void GaussianDrawObj::updateDataImage(osg::Image* paramsImage)
{
	_gpuResources.updateDataRange(gaussianPoints, 0, (int)gaussianPoints.size(), _sparkDrawable.isRadFile());
}

void GaussianDrawObj::updateDataImageRange(osg::Image* paramsImage, int firstSlot, int endSlot)
{
	_gpuResources.updateDataRange(gaussianPoints, firstSlot, endSlot, _sparkDrawable.isRadFile());
}

void GaussianDrawObj::updateSHImage(osg::Image* paramsImage)
{
	_gpuResources.updateSHRange(gaussianPoints, 0, (int)gaussianPoints.size());
}

void GaussianDrawObj::updateSHImageRange(osg::Image* paramsImage, int firstSlot, int endSlot)
{
	_gpuResources.updateSHRange(gaussianPoints, firstSlot, endSlot);
}

void GaussianDrawObj::updateIndexImage(osg::Image* paramsImage)
{
	_gpuResources.updateIndexRange(_activeSet.getDepthIndex(), 0, (int)_activeSet.getDepthIndex().size());
}

void GaussianDrawObj::updateIndexImageRange(osg::Image* paramsImage, int firstIndex, int endIndex)
{
	_gpuResources.updateIndexRange(_activeSet.getDepthIndex(), firstIndex, endIndex);
}

void GaussianDrawObj::updateIndexImageChangedRanges(osg::Image* paramsImage, const std::vector<int>& previousIndices)
{
	// In Spark Paged Pool mode, the index image is fully managed by SparkOsgDrawable
	// and we should not overwrite it here with local _activeSet.getDepthIndex().
	if (_sparkDrawable.isRadFile() && useRadSparkPath(true))
		return;

	_gpuResources.updateIndexChangedRanges(_activeSet.getDepthIndex(), previousIndices);
}

void GaussianDrawObj::ensureIndexTextureCapacity(int requiredIndices)
{
	_gpuResources.ensureIndexCapacity(_activeSet.getDepthIndex(), requiredIndices);
}

void GaussianDrawObj::updateActiveIndexBuffer()
{
	_gpuResources.updateActiveIndexBuffer(_activeSet.getActiveIndices(), _sortPass.gpuSortCapacity());
}

void GaussianDrawObj::loadShader(osg::StateSet* ss)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	const auto& strDir = _shaderDirectory;
	osg::Shader* vs = SparkShaderLibrary::readShaderTextFile(osg::Shader::VERTEX, strDir + "gaussian.vert");
	osg::Shader* fs = SparkShaderLibrary::readShaderTextFile(osg::Shader::FRAGMENT, strDir + "gaussian.frag");

	// gaussian.vert references these helpers unconditionally; insert them even when the
	// runtime uniforms disable the optional features so GLSL compilation succeeds.
	if (vs) {
		std::string splatEditSource = SparkShaderLibrary::readFileToString(strDir + "splat_edit.glsl");
		if (!splatEditSource.empty())
			vs->setShaderSource(SparkShaderLibrary::insertShaderPrelude(vs->getShaderSource(), splatEditSource));
	}

	if (vs) {
		std::string skinningSource = SparkShaderLibrary::readFileToString(strDir + "splat_skinning.glsl");
		if (!skinningSource.empty())
			vs->setShaderSource(SparkShaderLibrary::insertShaderPrelude(vs->getShaderSource(), skinningSource));
	}

	if (vs) program->addShader(vs);
	if (fs) program->addShader(fs);
	ss->setAttribute(program);
}

void GaussianDrawObj::loadComputeShader()
{
	const auto& strDir = _shaderDirectory;
	osg::Shader* cs = SparkShaderLibrary::readShaderTextFile(osg::Shader::COMPUTE, strDir + "gaussian_depth.comp");
	if (cs) {
		_depthComputeProgram = new osg::Program;
		_depthComputeProgram->addShader(cs);
	}

	osg::Shader* sortCs = SparkShaderLibrary::readShaderTextFile(osg::Shader::COMPUTE, strDir + "gaussian_sort.comp");
	if (sortCs) {
		_sortComputeProgram = new osg::Program;
		_sortComputeProgram->addShader(sortCs);
	}

	_tileBinner.loadShaders(strDir);
	_tileRasterPass.loadShader(strDir);
	_compositePass.loadShader(strDir);
}

void GaussianDrawObj::createGpuSortBuffers()
{
	int requestedActiveCapacity = _sparkDrawable.isRadFile()
		? SparkRadQualitySettings::fromEnvironment().capacity()
		: _originalPointCount;

	_sortPass.configureSort(requestedActiveCapacity, _sparkDrawable.isRadFile());

	int capacity = _sortPass.gpuSortCapacity();
	_gpuResources.createSortBuffers(capacity, _activeSet.getActiveIndices());

	if (_tileBinner.isEnabled()) {
		_tileBinner.initialize(_gpuResources);
	}
}

osg::ref_ptr<osg::Node> GaussianDrawObj::createComputeDispatchNode(bool enableCovSplats)
{
	SparkGpuResources& gpu = _sparkDrawable.isRadFile() ? _sparkDrawable.gpuResources() : _gpuResources;
	SparkSortPass& sort = _sparkDrawable.isRadFile() ? _sparkDrawable.sortPass() : _sortPass;
	int currentDrawActiveCount = getGpuSortDrawCount();

	bool needGpuSort = sort.useGpuSort() && _depthComputeProgram && _sortComputeProgram;
	bool enableTileRaster = isEnvEnabled("OSG_3DGS_ENABLE_TILE_RASTER");
	bool needTilePath = enableTileRaster && _tileBinner.isEnabled() && _tileBinner.hasShaders() && needGpuSort;

	if (!needGpuSort && !needTilePath)
		return nullptr;

	osg::ref_ptr<osg::Group> group = new osg::Group;

	// GPU sort path: depth key generation + bitonic sort
	if (needGpuSort) {
		osg::ref_ptr<osg::Geode> depthGeode = new osg::Geode;
		osg::ref_ptr<osg::DispatchCompute> depthDispatch = new osg::DispatchCompute(1, 1, 1);
		depthDispatch->setCullingActive(false);
		depthGeode->addDrawable(depthDispatch.get());

		osg::StateSet* depthSS = depthGeode->getOrCreateStateSet();
		depthSS->setAttribute(_depthComputeProgram.get());
		depthSS->setUpdateCallback(new ComputeSSCallback(this, depthDispatch.get()));
		depthSS->addUniform(new osg::Uniform("activeCount", currentDrawActiveCount));
		depthSS->addUniform(new osg::Uniform("sortCount", sort.gpuSortCapacity()));
		depthSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "view"));
		depthSS->addUniform(new osg::Uniform("indexBuffer", 0));
		depthSS->addUniform(new osg::Uniform("dataBuffer", 1));
		depthSS->addUniform(new osg::Uniform("packedDataBuffer", 3));
		depthSS->addUniform(new osg::Uniform("usePackedEncoding", static_cast<int>(gpu.splatEncoding())));
		depthSS->addUniform(new osg::Uniform("sortRadial", sort.sortRadial()));
		if (gpu.indexTexture())
			depthSS->setTextureAttribute(0, gpu.indexTexture());
		if (gpu.dataTexture())
			depthSS->setTextureAttribute(1, gpu.dataTexture());
		if (gpu.packedDataTexture())
			depthSS->setTextureAttribute(3, gpu.packedDataTexture());
		if (gpu.activeIndexBuffer())
			depthSS->setAttribute(new osg::ShaderStorageBufferBinding(0, gpu.activeIndexBuffer()));
		if (gpu.depthKeyBuffer())
			depthSS->setAttribute(new osg::ShaderStorageBufferBinding(1, gpu.depthKeyBuffer()));
		if (gpu.gpuSortIndexBuffer())
			depthSS->setAttribute(new osg::ShaderStorageBufferBinding(2, gpu.gpuSortIndexBuffer()));
		_depthComputeDispatch = depthDispatch;
		group->addChild(depthGeode.get());

		osg::ref_ptr<osg::Geode> depthBarrier = new osg::Geode;
		depthBarrier->addDrawable(new MemoryBarrierDrawable(GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT));
		group->addChild(depthBarrier.get());

		int gpuSortCapacity = sort.gpuSortCapacity();
		for (int stage = 2; stage <= gpuSortCapacity; stage <<= 1) {
			for (int pass = stage >> 1; pass > 0; pass >>= 1) {
				osg::ref_ptr<osg::Geode> sortGeode = new osg::Geode;
				osg::ref_ptr<osg::DispatchCompute> sortDispatch = new osg::DispatchCompute(1, 1, 1);
				sortDispatch->setCullingActive(false);
				sortGeode->addDrawable(sortDispatch.get());

				osg::StateSet* sortSS = sortGeode->getOrCreateStateSet();
				sortSS->setAttribute(_sortComputeProgram.get());
				sortSS->setUpdateCallback(new ComputeSSCallback(this, sortDispatch.get()));
				sortSS->addUniform(new osg::Uniform("activeCount", currentDrawActiveCount));
				sortSS->addUniform(new osg::Uniform("sortCount", gpuSortCapacity));
				sortSS->addUniform(new osg::Uniform("sortStage", stage));
				sortSS->addUniform(new osg::Uniform("sortPass", pass));
				sortSS->addUniform(new osg::Uniform("sortAscending", needTilePath)); // Ascending (Front-to-Back) for Tile Raster, Descending (Back-to-Front) for legacy
				if (gpu.depthKeyBuffer())
					sortSS->setAttribute(new osg::ShaderStorageBufferBinding(1, gpu.depthKeyBuffer()));
				if (gpu.gpuSortIndexBuffer())
					sortSS->setAttribute(new osg::ShaderStorageBufferBinding(2, gpu.gpuSortIndexBuffer()));
				group->addChild(sortGeode.get());

				osg::ref_ptr<osg::Geode> sortBarrier = new osg::Geode;
				sortBarrier->addDrawable(new MemoryBarrierDrawable(GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT));
				group->addChild(sortBarrier.get());
			}
		}
	}

	// Tile binning + raster path (independent of GPU sort)
	if (needTilePath) {
		osg::Camera* camera = this->camera();
		if (camera) {
			// Create/update output image for tile raster
			osg::Viewport* viewport = camera->getViewport();
			int width = viewport ? (int)viewport->width() : 1;
			int height = viewport ? (int)viewport->height() : 1;
			if (width > 0 && height > 0)
				_tileRasterPass.createOutputImage(width, height);

			osg::ref_ptr<osg::Node> tileBinNode = _tileBinner.createDispatchNode(
				gpu, *camera, currentDrawActiveCount,
				static_cast<int>(gpu.splatEncoding()),
				getEnvFloat("OSG_3DGS_SUPPORT_RADIUS_SCALE", radAwareDefault(_sparkDrawable.isRadFile(), 1.0f, 0.9f), _sparkDrawable.isRadFile() ? 0.25f : 0.6f, 1.4f),
				getEnvFloat("OSG_3DGS_MAX_SCREEN_RADIUS", radAwareDefault(_sparkDrawable.isRadFile(), 512.0f, 40.0f), _sparkDrawable.isRadFile() ? 6.0f : 16.0f, 2048.0f),
				getEnvFloat("OSG_3DGS_CLIP_XY", _sparkDrawable.isRadFile() ? 1.4f : 2.0f, 1.0f, 10.0f),
				[this]() {
					SparkSortPass& currentSort = _sparkDrawable.isRadFile() ? _sparkDrawable.sortPass() : _sortPass;
					return std::min(getActiveCount(), currentSort.gpuSortCapacity());
				});
			if (tileBinNode)
				group->addChild(tileBinNode.get());

			bool enableBlurCompensation = radDefaultBool("OSG_3DGS_ENABLE_BLUR_COMPENSATION", "OSG_3DGS_DISABLE_BLUR_COMPENSATION", _sparkDrawable.isRadFile());
			bool sparkRadiusClamp = radDefaultBool("OSG_3DGS_SPARK_RADIUS_CLAMP", "OSG_3DGS_DISABLE_SPARK_RADIUS_CLAMP", _sparkDrawable.isRadFile());
			bool lodInflate = radDefaultBool("OSG_3DGS_LOD_INFLATE", "OSG_3DGS_DISABLE_LOD_INFLATE", false);
			float supportRadiusScale = getEnvFloat("OSG_3DGS_SUPPORT_RADIUS_SCALE", radAwareDefault(_sparkDrawable.isRadFile(), 1.0f, 0.9f), _sparkDrawable.isRadFile() ? 0.25f : 0.6f, 1.4f);
			float maxScreenRadius = getEnvFloat("OSG_3DGS_MAX_SCREEN_RADIUS", radAwareDefault(_sparkDrawable.isRadFile(), 512.0f, 40.0f), _sparkDrawable.isRadFile() ? 6.0f : 16.0f, 2048.0f);
			float preBlurAmount = getEnvFloat("OSG_3DGS_PRE_BLUR_AMOUNT", 0.0f, 0.0f, 16.0f);
			float blurAmount = getEnvFloat("OSG_3DGS_BLUR_AMOUNT", radAwareDefault(_sparkDrawable.isRadFile(), 0.3f, 0.3f), 0.0f, 16.0f);
			if (_sparkDrawable.isRadFile())
				logRadRenderControls(enableBlurCompensation, sparkRadiusClamp, lodInflate,
				                     supportRadiusScale, maxScreenRadius, preBlurAmount, blurAmount);

			const bool outputSRGB = isEnvEnabled("OSG_3DGS_OUTPUT_SRGB");
			osg::ref_ptr<osg::Node> tileRasterNode = _tileRasterPass.createDispatchNode(
				gpu, *camera, _tileBinner, hasSH, SparkRenderParams{
					getEnvFloat("OSG_3DGS_MIN_ALPHA", 0.5f / 255.0f, 0.0f, 1.0f),
					getEnvFloat("OSG_3DGS_FALLOFF", 1.0f, 0.0f, 1.0f),
					supportRadiusScale,
					maxScreenRadius,
					getEnvFloat("OSG_3DGS_MIN_PIXEL_RADIUS", 0.0f, 0.0f, 16.0f),
					preBlurAmount,
					blurAmount,
					enableBlurCompensation,
					sparkRadiusClamp,
					lodInflate,
					isEnvEnabled("OSG_3DGS_ENABLE_2DGS"),
					false,  // isOrthographic
					getEnvFloat("OSG_3DGS_FOCAL_DISTANCE", 0.0f, 0.0f, 10000.0f),
					getEnvFloat("OSG_3DGS_APERTURE_ANGLE", 0.0f, 0.0f, 3.14159f),
					enableCovSplats,
					getEnvFloat("OSG_3DGS_CLIP_XY", _sparkDrawable.isRadFile() ? 1.4f : 2.0f, 1.0f, 10.0f),
					outputSRGB,
					!_sparkDrawable.isRadFile(), // isColorPremultiplied
					hasSH ? (_sparkDrawable.isRadFile() ? _sparkDrawable.radFileInfo().maxSh : 3) : 0,
                    getEnvInt("OSG_3DGS_DEBUG_FLAG", 0, 0, 2)
				});
			if (tileRasterNode)
				group->addChild(tileRasterNode.get());

			// Composite pass: render tile output to framebuffer
			if (_compositePass.hasShader() && _tileRasterPass.outputImage()) {
				osg::ref_ptr<osg::Node> compositeNode = _compositePass.createCompositeNode(
					_tileRasterPass.outputImage(), outputSRGB);
				if (compositeNode)
					group->addChild(compositeNode.get());
			}
		}
	}

	return group;
}

osg::ref_ptr<osg::Node> GaussianDrawObj::createNode(osg::Camera& camera)
{
	_camera = &camera;
	SparkGpuResources& gpu = _sparkDrawable.isRadFile() ? _sparkDrawable.gpuResources() : _gpuResources;
	SparkSortPass& sort = _sparkDrawable.isRadFile() ? _sparkDrawable.sortPass() : _sortPass;
	const std::vector<int>& currentIndex = _sparkDrawable.isRadFile() ? _sparkDrawable.depthIndex() : _activeSet.getDepthIndex();
	int currentDrawActiveCount = getGpuSortDrawCount();

	osg::ref_ptr<osg::Group> root = new osg::Group;

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> pInstance = createQuadGeometry(currentDrawActiveCount);
	if (supportsPointEditing()) {
		_editableNode = root;
		if (currentDrawActiveCount == 0) root->setNodeMask(0);
		_editableQuad = pInstance;
		// Only the supplied model bounds matter; quad vertices are a GPU template.
		pInstance->setComputeBoundingBoxCallback(new osg::Drawable::ComputeBoundingBoxCallback);
	}
	geode->addDrawable(pInstance.get());

	gpu.setRadMode(_sparkDrawable.isRadFile());

	const bool useSparkPagedPool = gpu.hasPagedPool();
	int packedMode = getPackedEncodingMode(useSparkPagedPool);
	bool enableCovSplats = isEnvEnabled("OSG_3DGS_ENABLE_COVSPLATS");
	gpu.setSplatEncoding(static_cast<SplatEncoding>(packedMode));

	// Match Spark's radial alpha ordering for RAD. Explicit 0 keeps the
	// previous view-depth ordering for scenes that prefer it.
	sort.setSortRadial(isEnvSet("OSG_3DGS_SORT_RADIAL")
		? isEnvEnabled("OSG_3DGS_SORT_RADIAL") : _sparkDrawable.isRadFile());
	if (_sparkDrawable.isRadFile())
		std::cout << "RAD alpha sort metric: " << (sort.sortRadial() ? "radial" : "view-depth") << std::endl;
	float defaultMinSortIntervalMs = _sparkDrawable.isRadFile() ? 8.0f : 0.0f;
	sort.setMinSortIntervalMs(getEnvFloat("OSG_3DGS_MIN_SORT_INTERVAL_MS", defaultMinSortIntervalMs, 0.0f, 1000.0f));

	// Initialize or bind GPU resources. In paged-pool mode these resources were already
	// created up-front and must not be recreated here, otherwise CPU page state diverges
	// from the GPU buffers.
	if (useSparkPagedPool) {
		gpu.updateIndexRange(currentIndex, 0, (int)currentIndex.size());
		if (gpu.indexTexture())
			pInstance->getOrCreateStateSet()->setTextureAttribute(0, gpu.indexTexture());
		if (gpu.indexTexture())
			gpu.indexTexture()->setUpdateCallback(new IndexBufferUpdateCallback(this, pInstance.get()));

		if (gpu.packedDataTexture()) {
			gpu.updatePackedDataRange(gaussianPoints, 0, (int)gaussianPoints.size(), _sparkDrawable.isRadFile());
			pInstance->getOrCreateStateSet()->setTextureAttribute(3, gpu.packedDataTexture());
		}

		if (hasSH) {
			if (!gpu.shTexture())
				gpu.createSHTexture(gaussianPoints, std::max(1, gpu.maxPages() * gpu.pageSize()));
			else
				gpu.updateSHRange(gaussianPoints, 0, (int)gaussianPoints.size());
			if (gpu.shTexture())
				pInstance->getOrCreateStateSet()->setTextureAttribute(2, gpu.shTexture());
		}

		if (enableCovSplats) {
			if (!gpu.covSplatTexture())
				gpu.createCovSplatTexture(std::max(1, gpu.maxPages() * gpu.pageSize()));
			if (!gaussianPoints.empty())
				gpu.updateCovSplatRange(gaussianPoints, 0, (int)gaussianPoints.size());
			if (gpu.covSplatTexture())
				pInstance->getOrCreateStateSet()->setTextureAttribute(4, gpu.covSplatTexture());
		}
	} else {
		// Index buffer TBO
		{
			int pointCapacity = _sparkDrawable.isRadFile() ? nextPowerOfTwo(std::max(1, currentDrawActiveCount)) : nNum;
			gpu.createIndexTexture(currentIndex, pointCapacity);
			pInstance->getOrCreateStateSet()->setTextureAttribute(0, gpu.indexTexture());
			if (gpu.indexTexture())
				gpu.indexTexture()->setUpdateCallback(new IndexBufferUpdateCallback(this, pInstance.get()));
		}

		// Data buffer TBO (4 Vec4f per splat: pos+alpha, scale+r, quat.xyz+g, quat.w+b)
		{
			int pointCapacity = _sparkDrawable.isRadFile() ? nextPowerOfTwo(std::max(1, (int)gaussianPoints.size())) : (int)gaussianPoints.size();
			gpu.createDataTexture(gaussianPoints, pointCapacity, _sparkDrawable.isRadFile());
			pInstance->getOrCreateStateSet()->setTextureAttribute(1, gpu.dataTexture());
		}

		// Packed data buffer TBO (RGBA32UI, for PackedSplat/ExtSplat encoding)
		if (packedMode > 0)
		{
			int pointCapacity = _sparkDrawable.isRadFile() ? nextPowerOfTwo(std::max(1, (int)gaussianPoints.size())) : (int)gaussianPoints.size();
			gpu.createPackedDataTexture(pointCapacity);
			gpu.updatePackedDataRange(gaussianPoints, 0, (int)gaussianPoints.size(), _sparkDrawable.isRadFile());
			pInstance->getOrCreateStateSet()->setTextureAttribute(3, gpu.packedDataTexture());
		}

		// SH buffer TBO
		if (hasSH)
		{
			int pointCapacity = _sparkDrawable.isRadFile() ? nextPowerOfTwo(std::max(1, (int)gaussianPoints.size())) : (int)gaussianPoints.size();
			gpu.createSHTexture(gaussianPoints, pointCapacity);
			pInstance->getOrCreateStateSet()->setTextureAttribute(2, gpu.shTexture());
		}

		// CovSplat buffer TBO (texture unit 4)
		if (enableCovSplats)
		{
			int pointCapacity = _sparkDrawable.isRadFile() ? nextPowerOfTwo(std::max(1, (int)gaussianPoints.size())) : (int)gaussianPoints.size();
			gpu.createCovSplatTexture(pointCapacity);
			gpu.updateCovSplatRange(gaussianPoints, 0, (int)gaussianPoints.size());
			pInstance->getOrCreateStateSet()->setTextureAttribute(4, gpu.covSplatTexture());
		}
	}

	if (_sparkDrawable.isRadFile()) {
		std::cout << "RAD initial GPU capacities: index=" << gpu.indexCapacity()
		          << ", data=" << gpu.dataCapacity()
		          << ", packed=" << gpu.packedDataCapacity()
		          << ", sh=" << gpu.shCapacity()
		          << ", covSplat=" << gpu.covSplatCapacity()
		          << ", residentPoints=" << gaussianPoints.size()
		          << ", residentCapacity=" << _sparkDrawable._radResidentCapacity
		          << ", enableCovSplats=" << (enableCovSplats ? 1 : 0)
		          << std::endl;
	}

	// State setup
	osg::StateSet* ss = pInstance->getOrCreateStateSet();
	loadShader(ss);
	ss->setUpdateCallback(new SSCallback(this));

	ss->addUniform(new osg::Uniform("indexBuffer", 0));
	ss->addUniform(new osg::Uniform("dataBuffer", 1));
	ss->addUniform(new osg::Uniform("shBuffer", 2));
	ss->addUniform(new osg::Uniform("packedDataBuffer", 3));
	ss->addUniform(new osg::Uniform("covSplatBuffer", 4));
	ss->addUniform(new osg::Uniform("usePackedEncoding", packedMode));
	ss->addUniform(new osg::Uniform("useSparkPagedPool", useSparkPagedPool));
	ss->addUniform(new osg::Uniform("useGpuSort", sort.useGpuSort()));
	ss->addUniform(new osg::Uniform("hasSH", hasSH));
	ss->addUniform(new osg::Uniform("globalShOrder", hasSH ? _globalShOrder : 0));
	bool enableBlurCompensation = radDefaultBool("OSG_3DGS_ENABLE_BLUR_COMPENSATION", "OSG_3DGS_DISABLE_BLUR_COMPENSATION", _sparkDrawable.isRadFile());
	bool sparkRadiusClamp = radDefaultBool("OSG_3DGS_SPARK_RADIUS_CLAMP", "OSG_3DGS_DISABLE_SPARK_RADIUS_CLAMP", _sparkDrawable.isRadFile());
	bool lodInflate = radDefaultBool("OSG_3DGS_LOD_INFLATE", "OSG_3DGS_DISABLE_LOD_INFLATE", false);
	float supportRadiusScale = getEnvFloat("OSG_3DGS_SUPPORT_RADIUS_SCALE", radAwareDefault(_sparkDrawable.isRadFile(), 1.0f, 0.9f), _sparkDrawable.isRadFile() ? 0.25f : 0.6f, 1.4f);
	float maxScreenRadius = getEnvFloat("OSG_3DGS_MAX_SCREEN_RADIUS", radAwareDefault(_sparkDrawable.isRadFile(), 512.0f, 40.0f), _sparkDrawable.isRadFile() ? 6.0f : 16.0f, 2048.0f);
	float preBlurAmount = getEnvFloat("OSG_3DGS_PRE_BLUR_AMOUNT", 0.0f, 0.0f, 16.0f);
	float blurAmount = getEnvFloat("OSG_3DGS_BLUR_AMOUNT", radAwareDefault(_sparkDrawable.isRadFile(), 0.3f, 0.3f), 0.0f, 16.0f);
	if (_sparkDrawable.isRadFile())
	{
		logRadRenderControls(enableBlurCompensation, sparkRadiusClamp, lodInflate,
			supportRadiusScale, maxScreenRadius, preBlurAmount, blurAmount);
	}

	ss->addUniform(new osg::Uniform("supportRadiusScale", supportRadiusScale));
	ss->addUniform(new osg::Uniform("maxScreenRadius", maxScreenRadius));
	ss->addUniform(new osg::Uniform("minPixelRadius", getEnvFloat("OSG_3DGS_MIN_PIXEL_RADIUS", 0.0f, 0.0f, 16.0f)));
	ss->addUniform(new osg::Uniform("preBlurAmount", preBlurAmount));
	ss->addUniform(new osg::Uniform("blurAmount", blurAmount));
	ss->addUniform(new osg::Uniform("minAlpha", getEnvFloat("OSG_3DGS_MIN_ALPHA", 0.5f / 255.0f, 0.0f, 1.0f)));
	ss->addUniform(new osg::Uniform("falloff", getEnvFloat("OSG_3DGS_FALLOFF", 1.0f, 0.0f, 1.0f)));
	ss->addUniform(new osg::Uniform("enableBlurCompensation", enableBlurCompensation));
	ss->addUniform(new osg::Uniform("sparkRadiusClamp", sparkRadiusClamp));
	ss->addUniform(new osg::Uniform("lodInflate", lodInflate));
	ss->addUniform(new osg::Uniform("enable2DGS", isEnvEnabled("OSG_3DGS_ENABLE_2DGS")));
	ss->addUniform(new osg::Uniform("isOrthographic", false));
	ss->addUniform(new osg::Uniform("focalDistance", getEnvFloat("OSG_3DGS_FOCAL_DISTANCE", 0.0f, 0.0f, 10000.0f)));
	ss->addUniform(new osg::Uniform("apertureAngle", getEnvFloat("OSG_3DGS_APERTURE_ANGLE", 0.0f, 0.0f, 3.14159f)));
	ss->addUniform(new osg::Uniform("enableCovSplats", enableCovSplats));
	ss->addUniform(new osg::Uniform("clipXY", getEnvFloat("OSG_3DGS_CLIP_XY", _sparkDrawable.isRadFile() ? 1.4f : 2.0f, 1.0f, 10.0f)));
	ss->addUniform(new osg::Uniform("debugFlag", getEnvInt("OSG_3DGS_DEBUG_FLAG", 0, 0, 2)));
	const bool outputSRGB = isEnvEnabled("OSG_3DGS_OUTPUT_SRGB");
	ss->addUniform(new osg::Uniform("outputSRGB", outputSRGB));
	ss->addUniform(new osg::Uniform("isColorPremultiplied", !_sparkDrawable.isRadFile()));

	// Splat Edit integration
	bool enableSplatEdit = isEnvEnabled("OSG_3DGS_ENABLE_SPLAT_EDIT");
	ss->addUniform(new osg::Uniform("enableSplatEdit", enableSplatEdit));
	if (enableSplatEdit) {
		// Create a default SparkSplatEdits instance and set up the SDF texture
		SparkSplatEdits splatEdits;
		splatEdits.update();
		osg::Image* sdfImage = splatEdits.sdfTextureImage();

		int texUnit = 5; // Use texture unit 5 (after covSplatBuffer at 4)
		if (sdfImage) {
			osg::ref_ptr<osg::Texture2D> sdfTex = new osg::Texture2D;
			sdfTex->setImage(sdfImage);
			sdfTex->setSourceFormat(GL_RGBA_INTEGER);
			sdfTex->setSourceType(GL_UNSIGNED_INT);
			sdfTex->setInternalFormat(0x8D70); // GL_RGBA32UI
			sdfTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP_TO_BORDER);
			sdfTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP_TO_BORDER);
			sdfTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
			sdfTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
			ss->setTextureAttribute(texUnit, sdfTex.get());
		}
		ss->addUniform(new osg::Uniform("sdfTexture", texUnit));
		ss->addUniform(new osg::Uniform("numSdfs", splatEdits.numSdfs()));
		ss->addUniform(new osg::Uniform("numEdits", splatEdits.numEdits()));
		// Set edit data uniforms (all zeros for default)
		// Use vec4 with float bits of uint32 values for GLSL floatBitsToUint()
		osg::Uniform* editDataUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "editData", 8);
		for (int i = 0; i < 8; ++i) {
			editDataUniform->setElement(i, osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		}
		ss->addUniform(editDataUniform);

		std::cout << "Splat Edit enabled (numSdfs=" << splatEdits.numSdfs()
		          << ", numEdits=" << splatEdits.numEdits() << ")" << std::endl;
	} else {
		ss->addUniform(new osg::Uniform("sdfTexture", 5));
		ss->addUniform(new osg::Uniform("numSdfs", 0));
		ss->addUniform(new osg::Uniform("numEdits", 0));
		osg::Uniform* editDataUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "editData", 8);
		for (int i = 0; i < 8; ++i) {
			editDataUniform->setElement(i, osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
		}
		ss->addUniform(editDataUniform);
	}

	// Splat Skinning integration
	bool enableSkinning = isEnvEnabled("OSG_3DGS_ENABLE_SKINNING");
	int skinningModeInt = getEnvInt("OSG_3DGS_SKINNING_MODE", 0, 0, 1);  // 0=DQ, 1=LB
	ss->addUniform(new osg::Uniform("enableSkinning", enableSkinning));
	ss->addUniform(new osg::Uniform("skinningMode", skinningModeInt));
	if (enableSkinning) {
		// Create default skinning instance (users should set data via API)
		// Texture units: skinSampler=6, boneSampler=7 (after covSplat=4, sdfTexture=5)
		int skinTexUnit = 6;
		int boneTexUnit = 7;
		ss->addUniform(new osg::Uniform("skinSampler", skinTexUnit));
		ss->addUniform(new osg::Uniform("boneSampler", boneTexUnit));
		ss->addUniform(new osg::Uniform("skinningNumSplats", 0));
		ss->addUniform(new osg::Uniform("skinningNumBones", 0));

		std::cout << "Splat Skinning enabled (mode=" << (skinningModeInt == 0 ? "DQ" : "LB")
		          << ")" << std::endl;
	} else {
		ss->addUniform(new osg::Uniform("skinSampler", 6));
		ss->addUniform(new osg::Uniform("boneSampler", 7));
		ss->addUniform(new osg::Uniform("skinningNumSplats", 0));
		ss->addUniform(new osg::Uniform("skinningNumBones", 0));
	}

	// Portal integration
	bool enablePortal = isEnvEnabled("OSG_3DGS_ENABLE_PORTAL");
	ss->addUniform(new osg::Uniform("enablePortal", enablePortal));
	ss->addUniform(new osg::Uniform("diskCenter", osg::Vec3f(0.0f, 0.0f, 0.0f)));
	ss->addUniform(new osg::Uniform("diskNormal", osg::Vec3f(0.0f, 0.0f, -1.0f)));
	ss->addUniform(new osg::Uniform("diskRadius", 0.0f));
	ss->addUniform(new osg::Uniform("diskTwoSided", false));
	if (enablePortal) {
		std::cout << "Portal enabled (disk-based spatial clipping)" << std::endl;
	}

	// RAD keeps streamed/merged nodes above the initial resident count on DC
	if (gpu.gpuSortIndexBuffer())
		ss->setAttribute(new osg::ShaderStorageBufferBinding(2, gpu.gpuSortIndexBuffer()));

	// Disable depth test/write
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	// Premultiplied alpha blending: GL_ONE, GL_ONE_MINUS_SRC_ALPHA
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
	blendFunc->setFunction(osg::BlendFunc::ONE, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
	ss->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);
	osg::ref_ptr<osg::BlendEquation> blendEquation = new osg::BlendEquation(osg::BlendEquation::FUNC_ADD);
	ss->setAttributeAndModes(blendEquation, osg::StateAttribute::ON);
	ss->setMode(GL_FRAMEBUFFER_SRGB, outputSRGB ? osg::StateAttribute::ON : osg::StateAttribute::OFF);

	// Register pre-draw callback for automatic sorting
	_sortCallback = new GaussianSortCallback(this);
	camera.addPreDrawCallback(_sortCallback);

	osg::ref_ptr<osg::Node> computeDispatch = createComputeDispatchNode(enableCovSplats);
	if (computeDispatch) {
		root->addChild(computeDispatch.get());
	}
	// GPU sort fallback: if GPU sort was requested but depth/sort shaders
	// are missing, the GPU sort portion was not created.  Fall back to CPU.
	if (sort.useGpuSort() && (!_depthComputeProgram || !_sortComputeProgram)) {
		std::cerr << "WARNING: GPU sort requested but compute shader(s) missing; "
		          << "falling back to CPU sort" << std::endl;
		sort.fallbackToCpu();
		osg::Uniform* useGpuSortUniform = ss->getUniform("useGpuSort");
		if (useGpuSortUniform)
			useGpuSortUniform->set(sort.useGpuSort());
	}

	bool tilePathActive = isEnvEnabled("OSG_3DGS_ENABLE_TILE_RASTER") && _tileBinner.isEnabled() && _tileBinner.hasShaders() && sort.useGpuSort() && _depthComputeProgram && _sortComputeProgram;
	if (!tilePathActive)
		root->addChild(geode.get());
	return root;
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readSplatFile(const std::string& filename)
{
	SparkDatasetLoadResult result = SparkDatasetLoader::readSplatFile(filename);
	if (result.bounds.valid())
		bounds.expandBy(result.bounds);
	return std::move(result.points);
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readRadFile(const std::string& filename)
{
	SparkRadDatasetLoader::LoadResult result;
	SparkRadDatasetLoader::LoadConfig config;
	config.maxResident = getEnvInt("OSG_3DGS_RAD_MAX_RESIDENT", DEFAULT_RAD_MAX_RESIDENT, 65536, 8388608);
	config.initialChunks = getEnvInt("OSG_3DGS_RAD_INITIAL_CHUNKS", 1, 1, 200000000);
	config.disableSh = isEnvEnabled("OSG_3DGS_RAD_DISABLE_SH");
	if (!SparkRadDatasetLoader::readRadFile(filename, config, result))
		return std::vector<MI_GaussianPoint>();
	_focusBounds = result.focusBounds;

	if (result.bounds.valid())
		bounds.expandBy(result.bounds);

	_sparkDrawable._radFilename = filename;
	_sparkDrawable._radChunkDataBase = result.chunkDataBase;
	_sparkDrawable.radFileInfo() = std::move(result.fileInfo);
	_sparkDrawable.radChunks() = std::move(result.chunks);
	_sparkDrawable._radChunkSize = _sparkDrawable.radFileInfo().chunkSize;
	_sparkDrawable._radChunkSlots = std::move(result.chunkSlots);
	_sparkDrawable._radSlotToChunk = std::move(result.slotToChunk);
	_sparkDrawable._radFreeSlots.clear();
	_sparkDrawable._radResidentCapacity = result.residentCapacity;
	_sparkDrawable._isRadFile = result.isRadFile;
	_sparkDrawable._radTotalPointCount = result.totalPointCount;
	_sparkDrawable._radTotalChunkCount = result.totalChunkCount;
	_sparkDrawable._radLoadedChunkCount = result.loadedChunkCount;
	_sparkDrawable._radEvictedChunkCount = result.evictedChunkCount;
	_sparkDrawable._radFrameNumber = result.frameNumber;
	_sparkDrawable._radChildCounts = std::move(result.childCounts);
	_sparkDrawable._radChildStarts = std::move(result.childStarts);
	_sparkDrawable._radLocalToGlobal = std::move(result.localToGlobal);
	_sparkDrawable._radGlobalToLocal = std::move(result.globalToLocal);
	_sparkDrawable._radLeafGlobalIndices = std::move(result.leafGlobalIndices);
	_sparkDrawable._radLodSizes = std::move(result.lodSizes);
	_sparkDrawable._radLodNodes.resize(result.points.size());
	for (size_t i = 0; i < result.points.size(); ++i) {
		const MI_GaussianPoint& point = result.points[i];
		const float size = i < _sparkDrawable._radLodSizes.size()
			? _sparkDrawable._radLodSizes[i]
			: std::max(point.scale.x(), std::max(point.scale.y(), point.scale.z()));
		_sparkDrawable._radLodNodes[i] = { point.position, point.scale, point.color.a(), size };
	}
	_sparkDrawable._radHasLodOpacity = result.hasLodOpacity;
	_sparkDrawable._radChunkSlots.resize(_sparkDrawable.radChunks().size());
	this->hasSH = result.hasSH;
	_globalShOrder = result.globalShOrder;
	if (_sparkDrawable.radFileInfo().maxSh > 0 && !this->hasSH)
		std::cout << "RAD SH disabled by OSG_3DGS_RAD_DISABLE_SH" << std::endl;
	std::cout << "Loaded " << result.points.size() << " resident RAD LoD nodes from Spark RAD ("
	          << result.loadedChunkCount << "/" << _sparkDrawable.radChunks().size() << " chunks, totalCount=" << result.totalPointCount
	          << ", maxSh=" << _sparkDrawable.radFileInfo().maxSh << ", childStarts=" << _sparkDrawable._radChildStarts.size() << ")" << std::endl;
	return std::move(result.points);
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readPlyFile(const std::string& ply_path)
{
	SparkDatasetLoadResult result = SparkDatasetLoader::readPlyFile(ply_path);
	if (result.bounds.valid())
		bounds.expandBy(result.bounds);
	hasSH = result.hasSH;
	_globalShOrder = result.globalShOrder;
	return std::move(result.points);
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readWithGaussForge(
	const std::string& filename, const std::string& ext)
{
	SparkDatasetLoadResult result = SparkDatasetLoader::readWithGaussForge(filename, ext);
	if (result.bounds.valid())
		bounds.expandBy(result.bounds);
	hasSH = result.hasSH;
	_globalShOrder = result.globalShOrder;
	return std::move(result.points);
}

osg::ref_ptr<osg::Geometry> GaussianDrawObj::createQuadGeometry(int instanceCount)
{
	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
	(*vertices)[0].set( 1.0f,  1.0f, 0.0f);
	(*vertices)[1].set(-1.0f,  1.0f, 0.0f);
	(*vertices)[2].set( 1.0f, -1.0f, 0.0f);
	(*vertices)[3].set(-1.0f, -1.0f, 0.0f);
	quad->setVertexArray(vertices);

	osg::ref_ptr<osg::DrawElementsUShort> de =
		new osg::DrawElementsUShort(GL_TRIANGLES);
	de->push_back(0); de->push_back(1); de->push_back(2);
	de->push_back(1); de->push_back(3); de->push_back(2);
	de->setNumInstances(instanceCount);
	quad->addPrimitiveSet(de);

	quad->setCullingActive(false);
	if (bounds.valid())
		quad->setInitialBound(bounds);
	quad->setUseDisplayList(false);
	quad->setUseVertexBufferObjects(true);
	return quad;
}
