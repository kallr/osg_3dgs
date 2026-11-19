#include "SparkTileBinner.h"
#include "SparkGpuResources.h"
#include "SparkShaderLibrary.h"
#include "SparkOsgCallbacks.h"

#include <osg/Geode>
#include <osg/DispatchCompute>
#include <osg/Uniform>
#include <osg/BufferIndexBinding>
#include <osg/StateSet>
#include <osg/Camera>
#include <osg/Viewport>

#include <algorithm>
#include <functional>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <iterator>
#include <string>

static bool getEnvBool(const char* name, bool def = false) {
    if (const char* env_p = std::getenv(name)) {
        return std::string(env_p) == "1";
    }
    return def;
}

static int getEnvInt(const char* name, int def, int minVal, int maxVal) {
    if (const char* env_p = std::getenv(name)) {
        int val = std::atoi(env_p);
        return std::max(minVal, std::min(val, maxVal));
    }
    return def;
}

namespace
{

class TileClearSSCallback : public osg::StateSet::Callback
{
public:
    TileClearSSCallback(int tileCount, int tileListCapacity, osg::DispatchCompute* dispatch)
        : _tileCount(tileCount), _tileListCapacity(tileListCapacity), _dispatch(dispatch) {}

    virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
    {
        if (!ss) return;
        if (!_tileCountUniform) {
            _tileCountUniform = ss->getOrCreateUniform("tileCount", osg::Uniform::INT);
            _tileListCapacityUniform = ss->getOrCreateUniform("tileListCapacity", osg::Uniform::INT);
            _dispatchGroupsXUniform = ss->getOrCreateUniform("dispatchGroupsX", osg::Uniform::INT);
        }
        _tileCountUniform->set(_tileCount);
        _tileListCapacityUniform->set(_tileListCapacity);
        if (_dispatch) {
            int workCount = std::max(_tileCount, _tileListCapacity);
            int totalGroups = std::max(1, (workCount + 255) / 256);
            // 使用 2D dispatch 避免 groupsX 超出 GL_MAX_COMPUTE_WORK_GROUP_COUNT (65535)
            int groupsX = std::min(totalGroups, 65535);
            int groupsY = (totalGroups + groupsX - 1) / groupsX;
            _dispatch->setComputeGroups(groupsX, groupsY, 1);
            _dispatchGroupsXUniform->set(groupsX);
        }
    }

private:
    int _tileCount;
    int _tileListCapacity;
    osg::DispatchCompute* _dispatch;
    osg::Uniform* _tileCountUniform = nullptr;
    osg::Uniform* _tileListCapacityUniform = nullptr;
    osg::Uniform* _dispatchGroupsXUniform = nullptr;
};

class TileBinSSCallback : public osg::StateSet::Callback
{
public:
    TileBinSSCallback(osg::Camera& camera, int tileCountX, int tileCountY, int tileSize,
                      int tileListCapacity, int tileListPerTileCapacity,
                      osg::DispatchCompute* dispatch,
                      const std::function<int()>& activeCountProvider = std::function<int()>())
        : _camera(&camera), _tileCountX(tileCountX), _tileCountY(tileCountY),
          _tileSize(tileSize), _tileListCapacity(tileListCapacity),
          _tileListPerTileCapacity(tileListPerTileCapacity), _dispatch(dispatch),
          _activeCountProvider(activeCountProvider) {}

    virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
    {
        if (!ss) return;
        osg::Camera* camera = _camera.get();
        if (!camera) return;

        if (!_viewUniform) {
            _viewUniform = ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4);
            _projUniform = ss->getOrCreateUniform("proj", osg::Uniform::FLOAT_MAT4);
            _viewportUniform = ss->getOrCreateUniform("viewport_size", osg::Uniform::FLOAT_VEC2);
            _activeCountUniform = ss->getOrCreateUniform("activeCount", osg::Uniform::INT);
            _tileCountXUniform = ss->getOrCreateUniform("tileCountX", osg::Uniform::INT);
            _tileCountYUniform = ss->getOrCreateUniform("tileCountY", osg::Uniform::INT);
            _tileSizeUniform = ss->getOrCreateUniform("tileSize", osg::Uniform::INT);
            _tileListCapacityUniform = ss->getOrCreateUniform("tileListCapacity", osg::Uniform::INT);
            _tileListPerTileCapacityUniform = ss->getOrCreateUniform("tileListPerTileCapacity", osg::Uniform::INT);
            _dispatchGroupsXUniform = ss->getOrCreateUniform("dispatchGroupsX", osg::Uniform::INT);
        }

        osg::Matrixf viewMatrix = camera->getViewMatrix();
        osg::Matrixf projMatrix = camera->getProjectionMatrix();
        _viewUniform->set(viewMatrix);
        _projUniform->set(projMatrix);

        osg::Vec2 vs;
        osg::Viewport* viewport = camera->getViewport();
        vs[0] = viewport ? viewport->width() : 1.0f;
        vs[1] = viewport ? viewport->height() : 1.0f;
        _viewportUniform->set(vs);

        if (_activeCountProvider)
            _activeCount = std::max(0, _activeCountProvider());

        // activeCount may change after LOD/streaming updates.
        _activeCountUniform->set(_activeCount);
        _tileCountXUniform->set(_tileCountX);
        _tileCountYUniform->set(_tileCountY);
        _tileSizeUniform->set(_tileSize);
        _tileListCapacityUniform->set(_tileListCapacity);
        _tileListPerTileCapacityUniform->set(_tileListPerTileCapacity);

        if (_dispatch) {
            int totalGroups = std::max(1, (_activeCount + 255) / 256);
            int groupsX = std::min(totalGroups, 65535);
            int groupsY = (totalGroups + groupsX - 1) / groupsX;
            _dispatch->setComputeGroups(groupsX, groupsY, 1);
            _dispatchGroupsXUniform->set(groupsX);
        }
    }

    void setActiveCount(int count) { _activeCount = count; }

private:
    osg::observer_ptr<osg::Camera> _camera;
    int _activeCount = 0;
    int _tileCountX;
    int _tileCountY;
    int _tileSize;
    int _tileListCapacity;
    int _tileListPerTileCapacity;
    osg::DispatchCompute* _dispatch;
    std::function<int()> _activeCountProvider;
    osg::Uniform* _viewUniform = nullptr;
    osg::Uniform* _projUniform = nullptr;
    osg::Uniform* _viewportUniform = nullptr;
    osg::Uniform* _activeCountUniform = nullptr;
    osg::Uniform* _tileCountXUniform = nullptr;
    osg::Uniform* _tileCountYUniform = nullptr;
    osg::Uniform* _tileSizeUniform = nullptr;
    osg::Uniform* _tileListCapacityUniform = nullptr;
    osg::Uniform* _tileListPerTileCapacityUniform = nullptr;
    osg::Uniform* _dispatchGroupsXUniform = nullptr;
};

} // anonymous namespace

void SparkTileBinner::loadShaders(const std::string& shaderDir)
{
    if (!_enabled) return;

    osg::Shader* tileClearCs = SparkShaderLibrary::readShaderTextFile(osg::Shader::COMPUTE, shaderDir + "gaussian_tile_clear.comp");
    if (tileClearCs) {
        _tileClearProgram = new osg::Program;
        _tileClearProgram->addShader(tileClearCs);
    }

    osg::Shader* tileBinCs = SparkShaderLibrary::readShaderTextFile(osg::Shader::COMPUTE, shaderDir + "gaussian_tile_bin.comp");
    if (tileBinCs) {
        std::string splatEditSource = SparkShaderLibrary::readFileToString(shaderDir + "splat_edit.glsl");
        if (!splatEditSource.empty())
            tileBinCs->setShaderSource(SparkShaderLibrary::insertShaderPrelude(tileBinCs->getShaderSource(), splatEditSource));
        _tileBinProgram = new osg::Program;
        _tileBinProgram->addShader(tileBinCs);
    }
}

void SparkTileBinner::initialize(SparkGpuResources& gpu)
{
    _tileSize = TILE_SIZE;
    _tileCountX = MAX_TILE_COUNT_X;
    _tileCountY = MAX_TILE_COUNT_Y;
    _tileCount = MAX_TILE_COUNT;
    _tileListCapacity = _tileCount * TILE_LIST_PER_TILE_CAPACITY;
    gpu.createTileBuffers(_tileCount, _tileListCapacity);

    std::cout << "Tile binning: tileSize=" << _tileSize
              << ", maxTiles=" << _tileCountX << "x" << _tileCountY
              << ", listCapacity=" << _tileListCapacity
              << ", listPerTile=" << TILE_LIST_PER_TILE_CAPACITY << std::endl;
}

osg::ref_ptr<osg::Node> SparkTileBinner::createDispatchNode(
    SparkGpuResources& gpu,
    osg::Camera& camera,
    int activeCount,
    int packedEncoding,
    float supportRadiusScale,
    float maxScreenRadius,
    float clipXY,
    const std::function<int()>& activeCountProvider)
{
    if (!hasShaders() || !gpu.tileCountBuffer())
        return nullptr;

    osg::ref_ptr<osg::Group> group = new osg::Group;

    // Tile clear pass
    {
        osg::ref_ptr<osg::Geode> tileClearGeode = new osg::Geode;
        osg::ref_ptr<osg::DispatchCompute> tileClearDispatch = new osg::DispatchCompute(1, 1, 1);
        tileClearDispatch->setCullingActive(false);
        tileClearGeode->addDrawable(tileClearDispatch.get());

        osg::StateSet* tileClearSS = tileClearGeode->getOrCreateStateSet();
        tileClearSS->setAttribute(_tileClearProgram.get());
        tileClearSS->setUpdateCallback(new TileClearSSCallback(_tileCount, _tileListCapacity, tileClearDispatch.get()));
        tileClearSS->addUniform(new osg::Uniform("tileCount", _tileCount));
        tileClearSS->addUniform(new osg::Uniform("tileListCapacity", _tileListCapacity));
        tileClearSS->setAttribute(new osg::ShaderStorageBufferBinding(3, gpu.tileCountBuffer()));
        if (gpu.tileCursorBuffer())
            tileClearSS->setAttribute(new osg::ShaderStorageBufferBinding(4, gpu.tileCursorBuffer()));
        if (gpu.tileListBuffer())
            tileClearSS->setAttribute(new osg::ShaderStorageBufferBinding(5, gpu.tileListBuffer()));
        if (gpu.tileStatsBuffer())
            tileClearSS->setAttribute(new osg::ShaderStorageBufferBinding(6, gpu.tileStatsBuffer()));
        group->addChild(tileClearGeode.get());

        osg::ref_ptr<osg::Geode> tileClearBarrier = new osg::Geode;
        tileClearBarrier->addDrawable(new MemoryBarrierDrawable(GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT));
        group->addChild(tileClearBarrier.get());
    }

    // Tile bin pass
    {
        osg::ref_ptr<osg::Geode> tileBinGeode = new osg::Geode;
        osg::ref_ptr<osg::DispatchCompute> tileBinDispatch = new osg::DispatchCompute(1, 1, 1);
        tileBinDispatch->setCullingActive(false);
        tileBinGeode->addDrawable(tileBinDispatch.get());

        TileBinSSCallback* binCallback = new TileBinSSCallback(
            camera, _tileCountX, _tileCountY, _tileSize, _tileListCapacity,
            TILE_LIST_PER_TILE_CAPACITY, tileBinDispatch.get(), activeCountProvider);
        binCallback->setActiveCount(activeCount);

        osg::StateSet* tileBinSS = tileBinGeode->getOrCreateStateSet();
        tileBinSS->setAttribute(_tileBinProgram.get());
        tileBinSS->setUpdateCallback(binCallback);
        tileBinSS->addUniform(new osg::Uniform("dataBuffer", 1));
        tileBinSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "view"));
        tileBinSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "proj"));
        tileBinSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC2, "viewport_size"));
        tileBinSS->addUniform(new osg::Uniform("activeCount", activeCount));
        tileBinSS->addUniform(new osg::Uniform("tileCountX", _tileCountX));
        tileBinSS->addUniform(new osg::Uniform("tileCountY", _tileCountY));
        tileBinSS->addUniform(new osg::Uniform("tileSize", _tileSize));
        tileBinSS->addUniform(new osg::Uniform("tileListCapacity", _tileListCapacity));
        tileBinSS->addUniform(new osg::Uniform("tileListPerTileCapacity", TILE_LIST_PER_TILE_CAPACITY));
        tileBinSS->addUniform(new osg::Uniform("packedDataBuffer", 3));
        tileBinSS->addUniform(new osg::Uniform("usePackedEncoding", packedEncoding));
        tileBinSS->addUniform(new osg::Uniform("supportRadiusScale", supportRadiusScale));
        tileBinSS->addUniform(new osg::Uniform("maxScreenRadius", maxScreenRadius));
        tileBinSS->addUniform(new osg::Uniform("clipXY", clipXY));
        tileBinSS->addUniform(new osg::Uniform("useSparkPagedPool", gpu.hasPagedPool()));
        bool enableSplatEdit = getEnvBool("OSG_3DGS_ENABLE_SPLAT_EDIT");
        tileBinSS->addUniform(new osg::Uniform("enableSplatEdit", enableSplatEdit));
        tileBinSS->addUniform(new osg::Uniform("sdfTexture", 5));
        tileBinSS->addUniform(new osg::Uniform("numSdfs", 0));
        tileBinSS->addUniform(new osg::Uniform("numEdits", 0));
        osg::ref_ptr<osg::Uniform> editDataUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "editData", 8);
        osg::Vec4 zeroEdit(0.0f, 0.0f, 0.0f, 0.0f);
        for (int i = 0; i < 8; ++i)
            editDataUniform->setElement(i, zeroEdit);
        tileBinSS->addUniform(editDataUniform.get());
        if (gpu.dataTexture())
            tileBinSS->setTextureAttribute(1, gpu.dataTexture());
        if (gpu.packedDataTexture())
            tileBinSS->setTextureAttribute(3, gpu.packedDataTexture());
        if (gpu.gpuSortIndexBuffer())
            tileBinSS->setAttribute(new osg::ShaderStorageBufferBinding(2, gpu.gpuSortIndexBuffer()));
        tileBinSS->setAttribute(new osg::ShaderStorageBufferBinding(3, gpu.tileCountBuffer()));
        if (gpu.tileCursorBuffer())
            tileBinSS->setAttribute(new osg::ShaderStorageBufferBinding(4, gpu.tileCursorBuffer()));
        if (gpu.tileListBuffer())
            tileBinSS->setAttribute(new osg::ShaderStorageBufferBinding(5, gpu.tileListBuffer()));
        if (gpu.tileStatsBuffer())
            tileBinSS->setAttribute(new osg::ShaderStorageBufferBinding(6, gpu.tileStatsBuffer()));
        group->addChild(tileBinGeode.get());

        osg::ref_ptr<osg::Geode> tileBinBarrier = new osg::Geode;
        tileBinBarrier->addDrawable(new MemoryBarrierDrawable(GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT));
        group->addChild(tileBinBarrier.get());
    }

    return group;
}
