#include "SparkTileRasterPass.h"
#include "SparkTileBinner.h"
#include "SparkGpuResources.h"
#include "SparkShaderLibrary.h"
#include "SparkOsgCallbacks.h"

#include <osg/Geode>
#include <osg/DispatchCompute>
#include <osg/Uniform>
#include <osg/BufferIndexBinding>
#include <osg/BindImageTexture>
#include <osg/StateSet>
#include <osg/Camera>
#include <osg/Viewport>
#include <osg/Texture2D>

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <iostream>

#ifndef GL_WRITE_ONLY
#define GL_WRITE_ONLY 0x88B9
#endif

namespace
{
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

class TileRasterSSCallback : public osg::StateSet::Callback
{
public:
    TileRasterSSCallback(osg::Camera& camera, int tileCount, int tileListPerTileCapacity,
                         int tileSize, int tileCountX,
                         int viewportWidth, int viewportHeight,
                         bool hasSH, int globalShOrder,
                         osg::DispatchCompute* dispatch)
        : _camera(&camera), _tileCount(tileCount), _tileListPerTileCapacity(tileListPerTileCapacity),
          _tileSize(tileSize), _tileCountX(tileCountX),
          _viewportWidth(viewportWidth), _viewportHeight(viewportHeight),
          _hasSH(hasSH), _globalShOrder(globalShOrder), _dispatch(dispatch) {}

    virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
    {
        if (!ss) return;

        if (!_tileCountUniform) {
            _tileCountUniform = ss->getOrCreateUniform("tileCount", osg::Uniform::INT);
            _tileListPerTileCapacityUniform = ss->getOrCreateUniform("tileListPerTileCapacity", osg::Uniform::INT);
            _tileSizeUniform = ss->getOrCreateUniform("tileSize", osg::Uniform::INT);
            _tileCountXUniform = ss->getOrCreateUniform("tileCountX", osg::Uniform::INT);
            _viewportWidthUniform = ss->getOrCreateUniform("viewport_width", osg::Uniform::INT);
            _viewportHeightUniform = ss->getOrCreateUniform("viewport_height", osg::Uniform::INT);
            _hasSHUniform = ss->getOrCreateUniform("hasSH", osg::Uniform::BOOL);
            _globalShOrderUniform = ss->getOrCreateUniform("globalShOrder", osg::Uniform::INT);
            _viewUniform = ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4);
            _projUniform = ss->getOrCreateUniform("proj", osg::Uniform::FLOAT_MAT4);
            _viewportSizeUniform = ss->getOrCreateUniform("viewport_size", osg::Uniform::FLOAT_VEC2);
        }

        _tileCountUniform->set(_tileCount);
        _tileListPerTileCapacityUniform->set(_tileListPerTileCapacity);
        _tileSizeUniform->set(_tileSize);
        _tileCountXUniform->set(_tileCountX);

        int viewportWidth = _viewportWidth;
        int viewportHeight = _viewportHeight;
        osg::Vec2 vs((float)viewportWidth, (float)viewportHeight);

        // Update view/proj/viewport from camera each frame
        osg::Camera* camera = _camera.get();
        if (camera) {
            _viewUniform->set(camera->getViewMatrix());
            _projUniform->set(camera->getProjectionMatrix());
            osg::Viewport* viewport = camera->getViewport();
            viewportWidth = viewport ? (int)viewport->width() : viewportWidth;
            viewportHeight = viewport ? (int)viewport->height() : viewportHeight;
            vs.set((float)viewportWidth, (float)viewportHeight);
        }
        _viewportWidthUniform->set(viewportWidth);
        _viewportHeightUniform->set(viewportHeight);
        _viewportSizeUniform->set(vs);
        _hasSHUniform->set(_hasSH);
        _globalShOrderUniform->set(_hasSH ? _globalShOrder : 0);

        if (_dispatch) {
            // 使用 2D dispatch 避免 groupsX 超出 GL_MAX_COMPUTE_WORK_GROUP_COUNT (65535)
            // tile 自然组织为 tileCountX × tileCountY 网格
            int groupsX = std::max(1, _tileCountX);
            int groupsY = (_tileCount + groupsX - 1) / groupsX;
            _dispatch->setComputeGroups(groupsX, groupsY, 1);
        }
    }

private:
    osg::observer_ptr<osg::Camera> _camera;
    int _tileCount;
    int _tileListPerTileCapacity;
    int _tileSize;
    int _tileCountX;
    int _viewportWidth;
    int _viewportHeight;
    bool _hasSH;
    int _globalShOrder;
    osg::DispatchCompute* _dispatch;
    osg::Uniform* _tileCountUniform = nullptr;
    osg::Uniform* _tileListPerTileCapacityUniform = nullptr;
    osg::Uniform* _tileSizeUniform = nullptr;
    osg::Uniform* _tileCountXUniform = nullptr;
    osg::Uniform* _viewportWidthUniform = nullptr;
    osg::Uniform* _viewportHeightUniform = nullptr;
    osg::Uniform* _hasSHUniform = nullptr;
    osg::Uniform* _globalShOrderUniform = nullptr;
    osg::Uniform* _viewUniform = nullptr;
    osg::Uniform* _projUniform = nullptr;
    osg::Uniform* _viewportSizeUniform = nullptr;
};

} // anonymous namespace

void SparkTileRasterPass::loadShader(const std::string& shaderDir)
{
    osg::Shader* tileRasterCs = SparkShaderLibrary::readShaderTextFile(osg::Shader::COMPUTE, shaderDir + "gaussian_tile_raster.comp");
    if (tileRasterCs) {
        // gaussian_tile_raster.comp references these helpers unconditionally; insert
        // them even when the runtime uniforms disable the optional features.
        std::string splatEditSource = SparkShaderLibrary::readFileToString(shaderDir + "splat_edit.glsl");
        if (!splatEditSource.empty())
            tileRasterCs->setShaderSource(SparkShaderLibrary::insertShaderPrelude(tileRasterCs->getShaderSource(), splatEditSource));

        std::string skinningSource = SparkShaderLibrary::readFileToString(shaderDir + "splat_skinning.glsl");
        if (!skinningSource.empty())
            tileRasterCs->setShaderSource(SparkShaderLibrary::insertShaderPrelude(tileRasterCs->getShaderSource(), skinningSource));

        _tileRasterProgram = new osg::Program;
        _tileRasterProgram->addShader(tileRasterCs);
    }
}

void SparkTileRasterPass::createOutputImage(int width, int height)
{
    if (width <= 0 || height <= 0)
        return;

    if (_outputImage.valid() && _outputWidth == width && _outputHeight == height)
        return;

    _outputImage = new osg::Image;
    _outputImage->allocateImage(width, height, 1, GL_RGBA, GL_FLOAT);
    std::memset(_outputImage->data(), 0, _outputImage->getTotalSizeInBytesIncludingMipmaps());
    _outputWidth = width;
    _outputHeight = height;
}

osg::ref_ptr<osg::Node> SparkTileRasterPass::createDispatchNode(
    SparkGpuResources& gpu,
    osg::Camera& camera,
    const SparkTileBinner& binner,
    bool hasSH,
    const RenderParams& params)
{
    if (!_tileRasterProgram.valid() || !_outputImage.valid())
        return nullptr;

    osg::ref_ptr<osg::Group> group = new osg::Group;

    int tileCount = binner.tileCount();
    int tileListPerTileCapacity = binner.tileListPerTileCapacity();

    osg::ref_ptr<osg::Geode> tileRasterGeode = new osg::Geode;
    osg::ref_ptr<osg::DispatchCompute> tileRasterDispatch = new osg::DispatchCompute(1, 1, 1);
    tileRasterDispatch->setCullingActive(false);
    tileRasterGeode->addDrawable(tileRasterDispatch.get());

    osg::StateSet* tileRasterSS = tileRasterGeode->getOrCreateStateSet();
    tileRasterSS->setAttribute(_tileRasterProgram.get());
    tileRasterSS->setUpdateCallback(new TileRasterSSCallback(
        camera, tileCount, tileListPerTileCapacity, binner.tileSize(), binner.tileCountX(),
        _outputWidth, _outputHeight, hasSH, params.globalShOrder, tileRasterDispatch.get()));
    tileRasterSS->addUniform(new osg::Uniform("tileCount", tileCount));
    tileRasterSS->addUniform(new osg::Uniform("tileListPerTileCapacity", tileListPerTileCapacity));
    tileRasterSS->addUniform(new osg::Uniform("tileSize", binner.tileSize()));
    tileRasterSS->addUniform(new osg::Uniform("tileCountX", binner.tileCountX()));
    tileRasterSS->addUniform(new osg::Uniform("viewport_width", _outputWidth));
    tileRasterSS->addUniform(new osg::Uniform("viewport_height", _outputHeight));
    tileRasterSS->addUniform(new osg::Uniform("dataBuffer", 1));
    tileRasterSS->addUniform(new osg::Uniform("shBuffer", 2));
    tileRasterSS->addUniform(new osg::Uniform("packedDataBuffer", 3));
    tileRasterSS->addUniform(new osg::Uniform("covSplatBuffer", 4));
    tileRasterSS->addUniform(new osg::Uniform("hasSH", hasSH));
    tileRasterSS->addUniform(new osg::Uniform("globalShOrder", params.globalShOrder));
    tileRasterSS->addUniform(new osg::Uniform("minAlpha", params.minAlpha));
    tileRasterSS->addUniform(new osg::Uniform("falloff", params.falloff));
    tileRasterSS->addUniform(new osg::Uniform("usePackedEncoding", (int)gpu.splatEncoding()));
    tileRasterSS->addUniform(new osg::Uniform("enableCovSplats", params.enableCovSplats));
    tileRasterSS->addUniform(new osg::Uniform("isOrthographic", params.isOrthographic));
    tileRasterSS->addUniform(new osg::Uniform("focalDistance", params.focalDistance));
    tileRasterSS->addUniform(new osg::Uniform("apertureAngle", params.apertureAngle));
    tileRasterSS->addUniform(new osg::Uniform("supportRadiusScale", params.supportRadiusScale));
    tileRasterSS->addUniform(new osg::Uniform("maxScreenRadius", params.maxScreenRadius));
    tileRasterSS->addUniform(new osg::Uniform("minPixelRadius", params.minPixelRadius));
    tileRasterSS->addUniform(new osg::Uniform("preBlurAmount", params.preBlurAmount));
    tileRasterSS->addUniform(new osg::Uniform("blurAmount", params.blurAmount));
    tileRasterSS->addUniform(new osg::Uniform("enableBlurCompensation", params.enableBlurCompensation));
    tileRasterSS->addUniform(new osg::Uniform("sparkRadiusClamp", params.sparkRadiusClamp));
    tileRasterSS->addUniform(new osg::Uniform("lodInflate", params.lodInflate));
    tileRasterSS->addUniform(new osg::Uniform("enable2DGS", params.enable2DGS));
    tileRasterSS->addUniform(new osg::Uniform("clipXY", params.clipXY));
    tileRasterSS->addUniform(new osg::Uniform("outputSRGB", params.outputSRGB));
    tileRasterSS->addUniform(new osg::Uniform("isColorPremultiplied", params.isColorPremultiplied));
    tileRasterSS->addUniform(new osg::Uniform("debugFlag", params.debugFlag));
    
    // Pass Spark Paged Pool parameters to Tile Rasterizer
    tileRasterSS->addUniform(new osg::Uniform("useSparkPagedPool", gpu.hasPagedPool()));

    // Splat Edit uniforms for tile raster pass
    bool enableSplatEdit = isEnvEnabled("OSG_3DGS_ENABLE_SPLAT_EDIT");
    tileRasterSS->addUniform(new osg::Uniform("enableSplatEdit", enableSplatEdit));
    tileRasterSS->addUniform(new osg::Uniform("sdfTexture", 5));
    tileRasterSS->addUniform(new osg::Uniform("numSdfs", 0));
    tileRasterSS->addUniform(new osg::Uniform("numEdits", 0));
    {
        osg::Uniform* editDataUniform = new osg::Uniform(osg::Uniform::FLOAT_VEC4, "editData", 8);
        for (int i = 0; i < 8; ++i) {
            editDataUniform->setElement(i, osg::Vec4(0.0f, 0.0f, 0.0f, 0.0f));
        }
        tileRasterSS->addUniform(editDataUniform);
    }

    // Splat Skinning uniforms for tile raster pass
    bool enableSkinning = isEnvEnabled("OSG_3DGS_ENABLE_SKINNING");
    int skinningModeInt = getEnvInt("OSG_3DGS_SKINNING_MODE", 0, 0, 1);
    tileRasterSS->addUniform(new osg::Uniform("enableSkinning", enableSkinning));
    tileRasterSS->addUniform(new osg::Uniform("skinningMode", skinningModeInt));
    tileRasterSS->addUniform(new osg::Uniform("skinSampler", 6));
    tileRasterSS->addUniform(new osg::Uniform("boneSampler", 7));
    tileRasterSS->addUniform(new osg::Uniform("skinningNumSplats", 0));
    tileRasterSS->addUniform(new osg::Uniform("skinningNumBones", 0));

    tileRasterSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "view"));
    tileRasterSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_MAT4, "proj"));
    tileRasterSS->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC2, "viewport_size"));

    // Bind data and SH textures
    if (gpu.dataTexture())
        tileRasterSS->setTextureAttribute(1, gpu.dataTexture());
    if (gpu.shTexture())
        tileRasterSS->setTextureAttribute(2, gpu.shTexture());
    if (gpu.packedDataTexture())
        tileRasterSS->setTextureAttribute(3, gpu.packedDataTexture());
    if (gpu.covSplatTexture())
        tileRasterSS->setTextureAttribute(4, gpu.covSplatTexture());

    // Bind tile buffers
    tileRasterSS->setAttribute(new osg::ShaderStorageBufferBinding(3, gpu.tileCountBuffer()));
    if (gpu.tileListBuffer())
        tileRasterSS->setAttribute(new osg::ShaderStorageBufferBinding(5, gpu.tileListBuffer()));
    if (gpu.tileStatsBuffer())
        tileRasterSS->setAttribute(new osg::ShaderStorageBufferBinding(6, gpu.tileStatsBuffer()));

    // Bind output image
    osg::ref_ptr<osg::Texture2D> outputTex = new osg::Texture2D;
    outputTex->setImage(_outputImage.get());
    outputTex->setSourceFormat(GL_RGBA);
    outputTex->setSourceType(GL_FLOAT);
    outputTex->setInternalFormat(GL_RGBA32F_ARB);
    tileRasterSS->setTextureAttribute(7, outputTex.get());
    tileRasterSS->setAttribute(new osg::BindImageTexture(7, outputTex.get(), osg::BindImageTexture::WRITE_ONLY, GL_RGBA32F_ARB));

    group->addChild(tileRasterGeode.get());

    osg::ref_ptr<osg::Geode> tileRasterBarrier = new osg::Geode;
    tileRasterBarrier->addDrawable(new MemoryBarrierDrawable(
        GL_SHADER_STORAGE_BARRIER_BIT | GL_TEXTURE_FETCH_BARRIER_BIT));
    group->addChild(tileRasterBarrier.get());

    return group;
}
