#ifndef SPARK_TILE_RASTER_PASS_H
#define SPARK_TILE_RASTER_PASS_H

#include "SparkRenderTypes.h"

#include <osg/ref_ptr>
#include <osg/Node>
#include <osg/Program>
#include <osg/Image>

class SparkGpuResources;
class SparkTileBinner;

// Manages tile-local rasterization: per-tile Gaussian evaluation and
// output image writing. Consumes tile lists produced by SparkTileBinner.
class SparkTileRasterPass
{
public:
    using RenderParams = SparkRenderParams;

    SparkTileRasterPass() = default;

    // Load tile raster compute shader from the given directory.
    void loadShader(const std::string& shaderDir);

    // Create the output image for tile raster output.
    // Must be called before createDispatchNode().
    void createOutputImage(int width, int height);

    // Create the OSG compute dispatch subgraph for tile raster.
    // Returns null if shaders failed to load or output image not created.
    // hasSH: whether spherical harmonics data is available.
    // params: rendering parameters to ensure consistency with main render path.
    osg::ref_ptr<osg::Node> createDispatchNode(
        SparkGpuResources& gpu,
        osg::Camera& camera,
        const SparkTileBinner& binner,
        bool hasSH,
        const RenderParams& params = RenderParams());

    osg::Image* outputImage() const { return _outputImage.get(); }

    bool hasShader() const { return _tileRasterProgram.valid(); }

private:
    osg::ref_ptr<osg::Program> _tileRasterProgram;
    osg::ref_ptr<osg::Image> _outputImage;
    int _outputWidth = 0;
    int _outputHeight = 0;
};

#endif
