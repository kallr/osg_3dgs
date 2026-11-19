#ifndef SPARK_COMPOSITE_PASS_H
#define SPARK_COMPOSITE_PASS_H

#include <osg/ref_ptr>
#include <osg/Node>
#include <osg/Image>
#include <osg/Program>

// Composites the tile raster output image into the OSG framebuffer.
// Draws a fullscreen quad with the tile raster output texture.
class SparkCompositePass
{
public:
    SparkCompositePass() = default;

    // Load composite vertex/fragment shaders from the given directory.
    void loadShader(const std::string& shaderDir);

    // Create the OSG subgraph for compositing.
    // inputImage: the RGBA32F output image from SparkTileRasterPass.
    // Returns a node that renders a fullscreen quad with the input texture.
    osg::ref_ptr<osg::Node> createCompositeNode(osg::Image* inputImage, bool outputSRGB);

    bool hasShader() const { return _compositeProgram.valid(); }

private:
    osg::ref_ptr<osg::Program> _compositeProgram;
};

#endif
