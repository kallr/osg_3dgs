#include "SparkCompositePass.h"
#include "SparkShaderLibrary.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/StateSet>
#include <osg/Shader>
#include <osg/BlendFunc>

#ifndef GL_FRAMEBUFFER_SRGB
#define GL_FRAMEBUFFER_SRGB 0x8DB9
#endif

void SparkCompositePass::loadShader(const std::string& shaderDir)
{
    osg::Shader* vs = SparkShaderLibrary::readShaderTextFile(osg::Shader::VERTEX, shaderDir + "gaussian_composite.vert");
    osg::Shader* fs = SparkShaderLibrary::readShaderTextFile(osg::Shader::FRAGMENT, shaderDir + "gaussian_composite.frag");

    if (vs && fs) {
        _compositeProgram = new osg::Program;
        _compositeProgram->addShader(vs);
        _compositeProgram->addShader(fs);
    }
}

osg::ref_ptr<osg::Node> SparkCompositePass::createCompositeNode(osg::Image* inputImage, bool outputSRGB)
{
    if (!_compositeProgram || !inputImage)
        return nullptr;

    // Create fullscreen quad geometry
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;

    // Vertices: fullscreen quad in clip space (-1 to 1)
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->push_back(osg::Vec3(-1.0f, -1.0f, 0.0f));
    vertices->push_back(osg::Vec3( 1.0f, -1.0f, 0.0f));
    vertices->push_back(osg::Vec3( 1.0f,  1.0f, 0.0f));
    vertices->push_back(osg::Vec3(-1.0f,  1.0f, 0.0f));
    quad->setVertexArray(vertices.get());

    // Tex coords
    osg::ref_ptr<osg::Vec2Array> texcoords = new osg::Vec2Array;
    texcoords->push_back(osg::Vec2(0.0f, 0.0f));
    texcoords->push_back(osg::Vec2(1.0f, 0.0f));
    texcoords->push_back(osg::Vec2(1.0f, 1.0f));
    texcoords->push_back(osg::Vec2(0.0f, 1.0f));
    quad->setTexCoordArray(0, texcoords.get());

    // Indices
    osg::ref_ptr<osg::DrawElementsUShort> indices = new osg::DrawElementsUShort(GL_TRIANGLES);
    indices->push_back(0); indices->push_back(1); indices->push_back(2);
    indices->push_back(0); indices->push_back(2); indices->push_back(3);
    quad->addPrimitiveSet(indices.get());

    geode->addDrawable(quad.get());

    // State set
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setAttribute(_compositeProgram.get());

    // Bind input texture
    osg::ref_ptr<osg::Texture2D> inputTex = new osg::Texture2D;
    inputTex->setImage(inputImage);
    inputTex->setSourceFormat(GL_RGBA);
    inputTex->setSourceType(GL_FLOAT);
    inputTex->setInternalFormat(GL_RGBA32F_ARB);
    inputTex->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::NEAREST);
    inputTex->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::NEAREST);
    inputTex->setWrap(osg::Texture2D::WRAP_S, osg::Texture2D::CLAMP_TO_EDGE);
    inputTex->setWrap(osg::Texture2D::WRAP_T, osg::Texture2D::CLAMP_TO_EDGE);
    ss->setTextureAttribute(0, inputTex.get());
    ss->addUniform(new osg::Uniform("tileOutput", 0));

    // Premultiplied alpha blending — tile raster outputs (color*alpha, alpha)
    ss->setAttributeAndModes(new osg::BlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA));
    ss->setMode(GL_FRAMEBUFFER_SRGB, outputSRGB ? osg::StateAttribute::ON : osg::StateAttribute::OFF);

    // Disable depth test - composite goes on top
    ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    return geode;
}
