#ifndef SPARK_TILE_BINNER_H
#define SPARK_TILE_BINNER_H

#include <osg/ref_ptr>
#include <osg/Node>
#include <osg/Program>
#include <osg/StateSet>

#include <functional>

class SparkGpuResources;

// Insert a prelude (e.g. shared GLSL functions) immediately before `void main`
// in the given shader source. Used by tile bin / tile raster compute shaders.
std::string insertShaderPrelude(const std::string& source, const std::string& prelude);

// Manages tile binning configuration and OSG compute dispatch nodes.
// Tile binning scatters sorted splats into per-tile lists for tile-local raster.
class SparkTileBinner
{
public:
    // Tile configuration constants
    static const int TILE_SIZE = 16;
    static const int MAX_TILE_COUNT_X = 256;
    static const int MAX_TILE_COUNT_Y = 256;
    static const int MAX_TILE_COUNT = MAX_TILE_COUNT_X * MAX_TILE_COUNT_Y;
    static const int TILE_LIST_PER_TILE_CAPACITY = 1024; // Large RAD splats can overlap many dense tiles

    SparkTileBinner() = default;

    // Load tile compute shaders from the given directory.
    void loadShaders(const std::string& shaderDir);

    // Initialize tile buffers and configuration.
    void initialize(SparkGpuResources& gpu);

    // Create the OSG compute dispatch subgraph for tile clear + tile bin.
    // Returns null if tile binning is not configured or shaders failed to load.
    // The callback reads the supplied camera's view, projection and viewport.
    osg::ref_ptr<osg::Node> createDispatchNode(
        SparkGpuResources& gpu,
        osg::Camera& camera,
        int activeCount,
        int packedEncoding = 0,
        float supportRadiusScale = 0.85f,
        float maxScreenRadius = 32.0f,
        float clipXY = 2.0f,
        const std::function<int()>& activeCountProvider = std::function<int()>());

    bool isEnabled() const { return _enabled; }
    void setEnabled(bool enabled) { _enabled = enabled; }

    int tileSize() const { return _tileSize; }
    int tileCountX() const { return _tileCountX; }
    int tileCountY() const { return _tileCountY; }
    int tileCount() const { return _tileCount; }
    int tileListCapacity() const { return _tileListCapacity; }
    int tileListPerTileCapacity() const { return TILE_LIST_PER_TILE_CAPACITY; }

    bool hasShaders() const { return _tileClearProgram.valid() && _tileBinProgram.valid(); }

private:
    bool _enabled = false;
    int _tileSize = TILE_SIZE;
    int _tileCountX = MAX_TILE_COUNT_X;
    int _tileCountY = MAX_TILE_COUNT_Y;
    int _tileCount = MAX_TILE_COUNT;
    int _tileListCapacity = MAX_TILE_COUNT * TILE_LIST_PER_TILE_CAPACITY;

    osg::ref_ptr<osg::Program> _tileClearProgram;
    osg::ref_ptr<osg::Program> _tileBinProgram;
};

#endif
