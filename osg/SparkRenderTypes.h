#ifndef SPARK_RENDER_TYPES_H
#define SPARK_RENDER_TYPES_H

// Shared render parameters passed through Spark native renderer passes.
// Defaults mirror the previous SparkTileRasterPass::RenderParams values.
struct SparkRenderParams
{
    float minAlpha = 0.5f / 255.0f;
    float falloff = 1.0f;
    float supportRadiusScale = 0.85f;
    float maxScreenRadius = 32.0f;
    float minPixelRadius = 0.0f;
    float preBlurAmount = 0.0f;
    float blurAmount = 0.3f;
    bool enableBlurCompensation = true;
    bool sparkRadiusClamp = true;
    bool lodInflate = true;
    bool enable2DGS = false;
    bool isOrthographic = false;
    float focalDistance = 0.0f;
    float apertureAngle = 0.0f;
    bool enableCovSplats = false;
    float clipXY = 2.0f;
    bool outputSRGB = false;
    bool isColorPremultiplied = false;
    int globalShOrder = 0;
    int debugFlag = 0;
};

#endif
