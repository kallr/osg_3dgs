#ifndef SPARK_DATASET_LOADER_H
#define SPARK_DATASET_LOADER_H

#include "RadTypes.h"

#include <osg/BoundingBox>

#include <string>
#include <vector>

struct SparkDatasetLoadResult
{
    std::vector<MI_GaussianPoint> points;
    osg::BoundingBox bounds;
    bool hasSH = false;
    int globalShOrder = 0;
};

namespace SparkDatasetLoader
{
    SparkDatasetLoadResult readSplatFile(const std::string& filename);
    SparkDatasetLoadResult readPlyFile(const std::string& filename);
    SparkDatasetLoadResult readWithGaussForge(const std::string& filename, const std::string& ext);
}

#endif
