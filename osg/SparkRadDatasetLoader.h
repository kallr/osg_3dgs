#ifndef SPARK_RAD_DATASET_LOADER_H
#define SPARK_RAD_DATASET_LOADER_H

#include "RadTypes.h"

#include <osg/BoundingBox>

#include <ios>
#include <string>
#include <vector>

namespace SparkRadDatasetLoader
{
    struct LoadConfig
    {
        int maxResident = 1048576;
        int initialChunks = 1;
        bool disableSh = false;
    };

    struct LoadResult
    {
        std::vector<MI_GaussianPoint> points;
        osg::BoundingBox bounds;
        osg::BoundingBox focusBounds;
        RadFileInfo fileInfo;
        std::vector<RadChunkInfo> chunks;
        std::streamoff chunkDataBase = 0;
        int totalPointCount = 0;
        int totalChunkCount = 0;
        int residentCapacity = 0;
        int loadedChunkCount = 0;
        int evictedChunkCount = 0;
        unsigned int frameNumber = 0;
        bool isRadFile = false;
        bool hasSH = false;
        int globalShOrder = 0;
        bool hasLodOpacity = false;
        std::vector<uint16_t> childCounts;
        std::vector<uint32_t> childStarts;
        std::vector<uint32_t> localToGlobal;
        std::vector<int> globalToLocal;
        std::vector<uint32_t> leafGlobalIndices;
        std::vector<float> lodSizes;
        std::vector<std::vector<int>> chunkSlots;
        std::vector<int> slotToChunk;
    };

    bool readRadFile(const std::string& filename, const LoadConfig& config, LoadResult& result);
}

#endif
