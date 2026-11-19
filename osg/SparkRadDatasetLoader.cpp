#include "SparkRadDatasetLoader.h"

#include "SparkRadFile.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>

#ifdef HAVE_ZLIB
#include <zlib.h>
#endif

namespace SparkRadDatasetLoader
{

bool readRadFile(const std::string& filename, const LoadConfig& config, LoadResult& result)
{
#ifndef HAVE_ZLIB
    std::cerr << "RAD loader requires zlib: " << filename << std::endl;
    return false;
#else
    std::ifstream file(filename, std::ios::binary);
    if (!file) {
        std::cerr << "Failed to open RAD file: " << filename << std::endl;
        return false;
    }

    std::streamoff fileSize = 0;
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);
    if (fileSize < 8) {
        std::cerr << "RAD file too small: " << filename << std::endl;
        return false;
    }

    char magic[4] = {};
    file.read(magic, 4);
    if (std::strncmp(magic, "RAD0", 4) != 0) {
        std::cerr << "Invalid RAD magic: " << filename << std::endl;
        return false;
    }

    uint32_t headerBytes = 0;
    file.read(reinterpret_cast<char*>(&headerBytes), sizeof(headerBytes));
    if (file.gcount() != (std::streamsize)sizeof(headerBytes)) {
        std::cerr << "Truncated RAD header size: " << filename << std::endl;
        return false;
    }

    size_t roundedHeaderBytes = SparkRad::roundUp8((size_t)headerBytes);
    if (roundedHeaderBytes < (size_t)headerBytes || (uint64_t)8 + roundedHeaderBytes > (uint64_t)fileSize) {
        std::cerr << "Invalid RAD header length: " << headerBytes << " in " << filename << std::endl;
        return false;
    }

    std::string header(headerBytes, '\0');
    file.read(&header[0], headerBytes);
    if (file.gcount() != (std::streamsize)headerBytes) {
        std::cerr << "Truncated RAD header: " << filename << std::endl;
        return false;
    }

    result.chunkDataBase = 8 + (std::streamoff)roundedHeaderBytes;
    result.fileInfo.type = SparkRad::jsonString(header, "type");
    result.fileInfo.splatEncoding = SparkRad::jsonString(header, "splatEncoding");
    result.fileInfo.version = SparkRad::jsonInt(header, "version", 0);
    result.fileInfo.count = SparkRad::jsonInt(header, "count", 0);
    result.fileInfo.maxSh = SparkRad::jsonInt(header, "maxSh", 0);
    result.fileInfo.chunkSize = std::max(1, SparkRad::jsonInt(header, "chunkSize", 65536));
    result.fileInfo.lodTree = SparkRad::jsonBool(header, "lodTree", false);

    size_t chunksPos = header.find("\"chunks\"");
    if (chunksPos != std::string::npos) {
        size_t p = header.find('{', chunksPos);
        while (p != std::string::npos) {
            size_t end = header.find('}', p);
            if (end == std::string::npos)
                break;
            std::string item = header.substr(p, end - p + 1);
            RadChunkInfo ch;
            int fallbackChunkSize = std::max(1, result.fileInfo.chunkSize);
            int fallbackBase = (int)result.chunks.size() * fallbackChunkSize;
            int fallbackCount = fallbackChunkSize;
            if (result.fileInfo.count > 0 && fallbackBase < result.fileInfo.count)
                fallbackCount = std::min(fallbackChunkSize, result.fileInfo.count - fallbackBase);
            ch.offset = SparkRad::jsonUInt64(item, "offset", 0);
            ch.bytes = SparkRad::jsonUInt64(item, "bytes", 0);
            ch.payloadBytes = SparkRad::jsonUInt64(item, "payloadBytes", 0);
            ch.base = SparkRad::jsonInt(item, "base", fallbackBase);
            ch.count = SparkRad::jsonInt(item, "count", fallbackCount);
            ch.maxSh = SparkRad::jsonInt(item, "maxSh", 0);
            ch.lodTree = SparkRad::jsonBool(item, "lodTree", false);
            ch.splatEncoding = SparkRad::jsonString(item, "splatEncoding");
            ch.properties = SparkRad::parseProperties(item);
            uint64_t fileDataBytes = (uint64_t)fileSize - (uint64_t)result.chunkDataBase;
            bool validRange = ch.bytes > 0 && ch.offset <= fileDataBytes && ch.bytes <= fileDataBytes - ch.offset;
            if (validRange)
                result.chunks.push_back(ch);
            else
                std::cerr << "Skipping RAD chunk outside file: offset=" << ch.offset << ", bytes=" << ch.bytes << std::endl;
            p = header.find('{', end);
            size_t closeBracket = header.find(']', chunksPos);
            if (p != std::string::npos && closeBracket != std::string::npos && p > closeBracket)
                break;
        }
    }

    for (const RadChunkInfo& ch : result.chunks) {
        if (ch.count <= 0 || ch.base < 0 || (result.fileInfo.count > 0 && (ch.base >= result.fileInfo.count || ch.count > result.fileInfo.count - ch.base))) {
            std::cerr << "Invalid RAD chunk base/count: base=" << ch.base << ", count=" << ch.count << std::endl;
            return false;
        }
    }

    result.totalPointCount = result.fileInfo.count;
    result.totalChunkCount = (int)result.chunks.size();
    result.residentCapacity = config.maxResident;
    result.isRadFile = true;
    result.hasLodOpacity = true;
    result.loadedChunkCount = 0;
    result.evictedChunkCount = 0;
    result.frameNumber = 0;

    int maxSh = result.fileInfo.maxSh;
    std::cout << "RAD resident capacity: " << result.residentCapacity
              << ", totalCount=" << result.totalPointCount
              << ", chunkSize=" << result.fileInfo.chunkSize
              << ", maxSh=" << maxSh
              << std::endl;

    const bool radLODEnabled = true;
    const int MAX_RAD_PREVIEW_POINTS = 1048576;
    const int MAX_RAD_TOTAL_POINTS = 200000000;
    if (result.totalPointCount <= 0 || result.totalPointCount > MAX_RAD_TOTAL_POINTS) {
        std::cerr << "Unsupported RAD point count: " << result.totalPointCount << " in " << filename << std::endl;
        return false;
    }

    result.points.reserve(std::min(result.totalPointCount, result.residentCapacity));
    result.childCounts.reserve(std::min(result.totalPointCount, MAX_RAD_PREVIEW_POINTS));
    result.childStarts.reserve(std::min(result.totalPointCount, MAX_RAD_PREVIEW_POINTS));
    result.localToGlobal.reserve(std::min(result.totalPointCount, MAX_RAD_PREVIEW_POINTS));
    result.leafGlobalIndices.reserve(MAX_RAD_PREVIEW_POINTS);
    result.lodSizes.reserve(MAX_RAD_PREVIEW_POINTS);
    if (radLODEnabled)
        result.globalToLocal.assign(result.totalPointCount, -1);

    int initialChunkLimit = std::max(1, std::min(config.initialChunks, std::max(1, (int)result.chunks.size())));
    std::vector<size_t> chunkQueue;
    std::vector<char> chunkScheduled(result.chunks.size(), 0);
    if (!result.chunks.empty()) {
        chunkQueue.push_back(0);
        chunkScheduled[0] = 1;
    }

    int loadedChunks = 0;
    for (size_t queuePos = 0; queuePos < chunkQueue.size() && (int)result.points.size() < MAX_RAD_PREVIEW_POINTS && (int)result.points.size() < result.residentCapacity && loadedChunks < initialChunkLimit; ++queuePos) {
        size_t chunkIndex = chunkQueue[queuePos];
        const RadChunkInfo& ch = result.chunks[chunkIndex];
        file.seekg(result.chunkDataBase + (std::streamoff)ch.offset);
        std::vector<uint8_t> chunk((size_t)ch.bytes);
        file.read(reinterpret_cast<char*>(chunk.data()), chunk.size());
        if ((size_t)file.gcount() != chunk.size()) {
            std::cerr << "Short RAD initial chunk read: chunk=" << chunkIndex << std::endl;
            continue;
        }

        SparkRad::DecodedChunk decoded;
        if (!SparkRad::decodeChunkPayload(chunk, ch.count, ch.base, decoded))
            continue;

        int base = decoded.base;
        int count = decoded.count;
        if (base < 0 || count <= 0 || base >= result.totalPointCount || count > result.totalPointCount - base) {
            std::cerr << "Skipping RAD chunk with invalid decoded base/count: chunk=" << chunkIndex
                      << ", base=" << base << ", count=" << count
                      << ", total=" << result.totalPointCount << std::endl;
            continue;
        }

        if (chunkIndex < result.chunks.size()) {
            result.chunks[chunkIndex].properties = decoded.properties;
            result.chunks[chunkIndex].payloadBytes = (uint64_t)decoded.payloadBytes;
        }

        int remainingPreview = MAX_RAD_PREVIEW_POINTS - (int)result.points.size();
        int remainingResident = result.residentCapacity - (int)result.points.size();
        if (remainingPreview <= 0 || remainingResident <= 0)
            break;

        std::vector<SparkRad::DecodedSplat> decodedSplats;
        if (!SparkRad::convertDecodedChunkToSplats(decoded, decodedSplats))
            continue;

        int emitCount = std::min((int)decodedSplats.size(), std::min(remainingPreview, remainingResident));
        int startSlot = (int)result.points.size();
        for (int i = 0; i < emitCount; ++i) {
            const SparkRad::DecodedSplat& decodedSplat = decodedSplats[i];
            const MI_GaussianPoint& g = decodedSplat.point;
            uint16_t childCount = decodedSplat.childCount;
            uint32_t childStart = decodedSplat.childStart;

            result.childCounts.push_back(childCount);
            result.childStarts.push_back(childStart);
            if (childCount == 0)
                result.leafGlobalIndices.push_back((uint32_t)(base + i));
            if (radLODEnabled) {
                for (uint16_t c = 0; c < childCount; ++c) {
                    size_t childChunk = (size_t)(childStart + c) / (size_t)result.fileInfo.chunkSize;
                    if (childChunk < result.chunks.size() && !chunkScheduled[childChunk]) {
                        chunkScheduled[childChunk] = 1;
                        chunkQueue.push_back(childChunk);
                    }
                }
            }
            result.bounds.expandBy(g.position);
            uint32_t globalIndex = (uint32_t)(base + i);
            result.lodSizes.push_back(decodedSplat.lodSize);
            result.localToGlobal.push_back(globalIndex);
            if (radLODEnabled && globalIndex < result.globalToLocal.size())
                result.globalToLocal[globalIndex] = (int)result.points.size();
            result.points.push_back(g);
        }

        ++loadedChunks;
        if (chunkIndex < result.chunks.size()) {
            result.chunks[chunkIndex].resident = true;
            result.chunks[chunkIndex].queued = false;
            result.chunks[chunkIndex].base = base;
            result.chunks[chunkIndex].count = emitCount;
            result.chunkSlots.resize(result.chunks.size());
            if (result.slotToChunk.size() < result.points.size())
                result.slotToChunk.resize(result.points.size(), -1);
            result.chunkSlots[chunkIndex].clear();
            for (int slot = startSlot; slot < startSlot + emitCount; ++slot) {
                result.chunkSlots[chunkIndex].push_back(slot);
                if (slot >= 0 && slot < (int)result.slotToChunk.size())
                    result.slotToChunk[slot] = (int)chunkIndex;
            }
        }
        result.loadedChunkCount = loadedChunks;
    }

    result.hasSH = maxSh > 0 && !config.disableSh;
    result.globalShOrder = result.hasSH ? maxSh : 0;

    // The first LOD chunk can be dominated by distant background splats. Sample
    // centers across the file instead of fitting the camera to that chunk.
    std::vector<osg::Vec3f> focusSamples;
    const size_t sampleChunks = std::min(size_t(20), result.chunks.size());
    focusSamples.reserve(sampleChunks * 256);
    size_t sampledChunks = 0;
    for (size_t sample = 0; sample < sampleChunks; ++sample) {
        const size_t chunkIndex = size_t((2 * uint64_t(sample) + 1) * uint64_t(result.chunks.size()) / (2 * sampleChunks));
        if (SparkRad::readChunkCenterSamples(file, result.chunkDataBase, result.chunks[chunkIndex], focusSamples))
            ++sampledChunks;
    }
    result.focusBounds = sampledChunks * 2 >= sampleChunks
        ? SparkRad::computeFocusBounds(focusSamples, result.bounds) : result.bounds;
    if (result.focusBounds.valid()) {
        const osg::Vec3f center = result.focusBounds.center();
        std::cout << "RAD focus: samples=" << focusSamples.size()
                  << ", chunks=" << sampledChunks
                  << ", center=" << center.x() << "," << center.y() << "," << center.z()
                  << ", radius=" << result.focusBounds.radius() << std::endl;
    }
    return true;
#endif
}

} // namespace SparkRadDatasetLoader
