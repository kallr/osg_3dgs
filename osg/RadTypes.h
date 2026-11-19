#ifndef RAD_TYPES_H
#define RAD_TYPES_H

#include "osg/Vec3f"
#include "osg/Vec4f"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

struct MI_GaussianPoint {
	osg::Vec3f position;
	osg::Vec4f color;      // DC color (rgba), alpha = sigmoid(opacity)
	osg::Vec3f scale;      // log-space scale (already exponentiated for .splat)
	osg::Vec4f rotation;   // quaternion (x, y, z, w)
	float sh[45] = {};     // 1~3 order SH coefficients
	int shOrder = 0;       // SH degree: 0=DC only, 1=1st order, 2=2nd order, 3=3rd order
};

struct RadPropertyInfo {
	std::string property;
	std::string encoding;
	uint32_t offset = 0;
	uint32_t bytes = 0;
	float minValue = 0.0f;
	float maxValue = 1.0f;
	bool compressed = true;
};

struct RadFileInfo {
	std::string type;
	std::string splatEncoding;
	int version = 0;
	int count = 0;
	int maxSh = 0;
	int chunkSize = 65536;
	bool lodTree = false;
};

struct RadChunkInfo {
	uint64_t offset = 0;
	uint64_t bytes = 0;
	uint64_t payloadBytes = 0;
	int base = 0;
	int count = 0;
	int maxSh = 0;
	bool lodTree = false;
	std::string splatEncoding;
	std::vector<RadPropertyInfo> properties;
	bool resident = false;
	bool queued = false;
	float priority = 0.0f;
	unsigned int lastTouchedFrame = 0;
	uint32_t loadGeneration = 0;
	// Paged Pool extension
	int pageIndex = -1;
};

namespace SparkRad { struct PreparedChunk; }

struct RadRawChunk {
	uint32_t chunkIndex = 0;
	uint32_t loadGeneration = 0;
	std::vector<uint8_t> bytes;
	// Optional immutable CPU preparation produced by the streaming thread.
	// No GPU or resident-slot state is owned or changed by this payload.
	std::shared_ptr<const SparkRad::PreparedChunk> prepared;
};

#endif
