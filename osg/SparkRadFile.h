#ifndef SPARK_RAD_FILE_H
#define SPARK_RAD_FILE_H

#include "RadTypes.h"
#include <osg/BoundingBox>

#include <cstdint>
#include <istream>
#include <string>
#include <vector>

namespace SparkRad
{
	size_t roundUp8(size_t size);
	int jsonInt(const std::string& s, const std::string& key, int fallback);
	uint64_t jsonUInt64(const std::string& s, const std::string& key, uint64_t fallback);
	float jsonFloat(const std::string& s, const std::string& key, float fallback);
	std::string jsonString(const std::string& s, const std::string& key, const std::string& fallback = std::string());
	bool jsonBool(const std::string& s, const std::string& key, bool fallback);

	bool knownProperty(const std::string& property);
	bool propertyEncodingSupported(const std::string& property, const std::string& encoding);
	std::vector<RadPropertyInfo> parseProperties(const std::string& chunkHeader);
	bool propertyRange(const std::vector<RadPropertyInfo>& properties, const std::string& name, float& minValue, float& maxValue);

	struct DecodedChunk
	{
		std::string header;
		int count = 0;
		int base = 0;
		size_t payloadBytes = 0;
		std::vector<RadPropertyInfo> properties;
		std::vector<uint8_t> center;
		std::vector<uint8_t> alpha;
		std::vector<uint8_t> rgb;
		std::vector<uint8_t> scales;
		std::vector<uint8_t> orientation;
		std::vector<uint8_t> sh1;
		std::vector<uint8_t> sh2;
		std::vector<uint8_t> sh3;
		std::vector<uint8_t> shLabel;
		std::vector<uint8_t> childCountData;
		std::vector<uint8_t> childStartData;
		// Per-property encoding strings (for dispatch in conversion)
		std::string centerEncoding;
		std::string alphaEncoding;
		std::string rgbEncoding;
		std::string scalesEncoding;
		std::string orientationEncoding;
		std::string sh1Encoding;
		std::string sh2Encoding;
		std::string sh3Encoding;
		std::string shLabelEncoding;
		// Quantization ranges from property metadata
		float alphaMin = 0.0f;
		float alphaMax = 1.0f;
		float sh1Max = 1.0f;
		float sh2Max = 1.0f;
		float sh3Max = 1.0f;
		// SH codebook (clustering)
		int shCodeCount = 0;
		std::vector<float> sh1Codebook;  // shCodeCount * 9 floats
		std::vector<float> sh2Codebook;  // shCodeCount * 15 floats
		std::vector<float> sh3Codebook;  // shCodeCount * 21 floats
	};

	struct DecodedSplat
	{
		MI_GaussianPoint point;
		uint16_t childCount = 0;
		uint32_t childStart = 0;
		float lodSize = 0.0f;
	};

	struct PrepareOptions
	{
		int totalPointCount = 0;
		// Zero page size and encoding select the legacy decoded-only path.
		int maxPageSplats = 0;
		int splatEncoding = 0;
		bool timingsEnabled = false;
	};

	// Detached CPU payload. Preparation does not allocate a physical page or
	// mutate renderer arrays, residency, OSG images or an OpenGL context.
	struct PreparedChunk
	{
		int count = 0;
		int base = 0;
		size_t payloadBytes = 0;
		std::vector<RadPropertyInfo> properties;
		std::vector<DecodedSplat> splats;
		std::vector<uint32_t> packedData;
		std::vector<float> shData;
		int encoding = 0;
		double decodeMilliseconds = 0.0;
		double convertMilliseconds = 0.0;
		double packMilliseconds = 0.0;
	};

	// Helper for Spark Paged Pipeline
	bool encodeSplatsToPacked(const std::vector<DecodedSplat>& splats, int splatEncoding, std::vector<uint32_t>& outPackedData);

	// Framing bounds are sampled independently of render/culling bounds.
	osg::BoundingBox computeFocusBounds(const std::vector<osg::Vec3f>& centers, const osg::BoundingBox& fallback);

#ifdef HAVE_ZLIB
	bool encodeChunkToPacked(const DecodedChunk& decoded, int splatEncoding, std::vector<uint32_t>& outPackedData);
	// Read/inflate only the center property; sampled chunks do not become resident.
	bool readChunkCenterSamples(std::istream& file, std::streamoff chunkDataBase, const RadChunkInfo& chunk,
	                            std::vector<osg::Vec3f>& centers, size_t maxSamples = 256);
	bool decodeChunkPayload(const std::vector<uint8_t>& chunk, int fallbackCount, int fallbackBase, DecodedChunk& out);
	bool convertDecodedChunkToSplats(const DecodedChunk& decoded, std::vector<DecodedSplat>& splats);
	// Builds a complete payload before publishing it; false leaves out empty.
	bool prepareChunk(const std::vector<uint8_t>& chunk, int fallbackCount, int fallbackBase,
	                  const PrepareOptions& options, PreparedChunk& out);
#endif
}

#endif
