#include "SparkRadFile.h"
#include "SparkPackedEncoding.h"
#include "SparkGpuResources.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <utility>

bool SparkRad::encodeSplatsToPacked(const std::vector<DecodedSplat>& splats, int splatEncoding, std::vector<uint32_t>& outPackedData)
{
	if (splats.empty()) return false;

	int texelsPerSplat = (splatEncoding == (int)SplatEncoding::PackedSplat) ? 1 : 2;
	int wordsPerSplat = texelsPerSplat * 4;
	outPackedData.resize(splats.size() * wordsPerSplat);

	for (size_t i = 0; i < splats.size(); ++i)
	{
		const MI_GaussianPoint& g = splats[i].point;
		// RAD base color is already pure color, we don't premultiply here for Spark path.
		float alpha = g.color.a();
		float r = g.color.r();
		float gr = g.color.g();
		float b = g.color.b();

		if (splatEncoding == (int)SplatEncoding::PackedSplat) {
			SparkPacked::PackedSplat ps;
			ps.encode(g.position.x(), g.position.y(), g.position.z(),
					  g.scale.x(), g.scale.y(), g.scale.z(),
					  g.rotation.x(), g.rotation.y(), g.rotation.z(), g.rotation.w(),
					  r, gr, b, alpha);
			std::memcpy(&outPackedData[i * wordsPerSplat], ps.words, sizeof(ps.words));
		}
		else if (splatEncoding == (int)SplatEncoding::ExtSplat) {
			SparkPacked::ExtSplat es;
			es.encode(g.position.x(), g.position.y(), g.position.z(),
					  g.scale.x(), g.scale.y(), g.scale.z(),
					  g.rotation.x(), g.rotation.y(), g.rotation.z(), g.rotation.w(),
					  r, gr, b, alpha);
			std::memcpy(&outPackedData[i * wordsPerSplat], es.extA, sizeof(es.extA));
			std::memcpy(&outPackedData[i * wordsPerSplat + 4], es.extB, sizeof(es.extB));
		}
	}
	return true;
}

#ifdef HAVE_ZLIB
bool SparkRad::encodeChunkToPacked(const DecodedChunk& decoded, int splatEncoding, std::vector<uint32_t>& outPackedData)
{
	if (decoded.count <= 0) return false;

	std::vector<DecodedSplat> splats;
	if (!convertDecodedChunkToSplats(decoded, splats)) return false;
	return encodeSplatsToPacked(splats, splatEncoding, outPackedData);
}

#include <zlib.h>
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SparkRad
{
	osg::BoundingBox computeFocusBounds(const std::vector<osg::Vec3f>& centers, const osg::BoundingBox& fallback)
	{
		std::vector<float> coordinates[3];
		for (const osg::Vec3f& center : centers) {
			if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z()))
				continue;
			for (int d = 0; d < 3; ++d)
				coordinates[d].push_back(center[d]);
		}
		const size_t count = coordinates[0].size();
		if (count < 32)
			return fallback;
		osg::Vec3f lower, upper;
		const size_t trim = count / 20; // Trim the outer 5% on each axis.
		for (int d = 0; d < 3; ++d) {
			std::sort(coordinates[d].begin(), coordinates[d].end());
			const double lo = coordinates[d][trim];
			const double hi = coordinates[d][count - 1 - trim];
			const double padding = (hi - lo) * 0.1;
			lower[d] = float(lo - padding);
			upper[d] = float(hi + padding);
			if (!std::isfinite(lower[d]) || !std::isfinite(upper[d]))
				return fallback;
		}
		osg::BoundingBox focus(lower, upper);
		const double radius = focus.radius();
		return focus.valid() && std::isfinite(radius) && radius > 1.0e-6 ? focus : fallback;
	}

	size_t roundUp8(size_t size)
	{
		return (size + 7u) & ~size_t(7u);
	}

	int jsonInt(const std::string& s, const std::string& key, int fallback)
	{
		size_t p = s.find("\"" + key + "\"");
		if (p == std::string::npos) return fallback;
		p = s.find(':', p);
		if (p == std::string::npos) return fallback;
		return std::atoi(s.c_str() + p + 1);
	}

	uint64_t jsonUInt64(const std::string& s, const std::string& key, uint64_t fallback)
	{
		size_t p = s.find("\"" + key + "\"");
		if (p == std::string::npos) return fallback;
		p = s.find(':', p);
		if (p == std::string::npos) return fallback;
		return (uint64_t)std::strtoull(s.c_str() + p + 1, nullptr, 10);
	}

	float jsonFloat(const std::string& s, const std::string& key, float fallback)
	{
		size_t p = s.find("\"" + key + "\"");
		if (p == std::string::npos) return fallback;
		p = s.find(':', p);
		if (p == std::string::npos) return fallback;
		return (float)std::atof(s.c_str() + p + 1);
	}

	std::string jsonString(const std::string& s, const std::string& key, const std::string& fallback)
	{
		size_t p = s.find("\"" + key + "\"");
		if (p == std::string::npos) return fallback;
		p = s.find(':', p);
		if (p == std::string::npos) return fallback;
		p = s.find('"', p);
		if (p == std::string::npos) return fallback;
		size_t q = s.find('"', p + 1);
		if (q == std::string::npos) return fallback;
		return s.substr(p + 1, q - p - 1);
	}

	bool jsonBool(const std::string& s, const std::string& key, bool fallback)
	{
		size_t p = s.find("\"" + key + "\"");
		if (p == std::string::npos) return fallback;
		p = s.find(':', p);
		if (p == std::string::npos) return fallback;
		const char* v = s.c_str() + p + 1;
		while (*v && std::isspace((unsigned char)*v)) ++v;
		if (std::strncmp(v, "true", 4) == 0) return true;
		if (std::strncmp(v, "false", 5) == 0) return false;
		return fallback;
	}

	bool knownProperty(const std::string& property)
	{
		return property == "center" || property == "alpha" || property == "rgb" ||
		       property == "scales" || property == "orientation" ||
		       property == "sh1" || property == "sh2" || property == "sh3" ||
		       property == "sh_label" ||
		       property == "child_count" || property == "child_start";
	}

	namespace
	{
		bool encodingTokenMatches(const std::string& encoding, const char* a, const char* b = nullptr, const char* c = nullptr)
		{
			std::string lower = encoding;
			std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char ch) { return (char)std::tolower(ch); });
			return lower.find(a) != std::string::npos ||
			       (b && lower.find(b) != std::string::npos) ||
			       (c && lower.find(c) != std::string::npos);
		}
	}

	bool propertyEncodingSupported(const std::string& property, const std::string& encoding)
	{
		if (encoding.empty())
			return true;
		// Center: F32, F16, F32LeBytes, F16LeBytes
		if (property == "center") return encodingTokenMatches(encoding, "float", "f32", "f16") ||
		                                   encodingTokenMatches(encoding, "le_bytes");
		// Alpha: F32, F16, R8, U16
		if (property == "alpha") return encodingTokenMatches(encoding, "float", "f32", "f16") ||
		                                  encodingTokenMatches(encoding, "half", "r8", "u16");
		// RGB: F32, F16, R8, R8Delta
		if (property == "rgb") return encodingTokenMatches(encoding, "float", "f32", "f16") ||
		                               encodingTokenMatches(encoding, "r8", "u8", "uint8");
		// Scales: F32, Ln0R8, LnF16
		if (property == "scales") return encodingTokenMatches(encoding, "float", "f32", "f16") ||
		                                   encodingTokenMatches(encoding, "ln_0r8", "u8", "ln_f16");
		// Orientation: F32, F16, Oct88R8
		if (property == "orientation") return encodingTokenMatches(encoding, "float", "f32", "f16") ||
		                                        encodingTokenMatches(encoding, "oct88r8", "u8");
		// SH1/2/3: F32, F16, S8, S8Delta
		if (property == "sh1" || property == "sh2" || property == "sh3")
			return encodingTokenMatches(encoding, "float", "f32", "f16") ||
			       encodingTokenMatches(encoding, "s8", "i8", "int8");
		// SH Label: U16, U32
		if (property == "sh_label") return encodingTokenMatches(encoding, "u16", "u32", "uint16");
		if (property == "child_count") return encodingTokenMatches(encoding, "u16", "uint16");
		if (property == "child_start") return encodingTokenMatches(encoding, "u32", "uint32");
		return true;
	}

	std::vector<RadPropertyInfo> parseProperties(const std::string& chunkHeader)
	{
		std::vector<RadPropertyInfo> props;
		size_t propsPos = chunkHeader.find("\"properties\"");
		if (propsPos == std::string::npos) return props;
		// 使用括号配对计数定位 properties 数组的真正结束位置，避免嵌套数组提前截断
		size_t arrayStart = chunkHeader.find('[', propsPos);
		if (arrayStart == std::string::npos) return props;
		size_t arrayEnd = std::string::npos;
		int depth = 0;
		for (size_t i = arrayStart; i < chunkHeader.size(); ++i) {
			if (chunkHeader[i] == '[') ++depth;
			else if (chunkHeader[i] == ']') {
				--depth;
				if (depth == 0) { arrayEnd = i; break; }
			}
		}
		if (arrayEnd == std::string::npos) return props;
		size_t p = chunkHeader.find('{', propsPos);
		while (p != std::string::npos && p < arrayEnd) {
			size_t end = chunkHeader.find('}', p);
			if (end == std::string::npos || end > arrayEnd) break;
			std::string item = chunkHeader.substr(p, end - p + 1);
			RadPropertyInfo prop;
			prop.property = jsonString(item, "property");
			prop.encoding = jsonString(item, "encoding");
			uint64_t offset64 = jsonUInt64(item, "offset", 0);
			uint64_t bytes64 = jsonUInt64(item, "bytes", 0);
			if (prop.property.empty()) {
				p = chunkHeader.find('{', end);
				continue;
			}
			if (offset64 > std::numeric_limits<uint32_t>::max() || bytes64 > std::numeric_limits<uint32_t>::max()) {
				std::cerr << "Skipping RAD property with oversized range: " << prop.property
				          << ", offset=" << offset64 << ", bytes=" << bytes64 << std::endl;
				p = chunkHeader.find('{', end);
				continue;
			}
			if (knownProperty(prop.property) && bytes64 == 0) {
				std::cerr << "Skipping RAD property with zero bytes: " << prop.property << std::endl;
				p = chunkHeader.find('{', end);
				continue;
			}
			if (knownProperty(prop.property) && !propertyEncodingSupported(prop.property, prop.encoding)) {
				std::cerr << "Skipping RAD property with unsupported encoding: " << prop.property
				          << ", encoding=" << prop.encoding << std::endl;
				p = chunkHeader.find('{', end);
				continue;
			}
			prop.offset = (uint32_t)offset64;
			prop.bytes = (uint32_t)bytes64;
			prop.minValue = jsonFloat(item, "min", 0.0f);
			prop.maxValue = jsonFloat(item, "max", 1.0f);
			prop.compressed = jsonBool(item, "compressed", true);
			props.push_back(prop);
			p = chunkHeader.find('{', end);
		}
		return props;
	}

	bool propertyRange(const std::vector<RadPropertyInfo>& properties, const std::string& name, float& minValue, float& maxValue)
	{
		for (const RadPropertyInfo& prop : properties) {
			if (prop.property != name)
				continue;
			minValue = prop.minValue;
			maxValue = prop.maxValue;
			return true;
		}
		return false;
	}

#ifdef HAVE_ZLIB
	namespace
	{
		float halfToFloat(uint16_t h)
		{
			uint32_t sign = (uint32_t)(h & 0x8000u) << 16;
			uint32_t exp = (h >> 10) & 0x1fu;
			uint32_t mant = h & 0x03ffu;
			uint32_t bits = 0;
			if (exp == 0) {
				if (mant == 0) {
					bits = sign;
				} else {
					exp = 1;
					while ((mant & 0x0400u) == 0) { mant <<= 1; --exp; }
					mant &= 0x03ffu;
					bits = sign | ((exp + 112u) << 23) | (mant << 13);
				}
			} else if (exp == 31) {
				bits = sign | 0x7f800000u | (mant << 13);
			} else {
				bits = sign | ((exp + 112u) << 23) | (mant << 13);
			}
			float f;
			std::memcpy(&f, &bits, sizeof(float));
			return f;
		}

		bool mulSize(size_t a, size_t b, size_t& out)
		{
			if (a != 0 && b > std::numeric_limits<size_t>::max() / a)
				return false;
			out = a * b;
			return true;
		}

		bool inflateRaw(const uint8_t* src, size_t srcBytes, std::vector<uint8_t>& out, size_t expectedBytes = 0)
		{
			z_stream zs;
			std::memset(&zs, 0, sizeof(zs));
			if (inflateInit2(&zs, -MAX_WBITS) != Z_OK) return false;
			zs.next_in = const_cast<Bytef*>(reinterpret_cast<const Bytef*>(src));
			zs.avail_in = (uInt)srcBytes;
			out.clear();
			if (expectedBytes > 0)
				out.reserve(expectedBytes);
			uint8_t temp[32768];
			int ret = Z_OK;
			while (ret == Z_OK) {
				zs.next_out = temp;
				zs.avail_out = sizeof(temp);
				ret = inflate(&zs, Z_NO_FLUSH);
				if (ret != Z_OK && ret != Z_STREAM_END) break;
				size_t produced = sizeof(temp) - zs.avail_out;
				if (expectedBytes > 0 && out.size() + produced > expectedBytes) {
					inflateEnd(&zs);
					out.clear();
					return false;
				}
				out.insert(out.end(), temp, temp + produced);
			}
			inflateEnd(&zs);
			if (ret != Z_STREAM_END)
				return false;
			return expectedBytes == 0 || out.size() == expectedBytes;
		}

		bool expectedPropertyBytes(const std::string& property, const std::string& encoding, int count, size_t& bytes)
		{
			bytes = 0;
			if (count <= 0)
				return false;
			size_t n = (size_t)count;
			// Center: F32=12, F16=6, F32LeBytes=12, F16LeBytes=6
			if (property == "center") {
				if (encodingTokenMatches(encoding, "f16") || encodingTokenMatches(encoding, "le_bytes"))
					return encodingTokenMatches(encoding, "f16") && !encodingTokenMatches(encoding, "f32")
					       ? mulSize(n, 6, bytes) : mulSize(n, 12, bytes);
				return mulSize(n, 12, bytes); // default F32
			}
			// Alpha: F32=4, F16/U16=2, R8=1
			if (property == "alpha") {
				if (encodingTokenMatches(encoding, "r8")) return mulSize(n, 1, bytes);
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 4, bytes);
				return mulSize(n, 2, bytes); // default F16/U16
			}
			// RGB: F32=12, F16=6, R8/R8Delta=3
			if (property == "rgb") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 12, bytes);
				if (encodingTokenMatches(encoding, "f16")) return mulSize(n, 6, bytes);
				return mulSize(n, 3, bytes); // default R8/R8Delta
			}
			// Scales: F32=12, F16=6, Ln0R8=3, LnF16=6
			if (property == "scales") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 12, bytes);
				if (encodingTokenMatches(encoding, "f16") || encodingTokenMatches(encoding, "ln_f16"))
					return mulSize(n, 6, bytes);
				return mulSize(n, 3, bytes); // default Ln0R8
			}
			// RAD float orientations store xyz; the positive w is reconstructed.
			if (property == "orientation") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 12, bytes);
				if (encodingTokenMatches(encoding, "f16")) return mulSize(n, 6, bytes);
				return mulSize(n, 3, bytes); // default Oct88R8
			}
			// SH1: F32=36, F16=18, S8/S8Delta=9
			if (property == "sh1") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 36, bytes);
				if (encodingTokenMatches(encoding, "f16")) return mulSize(n, 18, bytes);
				return mulSize(n, 9, bytes); // default S8
			}
			// SH2: F32=60, F16=30, S8/S8Delta=15
			if (property == "sh2") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 60, bytes);
				if (encodingTokenMatches(encoding, "f16")) return mulSize(n, 30, bytes);
				return mulSize(n, 15, bytes); // default S8
			}
			// SH3: F32=84, F16=42, S8/S8Delta=21
			if (property == "sh3") {
				if (encodingTokenMatches(encoding, "f32") || encodingTokenMatches(encoding, "float"))
					return mulSize(n, 84, bytes);
				if (encodingTokenMatches(encoding, "f16")) return mulSize(n, 42, bytes);
				return mulSize(n, 21, bytes); // default S8
			}
			// SH Label: U16=2, U32=4
			if (property == "sh_label") {
				if (encodingTokenMatches(encoding, "u32")) return mulSize(n, 4, bytes);
				return mulSize(n, 2, bytes); // default U16
			}
			if (property == "child_count") return mulSize(n, 2, bytes);
			if (property == "child_start") return mulSize(n, 4, bytes);
			return false;
		}

		float bytePlaneFloat(const std::vector<uint8_t>& data, int count, int componentIndex, int pointIndex)
		{
			int k = componentIndex * count + pointIndex;
			int planeSize = count * 3;
			uint8_t bytes[4] = { data[k], data[planeSize + k], data[planeSize * 2 + k], data[planeSize * 3 + k] };
			float v = 0.0f;
			std::memcpy(&v, bytes, sizeof(float));
			return v;
		}

		float planarFloat(const std::vector<uint8_t>& data, int count, int componentIndex, int pointIndex)
		{
			float value = 0.0f;
			std::memcpy(&value, &data[(componentIndex * count + pointIndex) * sizeof(float)], sizeof(value));
			return value;
		}

		float bytePlaneHalf(const std::vector<uint8_t>& data, int count, int componentIndex, int pointIndex)
		{
			int index = componentIndex * count + pointIndex;
			return halfToFloat((uint16_t)data[index] | ((uint16_t)data[count * 3 + index] << 8));
		}

		uint16_t readU16(const std::vector<uint8_t>& data, int index)
		{
			return (uint16_t)(data[index * 2] | (data[index * 2 + 1] << 8));
		}

		uint32_t readU32(const std::vector<uint8_t>& data, int index)
		{
			return (uint32_t)data[index * 4] |
			       ((uint32_t)data[index * 4 + 1] << 8) |
			       ((uint32_t)data[index * 4 + 2] << 16) |
			       ((uint32_t)data[index * 4 + 3] << 24);
		}

		uint8_t planarU8(const std::vector<uint8_t>& data, int count, int dim, int i)
		{
			return data[dim * count + i];
		}

		float decodeScale8(uint8_t value, float lnMin, float lnMax)
		{
			if (value == 0)
				return 0.0f;
			float t = (float)(value - 1) / 254.0f;
			return std::exp(lnMin + t * (lnMax - lnMin));
		}

		osg::Vec4f decodeQuatOct888(uint8_t u, uint8_t v, uint8_t r)
		{
			// R8 has only 256 possible angles. Keep the original float arithmetic
			// and libm results, but share them across all points and decode workers.
			static const std::array<std::array<float, 2>, 256> angles = [] {
				std::array<std::array<float, 2>, 256> table{};
				for (size_t i = 0; i < table.size(); ++i) {
					const float halfTheta = (float)i / 255.0f * 0.5f * (float)M_PI;
					table[i] = {{std::sin(halfTheta), std::cos(halfTheta)}};
				}
				return table;
			}();
			float x = (float)u / 255.0f * 2.0f - 1.0f;
			float y = (float)v / 255.0f * 2.0f - 1.0f;
			float z = 1.0f - std::abs(x) - std::abs(y);
			float t = std::max(-z, 0.0f);
			x = (x >= 0.0f) ? x - t : x + t;
			y = (y >= 0.0f) ? y - t : y + t;
			float len = std::sqrt(x * x + y * y + z * z);
			if (len <= 1.0e-6f)
				return osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
			x /= len; y /= len; z /= len;
			const float s = angles[r][0];
			const float w = angles[r][1];
			return osg::Vec4f(x * s, y * s, z * s, w);
		}

		osg::Vec3f decodeCenter(const std::vector<uint8_t>& data, int count, bool half, bool bytePlanes, int i)
		{
			osg::Vec3f center;
			for (int d = 0; d < 3; ++d) {
				center[d] = bytePlanes
					? (half ? bytePlaneHalf(data, count, d, i) : bytePlaneFloat(data, count, d, i))
					: (half ? halfToFloat(readU16(data, d * count + i)) : planarFloat(data, count, d, i));
			}
			return center;
		}
	}

	bool readChunkCenterSamples(std::istream& file, std::streamoff chunkDataBase, const RadChunkInfo& chunk,
	                            std::vector<osg::Vec3f>& centers, size_t maxSamples)
	{
		// Sampling is optional: bound its memory use even for malformed metadata.
		if (chunkDataBase < 0 || chunk.bytes < 16 || maxSamples == 0 ||
		    chunk.offset > uint64_t(std::numeric_limits<std::streamoff>::max() - chunkDataBase))
			return false;
		const std::streamoff start = chunkDataBase + std::streamoff(chunk.offset);
		if (chunk.bytes > uint64_t(std::numeric_limits<std::streamoff>::max() - start))
			return false;
		file.clear();
		file.seekg(start);
		char magic[4];
		uint32_t headerBytes = 0;
		file.read(magic, 4);
		file.read(reinterpret_cast<char*>(&headerBytes), sizeof(headerBytes));
		if (!file || std::memcmp(magic, "RADC", 4) != 0 || headerBytes == 0 || headerBytes > 1024 * 1024)
			return false;
		const size_t payloadOffset = 16 + roundUp8(headerBytes);
		if (payloadOffset > chunk.bytes)
			return false;
		std::string header(headerBytes, '\0');
		file.read(&header[0], headerBytes);
		if (!file)
			return false;
		const int count = jsonInt(header, "count", chunk.count);
		if (count <= 0 || count > 1048576)
			return false;
		file.seekg(start + std::streamoff(payloadOffset - 8));
		uint64_t declaredBytes = 0;
		file.read(reinterpret_cast<char*>(&declaredBytes), sizeof(declaredBytes));
		const uint64_t availableBytes = chunk.bytes - payloadOffset;
		if (!file || declaredBytes > availableBytes)
			return false;
		const uint64_t payloadBytes = declaredBytes ? declaredBytes : availableBytes;
		for (const RadPropertyInfo& property : parseProperties(header)) {
			if (property.property != "center")
				continue;
			size_t expectedBytes = 0;
			if (!propertyEncodingSupported("center", property.encoding) ||
			    !expectedPropertyBytes("center", property.encoding, count, expectedBytes) ||
			    property.offset > payloadBytes || property.bytes > payloadBytes - property.offset ||
			    property.bytes > 16 * 1024 * 1024)
				return false;
			std::vector<uint8_t> encoded(property.bytes), decoded;
			file.seekg(start + std::streamoff(payloadOffset) + property.offset);
			file.read(reinterpret_cast<char*>(encoded.data()), encoded.size());
			if (!file)
				return false;
			if (property.compressed) {
				if (!inflateRaw(encoded.data(), encoded.size(), decoded, expectedBytes))
					return false;
			} else {
				if (encoded.size() < expectedBytes)
					return false;
				decoded.swap(encoded);
			}
			const size_t samples = std::min(maxSamples, size_t(count));
			const bool half = encodingTokenMatches(property.encoding, "f16") && !encodingTokenMatches(property.encoding, "f32");
			const bool bytePlanes = encodingTokenMatches(property.encoding, "lebytes", "le_bytes");
			for (size_t sample = 0; sample < samples; ++sample) {
				const int i = int((2 * uint64_t(sample) + 1) * uint64_t(count) / (2 * uint64_t(samples)));
				const osg::Vec3f center = decodeCenter(decoded, count, half, bytePlanes, i);
				if (std::isfinite(center.x()) && std::isfinite(center.y()) && std::isfinite(center.z()))
					centers.push_back(center);
			}
			return true;
		}
		return false;
	}

	bool decodeChunkPayload(const std::vector<uint8_t>& chunk, int fallbackCount, int fallbackBase, DecodedChunk& out)
	{
		out = DecodedChunk();
		if (chunk.size() < 8 || std::strncmp(reinterpret_cast<const char*>(chunk.data()), "RADC", 4) != 0)
			return false;
		uint32_t chunkHeaderBytes = 0;
		std::memcpy(&chunkHeaderBytes, chunk.data() + 4, sizeof(chunkHeaderBytes));
		if ((size_t)chunkHeaderBytes > chunk.size() - 8)
			return false;
		size_t roundedHeaderBytes = roundUp8((size_t)chunkHeaderBytes);
		if (roundedHeaderBytes < (size_t)chunkHeaderBytes || roundedHeaderBytes > chunk.size() - 8)
			return false;
		size_t metaEnd = 8 + roundedHeaderBytes;
		if (metaEnd > chunk.size() || chunk.size() - metaEnd < 8)
			return false;
		size_t payloadStart = metaEnd + 8;

		out.header.assign(reinterpret_cast<const char*>(chunk.data() + 8), chunkHeaderBytes);
		out.count = jsonInt(out.header, "count", fallbackCount);
		out.base = jsonInt(out.header, "base", fallbackBase);
		if (out.count <= 0 || out.base < 0)
			return false;
		out.properties = parseProperties(out.header);

		const uint8_t* payload = chunk.data() + payloadStart;
		out.payloadBytes = chunk.size() - payloadStart;
		uint64_t declaredPayloadBytes = 0;
		std::memcpy(&declaredPayloadBytes, chunk.data() + metaEnd, sizeof(declaredPayloadBytes));
		if (declaredPayloadBytes > out.payloadBytes)
			return false;
		if (declaredPayloadBytes > 0)
			out.payloadBytes = (size_t)declaredPayloadBytes;

		for (const RadPropertyInfo& prop : out.properties) {
			if (prop.offset > out.payloadBytes || (size_t)prop.bytes > out.payloadBytes - prop.offset) {
				std::cerr << "Skipping RAD property outside payload: " << prop.property
				          << ", offset=" << prop.offset << ", bytes=" << prop.bytes
				          << ", payloadBytes=" << out.payloadBytes << std::endl;
				continue;
			}
			size_t expectedBytes = 0;
			if (!expectedPropertyBytes(prop.property, prop.encoding, out.count, expectedBytes))
				continue;
			std::vector<uint8_t> decoded;
			if (prop.compressed) {
				if (!inflateRaw(payload + prop.offset, (size_t)prop.bytes, decoded, expectedBytes)) {
					std::cerr << "Failed to inflate RAD property: " << prop.property << std::endl;
					continue;
				}
			} else {
				if ((size_t)prop.bytes < expectedBytes) {
					std::cerr << "Skipping short uncompressed RAD property: " << prop.property
					          << ", bytes=" << prop.bytes << ", expected=" << expectedBytes << std::endl;
					continue;
				}
				decoded.assign(payload + prop.offset, payload + prop.offset + expectedBytes);
			}
			if (decoded.size() != expectedBytes)
				continue;
			if (prop.property == "center") { out.center.swap(decoded); out.centerEncoding = prop.encoding; }
			else if (prop.property == "alpha") { out.alpha.swap(decoded); out.alphaEncoding = prop.encoding; out.alphaMin = prop.minValue; out.alphaMax = prop.maxValue; }
			else if (prop.property == "rgb") { out.rgb.swap(decoded); out.rgbEncoding = prop.encoding; }
			else if (prop.property == "scales") { out.scales.swap(decoded); out.scalesEncoding = prop.encoding; }
			else if (prop.property == "orientation") { out.orientation.swap(decoded); out.orientationEncoding = prop.encoding; }
			else if (prop.property == "sh1") { out.sh1.swap(decoded); out.sh1Encoding = prop.encoding; out.sh1Max = prop.maxValue; }
			else if (prop.property == "sh2") { out.sh2.swap(decoded); out.sh2Encoding = prop.encoding; out.sh2Max = prop.maxValue; }
			else if (prop.property == "sh3") { out.sh3.swap(decoded); out.sh3Encoding = prop.encoding; out.sh3Max = prop.maxValue; }
			else if (prop.property == "sh_label") { out.shLabel.swap(decoded); out.shLabelEncoding = prop.encoding; }
			else if (prop.property == "child_count") out.childCountData.swap(decoded);
			else if (prop.property == "child_start") out.childStartData.swap(decoded);
		}
		size_t expectedCenterBytes = 0;
		return expectedPropertyBytes("center", out.centerEncoding, out.count, expectedCenterBytes) && out.center.size() == expectedCenterBytes;
	}

	bool convertDecodedChunkToSplats(const DecodedChunk& decoded, std::vector<DecodedSplat>& splats)
	{
		splats.clear();
		if (decoded.count <= 0)
			return false;
		size_t expectedCenterBytes = 0;
		if (!expectedPropertyBytes("center", decoded.centerEncoding, decoded.count, expectedCenterBytes) || decoded.center.size() != expectedCenterBytes)
			return false;

		splats.resize(decoded.count);
		const bool centerHalf = encodingTokenMatches(decoded.centerEncoding, "f16") && !encodingTokenMatches(decoded.centerEncoding, "f32");
		const bool centerBytePlanes = encodingTokenMatches(decoded.centerEncoding, "lebytes", "le_bytes");

		// Alpha decoding mode
		bool alphaF32 = encodingTokenMatches(decoded.alphaEncoding, "f32") || encodingTokenMatches(decoded.alphaEncoding, "float");
		bool alphaR8 = encodingTokenMatches(decoded.alphaEncoding, "r8");
		const bool alphaU16 = encodingTokenMatches(decoded.alphaEncoding, "u16");

		// RGB decoding mode
		bool rgbF32 = encodingTokenMatches(decoded.rgbEncoding, "f32") || encodingTokenMatches(decoded.rgbEncoding, "float");
		bool rgbF16 = encodingTokenMatches(decoded.rgbEncoding, "f16");
		bool rgbR8Delta = encodingTokenMatches(decoded.rgbEncoding, "r8_delta") || encodingTokenMatches(decoded.rgbEncoding, "delta");

		// Scales decoding mode
		bool scalesF32 = encodingTokenMatches(decoded.scalesEncoding, "f32") || encodingTokenMatches(decoded.scalesEncoding, "float");
		bool scalesLnF16 = encodingTokenMatches(decoded.scalesEncoding, "ln_f16");
		bool scalesF16 = encodingTokenMatches(decoded.scalesEncoding, "f16") && !scalesLnF16;

		// Orientation decoding mode
		bool orientF32 = encodingTokenMatches(decoded.orientationEncoding, "f32") || encodingTokenMatches(decoded.orientationEncoding, "float");
		bool orientF16 = encodingTokenMatches(decoded.orientationEncoding, "f16");

		// SH decoding modes
		bool sh1F32 = encodingTokenMatches(decoded.sh1Encoding, "f32") || encodingTokenMatches(decoded.sh1Encoding, "float");
		bool sh1F16 = encodingTokenMatches(decoded.sh1Encoding, "f16");
		bool sh1Delta = encodingTokenMatches(decoded.sh1Encoding, "s8_delta") || encodingTokenMatches(decoded.sh1Encoding, "delta");
		bool sh2F32 = encodingTokenMatches(decoded.sh2Encoding, "f32") || encodingTokenMatches(decoded.sh2Encoding, "float");
		bool sh2F16 = encodingTokenMatches(decoded.sh2Encoding, "f16");
		bool sh2Delta = encodingTokenMatches(decoded.sh2Encoding, "s8_delta") || encodingTokenMatches(decoded.sh2Encoding, "delta");
		bool sh3F32 = encodingTokenMatches(decoded.sh3Encoding, "f32") || encodingTokenMatches(decoded.sh3Encoding, "float");
		bool sh3F16 = encodingTokenMatches(decoded.sh3Encoding, "f16");
		bool sh3Delta = encodingTokenMatches(decoded.sh3Encoding, "s8_delta") || encodingTokenMatches(decoded.sh3Encoding, "delta");

		// SH label (clustering) mode
		bool useShLabel = !decoded.shLabel.empty();
		bool shLabelU32 = encodingTokenMatches(decoded.shLabelEncoding, "u32");

		// Quantization ranges
		float rgbMin = jsonFloat(decoded.header, "rgbMin", -0.014724731f);
		float rgbMax = jsonFloat(decoded.header, "rgbMax", 1.1445312f);
		propertyRange(decoded.properties, "rgb", rgbMin, rgbMax);
		float rgbScale = (rgbMax - rgbMin) / 255.0f;
		float lnMin = jsonFloat(decoded.header, "lnScaleMin", -12.0f);
		float lnMax = jsonFloat(decoded.header, "lnScaleMax", 9.0f);
		propertyRange(decoded.properties, "scales", lnMin, lnMax);
		float alphaScale = (decoded.alphaMax - decoded.alphaMin) / 255.0f;
		// Every axis uses the same quantization range. Evaluate each code once
		// per chunk instead of evaluating exp() three times per splat. Preserve
		// the existing fallback dispatch for partially supplied scalar formats.
		const bool useScale8 = !(scalesF32 && (int)decoded.scales.size() >= decoded.count * 12)
			&& !((scalesLnF16 || scalesF16) && (int)decoded.scales.size() >= decoded.count * 6)
			&& (int)decoded.scales.size() >= decoded.count * 3;
		std::array<float, 256> scale8{};
		if (useScale8)
			for (size_t code = 0; code < scale8.size(); ++code)
				scale8[code] = decodeScale8((uint8_t)code, lnMin, lnMax);

		// RGB delta accumulator
		uint8_t rgbLast[3] = {0, 0, 0};
		// SH delta accumulators
		int8_t sh1Last[9] = {};
		int8_t sh2Last[15] = {};
		int8_t sh3Last[21] = {};

		for (int i = 0; i < decoded.count; ++i) {
			DecodedSplat& out = splats[i];
			MI_GaussianPoint& g = out.point;

			// --- Center ---
			g.position = decodeCenter(decoded.center, decoded.count, centerHalf, centerBytePlanes, i);

			// --- Alpha ---
			float a = 1.0f;
			if (alphaR8 && (int)decoded.alpha.size() >= decoded.count) {
				a = std::max(0.0f, decoded.alphaMin + (float)decoded.alpha[i] * alphaScale);
			} else if (alphaF32 && (int)decoded.alpha.size() >= (i + 1) * 4) {
				float v; std::memcpy(&v, &decoded.alpha[i * 4], sizeof(float));
				a = std::max(0.0f, v);
			} else if (alphaU16 && (int)decoded.alpha.size() >= (i + 1) * 2) {
				a = std::max(0.0f, decoded.alphaMin + (float)readU16(decoded.alpha, i) / 65535.0f * (decoded.alphaMax - decoded.alphaMin));
			} else if ((int)decoded.alpha.size() >= (i + 1) * 2) {
				a = std::max(0.0f, halfToFloat(readU16(decoded.alpha, i)));
			}

			// --- RGB ---
			float r = 0.8f, gg = 0.8f, b = 0.8f;
			if (rgbF32 && (int)decoded.rgb.size() >= decoded.count * 12) {
				// F32: 3 x float32 per point, planar
				float rv, gv, bv;
				std::memcpy(&rv, &decoded.rgb[i * 4], sizeof(float));
				std::memcpy(&gv, &decoded.rgb[(decoded.count + i) * 4], sizeof(float));
				std::memcpy(&bv, &decoded.rgb[(decoded.count * 2 + i) * 4], sizeof(float));
				r = rv; gg = gv; b = bv;
			} else if (rgbF16 && (int)decoded.rgb.size() >= decoded.count * 6) {
				// F16: 3 x half-float per point, planar
				r = halfToFloat(readU16(decoded.rgb, 0 * decoded.count + i));
				gg = halfToFloat(readU16(decoded.rgb, 1 * decoded.count + i));
				b = halfToFloat(readU16(decoded.rgb, 2 * decoded.count + i));
			} else if ((int)decoded.rgb.size() >= decoded.count * 3) {
				// R8 or R8Delta: 3 x uint8 per point, planar
				if (rgbR8Delta) {
					rgbLast[0] = (uint8_t)(rgbLast[0] + planarU8(decoded.rgb, decoded.count, 0, i));
					rgbLast[1] = (uint8_t)(rgbLast[1] + planarU8(decoded.rgb, decoded.count, 1, i));
					rgbLast[2] = (uint8_t)(rgbLast[2] + planarU8(decoded.rgb, decoded.count, 2, i));
				} else {
					rgbLast[0] = planarU8(decoded.rgb, decoded.count, 0, i);
					rgbLast[1] = planarU8(decoded.rgb, decoded.count, 1, i);
					rgbLast[2] = planarU8(decoded.rgb, decoded.count, 2, i);
				}
				r = std::max(0.0f, rgbMin + rgbLast[0] * rgbScale);
				gg = std::max(0.0f, rgbMin + rgbLast[1] * rgbScale);
				b = std::max(0.0f, rgbMin + rgbLast[2] * rgbScale);
			}
			g.color.set(r, gg, b, a);

			// --- Scales ---
			if (scalesF32 && (int)decoded.scales.size() >= decoded.count * 12) {
				float sx, sy, sz;
				std::memcpy(&sx, &decoded.scales[i * 4], sizeof(float));
				std::memcpy(&sy, &decoded.scales[(decoded.count + i) * 4], sizeof(float));
				std::memcpy(&sz, &decoded.scales[(decoded.count * 2 + i) * 4], sizeof(float));
				g.scale.set(sx, sy, sz);
			} else if (scalesLnF16 && (int)decoded.scales.size() >= decoded.count * 6) {
				g.scale.set(std::exp(halfToFloat(readU16(decoded.scales, 0 * decoded.count + i))),
				            std::exp(halfToFloat(readU16(decoded.scales, 1 * decoded.count + i))),
				            std::exp(halfToFloat(readU16(decoded.scales, 2 * decoded.count + i))));
			} else if (scalesF16 && (int)decoded.scales.size() >= decoded.count * 6) {
				g.scale.set(std::exp(halfToFloat(readU16(decoded.scales, 0 * decoded.count + i))),
				            std::exp(halfToFloat(readU16(decoded.scales, 1 * decoded.count + i))),
				            std::exp(halfToFloat(readU16(decoded.scales, 2 * decoded.count + i))));
			} else if ((int)decoded.scales.size() >= decoded.count * 3) {
				g.scale.set(scale8[planarU8(decoded.scales, decoded.count, 0, i)],
				            scale8[planarU8(decoded.scales, decoded.count, 1, i)],
				            scale8[planarU8(decoded.scales, decoded.count, 2, i)]);
			} else {
				g.scale.set(0.01f, 0.01f, 0.01f);
			}

			// --- Orientation ---
			if (orientF32 && (int)decoded.orientation.size() >= decoded.count * 12) {
				float qx, qy, qz;
				std::memcpy(&qx, &decoded.orientation[i * 4], sizeof(float));
				std::memcpy(&qy, &decoded.orientation[(decoded.count + i) * 4], sizeof(float));
				std::memcpy(&qz, &decoded.orientation[(decoded.count * 2 + i) * 4], sizeof(float));
				g.rotation.set(qx, qy, qz, std::sqrt(std::max(0.0f, 1.0f - qx * qx - qy * qy - qz * qz)));
			} else if (orientF16 && (int)decoded.orientation.size() >= decoded.count * 6) {
				float qx = halfToFloat(readU16(decoded.orientation, 0 * decoded.count + i));
				float qy = halfToFloat(readU16(decoded.orientation, 1 * decoded.count + i));
				float qz = halfToFloat(readU16(decoded.orientation, 2 * decoded.count + i));
				g.rotation.set(qx, qy, qz, std::sqrt(std::max(0.0f, 1.0f - qx * qx - qy * qy - qz * qz)));
			} else if (!orientF32 && !orientF16 && (int)decoded.orientation.size() >= decoded.count * 3) {
				// Unlike scalar component planes, Oct88R8 stores one contiguous u/v/r triple per splat.
				g.rotation = decodeQuatOct888(decoded.orientation[i * 3],
				                              decoded.orientation[i * 3 + 1],
				                              decoded.orientation[i * 3 + 2]);
			} else {
				g.rotation.set(0.0f, 0.0f, 0.0f, 1.0f);
			}

			// --- SH ---
			std::memset(g.sh, 0, sizeof(g.sh));
			g.shOrder = 0;

			// SH Label (clustering): look up codebook
			// Codebook layout: sh1Codebook[label * 9 + k] where coeff=k/3, chan=k%3
			// Must remap to channel-first layout: g.sh[chan * 15 + coeff]
			if (useShLabel) {
				uint32_t label = shLabelU32 ? readU32(decoded.shLabel, i) : readU16(decoded.shLabel, i);
				if (!decoded.sh1Codebook.empty() && label < (uint32_t)decoded.shCodeCount
					&& decoded.sh1Codebook.size() >= (size_t)decoded.shCodeCount * 9) {
					for (int k = 0; k < 9; ++k) {
						int coeff = k / 3;
						int chan = k % 3;
						g.sh[chan * 15 + coeff] = decoded.sh1Codebook[label * 9 + k];
					}
					g.shOrder = 1;
				}
				if (!decoded.sh2Codebook.empty() && label < (uint32_t)decoded.shCodeCount
					&& decoded.sh2Codebook.size() >= (size_t)decoded.shCodeCount * 15) {
					for (int k = 0; k < 15; ++k) {
						int coeff = k / 3;
						int chan = k % 3;
						g.sh[chan * 15 + coeff + 3] = decoded.sh2Codebook[label * 15 + k];
					}
					g.shOrder = 2;
				}
				if (!decoded.sh3Codebook.empty() && label < (uint32_t)decoded.shCodeCount
					&& decoded.sh3Codebook.size() >= (size_t)decoded.shCodeCount * 21) {
					for (int k = 0; k < 21; ++k) {
						int coeff = k / 3;
						int chan = k % 3;
						g.sh[chan * 15 + coeff + 8] = decoded.sh3Codebook[label * 21 + k];
					}
					g.shOrder = 3;
				}
			}

			// SH1 direct (overrides codebook if present)
			if ((int)decoded.sh1.size() >= decoded.count * 9) {
				for (int k = 0; k < 9; ++k) {
					float value = 0.0f;
					if (sh1F32) {
						std::memcpy(&value, &decoded.sh1[(k * decoded.count + i) * 4], sizeof(float));
					} else if (sh1F16) {
						value = halfToFloat(readU16(decoded.sh1, k * decoded.count + i));
					} else {
						int8_t raw = (int8_t)planarU8(decoded.sh1, decoded.count, k, i);
						if (sh1Delta) { sh1Last[k] += raw; raw = sh1Last[k]; }
						value = (float)raw / 127.0f * decoded.sh1Max;
					}
					int coeff = k / 3;
					int chan = k % 3;
					g.sh[chan * 15 + coeff] = value;
				}
				g.shOrder = 1;
			}

			// SH2 direct
			if ((int)decoded.sh2.size() >= decoded.count * 15) {
				for (int k = 0; k < 15; ++k) {
					float value = 0.0f;
					if (sh2F32) {
						std::memcpy(&value, &decoded.sh2[(k * decoded.count + i) * 4], sizeof(float));
					} else if (sh2F16) {
						value = halfToFloat(readU16(decoded.sh2, k * decoded.count + i));
					} else {
						int8_t raw = (int8_t)planarU8(decoded.sh2, decoded.count, k, i);
						if (sh2Delta) { sh2Last[k] += raw; raw = sh2Last[k]; }
						value = (float)raw / 127.0f * decoded.sh2Max;
					}
					int coeff = k / 3;
					int chan = k % 3;
					g.sh[chan * 15 + coeff + 3] = value;
				}
				g.shOrder = 2;
			}

			// SH3 direct
			if ((int)decoded.sh3.size() >= decoded.count * 21) {
				for (int k = 0; k < 21; ++k) {
					float value = 0.0f;
					if (sh3F32) {
						std::memcpy(&value, &decoded.sh3[(k * decoded.count + i) * 4], sizeof(float));
					} else if (sh3F16) {
						value = halfToFloat(readU16(decoded.sh3, k * decoded.count + i));
					} else {
						int8_t raw = (int8_t)planarU8(decoded.sh3, decoded.count, k, i);
						if (sh3Delta) { sh3Last[k] += raw; raw = sh3Last[k]; }
						value = (float)raw / 127.0f * decoded.sh3Max;
					}
					int coeff = k / 3;
					int chan = k % 3;
					g.sh[chan * 15 + coeff + 8] = value;
				}
				g.shOrder = 3;
			}

			out.childCount = (int)decoded.childCountData.size() >= (i + 1) * 2 ? readU16(decoded.childCountData, i) : 0;
			out.childStart = (int)decoded.childStartData.size() >= (i + 1) * 4 ? readU32(decoded.childStartData, i) : 0;
			float avgScale = (g.scale.x() + g.scale.y() + g.scale.z()) / 3.0f;
			float expansion = 1.0f;
			if (a > 1.0f) {
				float opacityForLod = a * 4.0f - 3.0f;
				expansion = 1.0f + 0.7f * (opacityForLod - 1.0f);
			}
			out.lodSize = 2.0f * expansion * avgScale;
		}
		return true;
	}

	bool prepareChunk(const std::vector<uint8_t>& chunk, int fallbackCount, int fallbackBase,
	                  const PrepareOptions& options, PreparedChunk& out)
	{
		out = PreparedChunk();
		const bool paged = options.maxPageSplats > 0;
		const bool packedEncoding = options.splatEncoding == (int)SplatEncoding::PackedSplat ||
			options.splatEncoding == (int)SplatEncoding::ExtSplat;
		if (options.totalPointCount <= 0 || options.maxPageSplats < 0 ||
			(paged ? !packedEncoding : options.splatEncoding != (int)SplatEncoding::Legacy))
			return false;

		PreparedChunk prepared;
		using Clock = std::chrono::steady_clock;
		auto stageStart = options.timingsEnabled ? Clock::now() : Clock::time_point();
		auto recordStage = [&](double& milliseconds) {
			if (options.timingsEnabled) {
				const auto now = Clock::now();
				milliseconds = std::chrono::duration<double, std::milli>(now - stageStart).count();
				stageStart = now;
			}
		};
		DecodedChunk decoded;
		if (!decodeChunkPayload(chunk, fallbackCount, fallbackBase, decoded))
			return false;
		recordStage(prepared.decodeMilliseconds);
		// Check the decoded header, which may override the table's fallback
		// count/base, before allocating the much larger per-splat objects.
		if (decoded.base < 0 || decoded.count <= 0 || decoded.base >= options.totalPointCount ||
			decoded.count > options.totalPointCount - decoded.base ||
			(paged && decoded.count > options.maxPageSplats))
			return false;
		if (!convertDecodedChunkToSplats(decoded, prepared.splats) ||
			prepared.splats.size() != size_t(decoded.count))
			return false;
		recordStage(prepared.convertMilliseconds);

		if (paged) {
			const size_t wordsPerSplat = options.splatEncoding == (int)SplatEncoding::PackedSplat ? 4 : 8;
			if (!encodeSplatsToPacked(prepared.splats, options.splatEncoding, prepared.packedData) ||
				prepared.packedData.size() != size_t(decoded.count) * wordsPerSplat)
				return false;
			const bool hasSH = std::any_of(prepared.splats.begin(), prepared.splats.end(),
				[](const DecodedSplat& splat) { return splat.point.shOrder > 0; });
			if (hasSH) {
				prepared.shData.assign(size_t(decoded.count) * 48, 0.0f);
				for (size_t i = 0; i < prepared.splats.size(); ++i)
					std::copy_n(prepared.splats[i].point.sh, 45, prepared.shData.data() + i * 48);
			}
			recordStage(prepared.packMilliseconds);
		}
		prepared.count = decoded.count;
		prepared.base = decoded.base;
		prepared.payloadBytes = decoded.payloadBytes;
		prepared.properties = std::move(decoded.properties);
		prepared.encoding = options.splatEncoding;
		out = std::move(prepared);
		return true;
	}
#endif
}
