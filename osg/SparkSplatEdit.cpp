#include "SparkSplatEdit.h"

#include <cstring>
#include <iostream>

#ifndef GL_RGBA_INTEGER
#define GL_RGBA_INTEGER 0x8D99
#endif
#ifndef GL_UNSIGNED_INT
#define GL_UNSIGNED_INT 0x1405
#endif

SparkSplatEdits::SparkSplatEdits(int maxSdfs, int maxEdits)
	: _maxSdfs(maxSdfs), _maxEdits(maxEdits)
{
}

uint32_t SparkSplatEdits::floatToUintBits(float f) const
{
	uint32_t u;
	std::memcpy(&u, &f, sizeof(u));
	return u;
}

int SparkSplatEdits::addEdit(const SplatEdit& edit)
{
	if ((int)_edits.size() >= _maxEdits) {
		std::cerr << "SparkSplatEdits: max edits reached (" << _maxEdits << ")" << std::endl;
		return -1;
	}

	int sdfFirst = (int)_allSdfs.size();
	int sdfCount = (int)edit.sdfs.size();

	if (sdfFirst + sdfCount > _maxSdfs) {
		std::cerr << "SparkSplatEdits: max SDFs reached (" << _maxSdfs << ")" << std::endl;
		return -1;
	}

	// Append SDFs
	for (const SplatEditSdf& sdf : edit.sdfs)
		_allSdfs.push_back(sdf);

	// Store edit metadata
	InternalEdit ie;
	ie.rgbaBlendMode = edit.rgbaBlendMode;
	ie.sdfSmooth = edit.sdfSmooth;
	ie.softEdge = edit.softEdge;
	ie.invert = edit.invert;
	ie.sdfFirst = sdfFirst;
	ie.sdfCount = sdfCount;
	_edits.push_back(ie);

	_dirty = true;
	return (int)_edits.size() - 1;
}

void SparkSplatEdits::clear()
{
	_allSdfs.clear();
	_edits.clear();
	_sdfData.clear();
	_editData.clear();
	_numSdfs = 0;
	_numEdits = 0;
	_dirty = true;
}

void SparkSplatEdits::update()
{
	if (!_dirty) return;

	_numSdfs = (int)_allSdfs.size();
	_numEdits = (int)_edits.size();

	// Encode SDF data: 8 texels of uvec4 = 32 uint32 per SDF
	_sdfData.resize(_numSdfs * 32, 0);

	for (int i = 0; i < _numSdfs; ++i) {
		const SplatEditSdf& sdf = _allSdfs[i];
		uint32_t* texel = &_sdfData[i * 32];

		// texel[0]: center.xyz as float bits + flags in .w
		texel[0] = floatToUintBits(sdf.center.x());  // texel[0].x
		texel[1] = floatToUintBits(sdf.center.y());  // texel[0].y
		texel[2] = floatToUintBits(sdf.center.z());  // texel[0].z
		uint32_t flags = (uint32_t)sdf.type | ((sdf.invert ? 1u : 0u) << 8u);
		texel[3] = flags;                             // texel[0].w

		// texel[1]: quaternion.xyzw as float bits
		texel[4] = floatToUintBits(sdf.quaternion.x());  // texel[1].x
		texel[5] = floatToUintBits(sdf.quaternion.y());  // texel[1].y
		texel[6] = floatToUintBits(sdf.quaternion.z());  // texel[1].z
		texel[7] = floatToUintBits(sdf.quaternion.w());  // texel[1].w

		// texel[2]: scale.xyz as float bits + 0 in .w
		texel[8]  = floatToUintBits(sdf.scale.x());  // texel[2].x
		texel[9]  = floatToUintBits(sdf.scale.y());  // texel[2].y
		texel[10] = floatToUintBits(sdf.scale.z());  // texel[2].z
		texel[11] = 0u;                              // texel[2].w

		// texel[3]: sizes.xyzw as float bits
		// sizes.x = opacity, sizes.y = radius, sizes.z = 0, sizes.w = 0
		texel[12] = floatToUintBits(sdf.opacity);  // texel[3].x
		texel[13] = floatToUintBits(sdf.radius);   // texel[3].y
		texel[14] = 0u;                            // texel[3].z
		texel[15] = 0u;                            // texel[3].w

		// texel[4]: values[0] = (color.r, color.g, color.b, color.a) as float bits
		texel[16] = floatToUintBits(sdf.color.r());  // texel[4].x
		texel[17] = floatToUintBits(sdf.color.g());  // texel[4].y
		texel[18] = floatToUintBits(sdf.color.b());  // texel[4].z
		texel[19] = floatToUintBits(sdf.color.a());  // texel[4].w

		// texel[5]: values[1] = (displace.x, displace.y, displace.z, 0) as float bits
		texel[20] = floatToUintBits(sdf.displace.x());  // texel[5].x
		texel[21] = floatToUintBits(sdf.displace.y());  // texel[5].y
		texel[22] = floatToUintBits(sdf.displace.z());  // texel[5].z
		texel[23] = 0u;                                  // texel[5].w

		// texel[6-7]: values[2-3] = (0,0,0,0) unused
		texel[24] = 0u; texel[25] = 0u; texel[26] = 0u; texel[27] = 0u;
		texel[28] = 0u; texel[29] = 0u; texel[30] = 0u; texel[31] = 0u;
	}

	// Encode Edit data: 4 uint32 per edit (1 uvec4)
	_editData.resize(_numEdits * 4, 0);

	for (int i = 0; i < _numEdits; ++i) {
		const InternalEdit& ie = _edits[i];

		// x: rgbaBlendMode | (invert << 8)
		uint32_t x = (uint32_t)ie.rgbaBlendMode | ((ie.invert ? 1u : 0u) << 8u);

		// y: sdfFirst | (sdfCount << 16)
		uint32_t y = (uint32_t)ie.sdfFirst | ((uint32_t)ie.sdfCount << 16u);

		// z: softEdge as float bits
		uint32_t z = floatToUintBits(ie.softEdge);

		// w: sdfSmooth as float bits
		uint32_t w = floatToUintBits(ie.sdfSmooth);

		_editData[i * 4 + 0] = x;
		_editData[i * 4 + 1] = y;
		_editData[i * 4 + 2] = z;
		_editData[i * 4 + 3] = w;
	}

	_dirty = false;
}

osg::Image* SparkSplatEdits::sdfTextureImage()
{
	update(); // ensure data is current before checking _numSdfs
	if (_numSdfs == 0) return nullptr;

	if (!_sdfImage || _sdfImage->s() != 8 || _sdfImage->t() != _numSdfs) {
		_sdfImage = new osg::Image;
		_sdfImage->allocateImage(8, _numSdfs, 1, GL_RGBA_INTEGER, GL_UNSIGNED_INT);
		_sdfImage->setInternalTextureFormat(0x8D70); // GL_RGBA32UI = 0x8D70
	}

	// Copy encoded SDF data into the image
	size_t bytesNeeded = (size_t)_numSdfs * 32 * sizeof(uint32_t);
	if (_sdfImage->getTotalSizeInBytesIncludingMipmaps() >= (int)bytesNeeded) {
		std::memcpy(_sdfImage->data(), _sdfData.data(), bytesNeeded);
	}

	return _sdfImage.get();
}
