#ifndef SPARK_SPLAT_EDIT_H
#define SPARK_SPLAT_EDIT_H

#include <osg/Vec3f>
#include <osg/Vec4f>
#include <osg/ref_ptr>
#include <osg/Image>

#include <cstdint>
#include <vector>

// SDF primitive types (matching Spark's SplatEditSdfType)
enum SplatEditSdfType
{
	SPLAT_EDIT_SDF_ALL = 0,
	SPLAT_EDIT_SDF_PLANE = 1,
	SPLAT_EDIT_SDF_SPHERE = 2,
	SPLAT_EDIT_SDF_BOX = 3,
	SPLAT_EDIT_SDF_ELLIPSOID = 4,
	SPLAT_EDIT_SDF_CYLINDER = 5,
	SPLAT_EDIT_SDF_CAPSULE = 6,
	SPLAT_EDIT_SDF_INFINITE_CONE = 7
};

// RGBA blend modes (matching Spark's SplatEditRgbaBlendMode)
enum SplatEditRgbaBlendMode
{
	SPLAT_EDIT_BLEND_MULTIPLY = 0,
	SPLAT_EDIT_BLEND_SET_RGB = 1,
	SPLAT_EDIT_BLEND_ADD_RGBA = 2
};

// Single SDF primitive definition
struct SplatEditSdf
{
	SplatEditSdfType type = SPLAT_EDIT_SDF_ALL;
	bool invert = false;
	osg::Vec3f center = osg::Vec3f(0.0f, 0.0f, 0.0f);
	osg::Vec4f quaternion = osg::Vec4f(0.0f, 0.0f, 0.0f, 1.0f); // (x, y, z, w)
	osg::Vec3f scale = osg::Vec3f(1.0f, 1.0f, 1.0f);
	float radius = 1.0f;
	float opacity = 1.0f;
	osg::Vec4f color = osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f); // RGBA edit color
	osg::Vec3f displace = osg::Vec3f(0.0f, 0.0f, 0.0f);      // XYZ displacement
};

// An edit operation: combines multiple SDFs with a blend mode
struct SplatEdit
{
	SplatEditRgbaBlendMode rgbaBlendMode = SPLAT_EDIT_BLEND_MULTIPLY;
	float sdfSmooth = 0.0f;  // smoothing between SDF shapes
	float softEdge = 0.0f;   // soft edge width
	bool invert = false;
	std::vector<SplatEditSdf> sdfs;
};

// Manages a collection of SplatEdits and encodes them into GPU-ready data
// matching Spark's SplatEdit.ts encoding format
class SparkSplatEdits
{
public:
	SparkSplatEdits(int maxSdfs = 64, int maxEdits = 8);

	// Add an edit (with its SDFs). Returns the edit index or -1 on overflow.
	int addEdit(const SplatEdit& edit);

	// Clear all edits
	void clear();

	// Compile all SDF/Edit data into GPU-ready arrays.
	// Must be called after addEdit() before reading GPU data.
	void update();

	// Access encoded GPU data (valid after update())
	const std::vector<uint32_t>& sdfData() const { return _sdfData; }
	const std::vector<uint32_t>& editData() const { return _editData; } // 4 uint32 per edit
	int numSdfs() const { return _numSdfs; }
	int numEdits() const { return _numEdits; }

	// Create or return the SDF texture image (RGBA32UI, width=8, height=numSdfs)
	osg::Image* sdfTextureImage();

	// Mark data as needing re-upload
	void dirty() { _dirty = true; }

private:
	uint32_t floatToUintBits(float f) const;

	int _maxSdfs;
	int _maxEdits;

	// Internal storage
	struct InternalEdit
	{
		SplatEditRgbaBlendMode rgbaBlendMode;
		float sdfSmooth;
		float softEdge;
		bool invert;
		int sdfFirst;  // index of first SDF in the flat array
		int sdfCount;  // number of SDFs for this edit
	};

	std::vector<SplatEditSdf> _allSdfs;
	std::vector<InternalEdit> _edits;

	// GPU-ready encoded data
	std::vector<uint32_t> _sdfData;       // 32 uint32 per SDF (8 texels * 4 components)
	std::vector<uint32_t> _editData;      // 4 uint32 per edit (1 uvec4)
	int _numSdfs = 0;
	int _numEdits = 0;

	osg::ref_ptr<osg::Image> _sdfImage;
	bool _dirty = true;
};

#endif // SPARK_SPLAT_EDIT_H
