#include "GaussianDrawObj.h"

#include "osg/Material"
#include "osg/Geode"
#include "osg/Geometry"
#include "osg/TextureBuffer"
#include "osg/MatrixTransform"
#include "osgUtil/CullVisitor"
#include "osg/ComputeBoundsVisitor"
#include "osgDB/ReadFile"
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include "tools/miniply.h"
#include "tools/tools.h"

#include "gf/core/gauss_ir.h"
#include "gf/io/registry.h"
#include "gf/io/reader.h"

#include <algorithm>
#include <numeric>
#include <cmath>
#include <cfloat>
#include <cstring>
#include <fstream>
#include <iostream>

// ======================== GaussianSortThread ========================

GaussianSortThread::GaussianSortThread() {}

GaussianSortThread::~GaussianSortThread() { stop(); }

void GaussianSortThread::start()
{
	_running = true;
	_thread = std::thread(&GaussianSortThread::run, this);
}

void GaussianSortThread::stop()
{
	_running = false;
	_cv.notify_one();
	if (_thread.joinable())
		_thread.join();
}

void GaussianSortThread::requestSort(const osg::Matrix& modelView)
{
	{
		std::lock_guard<std::mutex> lock(_taskMutex);
		_taskModelView = modelView;
		_hasTask = true;
	}
	_cv.notify_one();
}

void GaussianSortThread::setPositions(const std::vector<MI_GaussianPoint>* points, int numPoints)
{
	_points = points;
	_numPoints = numPoints;
}

bool GaussianSortThread::fetchResult(std::vector<int>& outIndices)
{
	if (!_hasResult) return false;
	std::lock_guard<std::mutex> lock(_resultMutex);
	outIndices.swap(_resultIndices);
	_hasResult = false;
	return true;
}

void GaussianSortThread::run()
{
	while (_running)
	{
		{
			std::unique_lock<std::mutex> lock(_taskMutex);
			_cv.wait(lock, [this] { return _hasTask.load() || !_running.load(); });
		}

		if (!_running) break;
		if (!_hasTask) continue;

		osg::Matrix localMV;
		int localNum;
		{
			std::lock_guard<std::mutex> lock(_taskMutex);
			localMV = _taskModelView;
			localNum = _numPoints;
			_hasTask = false;
		}

		if (!_points || localNum <= 0) continue;

		// Radix sort by reinterpreting float depth as uint32
		std::vector<uint32_t> keys(localNum);
		std::vector<int> values(localNum);

		for (int i = 0; i < localNum; ++i)
		{
			const osg::Vec3f& p = (*_points)[i].position;
			float ez = (float)(p.x() * localMV(0, 2) + p.y() * localMV(1, 2) +
			                    p.z() * localMV(2, 2) + localMV(3, 2));
			float d = (ez > 0.0f) ? FLT_MAX : -ez;
			union { float f; uint32_t u; } un;
			un.f = d;
			keys[i] = un.u;
			values[i] = i;
		}

		// 8-bit radix sort (4 passes for 32-bit keys)
		const int RADIX_BITS = 8;
		const int RADIX_SIZE = 1 << RADIX_BITS;
		const int RADIX_MASK = RADIX_SIZE - 1;

		std::vector<uint32_t> tempKeys(localNum);
		std::vector<int> tempValues(localNum);

		for (int pass = 0; pass < 4; ++pass)
		{
			int shift = pass * RADIX_BITS;
			std::vector<int> count(RADIX_SIZE, 0);

			for (int i = 0; i < localNum; ++i)
				++count[(keys[i] >> shift) & RADIX_MASK];

			for (int i = 1; i < RADIX_SIZE; ++i)
				count[i] += count[i - 1];

			for (int i = localNum - 1; i >= 0; --i)
			{
				int bucket = (keys[i] >> shift) & RADIX_MASK;
				int dest = --count[bucket];
				tempKeys[dest] = keys[i];
				tempValues[dest] = values[i];
			}

			keys.swap(tempKeys);
			values.swap(tempValues);
		}

		// Reverse for back-to-front ordering
		std::vector<int> sorted(localNum);
		for (int i = 0; i < localNum; ++i)
			sorted[i] = values[localNum - 1 - i];

		{
			std::lock_guard<std::mutex> lock(_resultMutex);
			_resultIndices.swap(sorted);
			_hasResult = true;
		}
	}
}

// ======================== Callbacks ========================

class GaussianSortCallback : public osg::Camera::DrawCallback
{
public:
	GaussianSortCallback(GaussianDrawObj* obj) : _obj(obj) {}

	virtual void operator()(osg::RenderInfo& renderInfo) const
	{
		if (!_obj || !renderInfo.getCurrentCamera()) return;
		osg::Matrix viewMatrix = renderInfo.getCurrentCamera()->getViewMatrix();

		if (_obj->shouldSort(viewMatrix))
			_obj->requestSort();
	}

private:
	GaussianDrawObj* _obj;
};

class SSCallback : public osg::StateSet::Callback
{
public:
	SSCallback(GaussianDrawObj* obj) : _obj(obj) {}

	virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
	{
		if (!ss) return;
		osg::Camera* camera = getCamera();
		if (!camera) return;

		osg::Matrixf viewMatrix = camera->getViewMatrix();
		ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(viewMatrix);

		osg::Matrixf projMatrix = camera->getProjectionMatrix();
		ss->getOrCreateUniform("proj", osg::Uniform::FLOAT_MAT4)->set(projMatrix);

		osg::Vec2 vs;
		vs[0] = camera->getViewport()->width();
		vs[1] = camera->getViewport()->height();
		ss->getOrCreateUniform("viewport_size", osg::Uniform::FLOAT_VEC2)->set(vs);
	}

private:
	GaussianDrawObj* _obj;
};

class IndexBufferUpdateCallback : public osg::StateAttributeCallback
{
public:
	IndexBufferUpdateCallback(GaussianDrawObj* obj) : _obj(obj) {}

	virtual void operator()(osg::StateAttribute* sa, osg::NodeVisitor* nv)
	{
		if (!_obj) return;
		osg::TextureBuffer* tbo = dynamic_cast<osg::TextureBuffer*>(sa);
		if (!tbo) return;
		osg::Image* indexImage = tbo->getImage();
		if (!indexImage) return;

		_obj->applySortResult(indexImage);
	}

private:
	GaussianDrawObj* _obj;
};

// ======================== GaussianDrawObj ========================

GaussianDrawObj::GaussianDrawObj(const std::string& name)
{
	// Determine file extension
	std::string ext;
	size_t dotPos = name.rfind('.');
	if (dotPos != std::string::npos)
		ext = name.substr(dotPos + 1);

	// Use GaussForge for all supported formats
	if (ext == "spz" || ext == "ksplat")
	{
		gaussianPoints = readWithGaussForge(name, ext);
	}
	else if (ext == "ply")
	{
		// Try GaussForge first, fall back to miniply
		gaussianPoints = readWithGaussForge(name, ext);
		if (gaussianPoints.empty())
			gaussianPoints = readPlyFile(name);
	}
	else if (ext == "splat")
	{
		gaussianPoints = readWithGaussForge(name, ext);
		if (gaussianPoints.empty())
			gaussianPoints = readSplatFile(name);
	}
	else
	{
		// Try GaussForge for any unknown extension
		gaussianPoints = readWithGaussForge(name, ext);
	}

	nNum = (int)gaussianPoints.size();
	depthIndex.resize(nNum);
	std::iota(depthIndex.begin(), depthIndex.end(), 0);

	_sortThread.setPositions(&gaussianPoints, nNum);
	_sortThread.start();
}

GaussianDrawObj::~GaussianDrawObj()
{
	_sortThread.stop();
}

bool GaussianDrawObj::shouldSort(const osg::Matrix& currentView)
{
	if (_firstSort) return true;

	// Compare view matrices — skip sort if camera hasn't moved
	for (int i = 0; i < 4; ++i)
		for (int j = 0; j < 4; ++j)
			if (std::abs(_lastViewMatrix(i, j) - currentView(i, j)) > 1e-5)
				return true;
	return false;
}

void GaussianDrawObj::requestSort()
{
	osg::Camera* cam = getCamera();
	if (!cam) return;

	osg::Matrix viewMatrix = cam->getViewMatrix();
	_lastViewMatrix = viewMatrix;
	_firstSort = false;

	_sortThread.requestSort(viewMatrix);
}

bool GaussianDrawObj::applySortResult(osg::Image* indexImage)
{
	std::vector<int> newIndices;
	if (!_sortThread.fetchResult(newIndices))
		return false;

	depthIndex.swap(newIndices);
	updateIndexImage(indexImage);
	return true;
}

void GaussianDrawObj::updateDataImage(osg::Image* paramsImage)
{
	// Per-splat data: 4 Vec4f
	//   [0] position.xyz + alpha
	//   [1] scale.xyz + color.r
	//   [2] rotation.xyz + color.g
	//   [3] rotation.w + 0 + 0 + color.b
	const int stride = 4;
	for (int i = 0; i < nNum; i++)
	{
		const MI_GaussianPoint& g = gaussianPoints[i];
		osg::Vec4f* ptr = (osg::Vec4f*)paramsImage->data(stride * i);
		if (!ptr) continue;

		ptr[0].set(g.position.x(), g.position.y(), g.position.z(), g.color.a());
		ptr[1].set(g.scale.x(), g.scale.y(), g.scale.z(), g.color.r());
		ptr[2].set(g.rotation.x(), g.rotation.y(), g.rotation.z(), g.color.g());
		ptr[3].set(g.rotation.w(), 0.0f, 0.0f, g.color.b());
	}
	paramsImage->dirty();
}

void GaussianDrawObj::updateSHImage(osg::Image* paramsImage)
{
	const int shStride = 12;
	for (int i = 0; i < nNum; i++)
	{
		const MI_GaussianPoint& g = gaussianPoints[i];
		osg::Vec4f* ptr = (osg::Vec4f*)paramsImage->data(shStride * i);
		if (!ptr) continue;

		float* shPtr = (float*)ptr;
		for (int k = 0; k < 45; ++k)
			shPtr[k] = g.sh[k];
		shPtr[45] = shPtr[46] = shPtr[47] = 0.0f;
	}
	paramsImage->dirty();
}

void GaussianDrawObj::updateIndexImage(osg::Image* paramsImage)
{
	osg::Vec4f* ptr = (osg::Vec4f*)paramsImage->data(0);
	for (int i = 0; i < nNum; i++)
	{
		ptr->set((float)depthIndex[i], 0, 0, 0);
		ptr++;
	}
	paramsImage->dirty();
}

void GaussianDrawObj::loadShader(osg::StateSet* ss)
{
	osg::ref_ptr<osg::Program> program = new osg::Program;
	std::string strDir = osg_tools::getAppDir() + "/shader/";
	osg::Shader* vs = osgDB::readShaderFile(osg::Shader::VERTEX, strDir + "gaussian.vert");
	osg::Shader* fs = osgDB::readShaderFile(osg::Shader::FRAGMENT, strDir + "gaussian.frag");
	if (vs) program->addShader(vs);
	if (fs) program->addShader(fs);
	ss->setAttribute(program);
}

osg::ref_ptr<osg::Node> GaussianDrawObj::getNode()
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	osg::ref_ptr<osg::Geometry> pInstance = createQuadGeometry();
	geode->addChild(pInstance);

	// Index buffer TBO
	{
		osg::ref_ptr<osg::Image> indexImg = new osg::Image;
		indexImg->allocateImage(nNum, 1, 1, GL_RGBA, GL_FLOAT);
		updateIndexImage(indexImg);

		osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
		tbo->setImage(indexImg.get());
		tbo->setInternalFormat(GL_RGBA32F_ARB);
		pInstance->getOrCreateStateSet()->setTextureAttribute(0, tbo.get());
		tbo->setUpdateCallback(new IndexBufferUpdateCallback(this));
	}

	// Data buffer TBO (4 Vec4f per splat: pos+alpha, scale+r, quat.xyz+g, quat.w+b)
	{
		osg::ref_ptr<osg::Image> dataImg = new osg::Image;
		dataImg->allocateImage(4 * nNum, 1, 1, GL_RGBA, GL_FLOAT);
		updateDataImage(dataImg);

		osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
		tbo->setImage(dataImg.get());
		tbo->setInternalFormat(GL_RGBA32F_ARB);
		pInstance->getOrCreateStateSet()->setTextureAttribute(1, tbo.get());
	}

	// SH buffer TBO
	if (hasSH)
	{
		osg::ref_ptr<osg::Image> shImg = new osg::Image;
		shImg->allocateImage(12 * nNum, 1, 1, GL_RGBA, GL_FLOAT);
		updateSHImage(shImg);

		osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
		tbo->setImage(shImg.get());
		tbo->setInternalFormat(GL_RGBA32F_ARB);
		pInstance->getOrCreateStateSet()->setTextureAttribute(2, tbo.get());
	}

	// State setup
	osg::StateSet* ss = pInstance->getOrCreateStateSet();
	loadShader(ss);
	ss->setUpdateCallback(new SSCallback(this));

	ss->addUniform(new osg::Uniform("indexBuffer", 0));
	ss->addUniform(new osg::Uniform("dataBuffer", 1));
	ss->addUniform(new osg::Uniform("shBuffer", 2));
	ss->addUniform(new osg::Uniform("hasSH", hasSH));

	// Disable depth test/write
	ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

	// Premultiplied alpha blending: GL_ONE, GL_ONE_MINUS_SRC_ALPHA
	ss->setMode(GL_BLEND, osg::StateAttribute::ON);
	osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc;
	blendFunc->setFunction(osg::BlendFunc::ONE, osg::BlendFunc::ONE_MINUS_SRC_ALPHA);
	ss->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);

	// Register pre-draw callback for automatic sorting
	osg::Camera* cam = getCamera();
	if (cam)
		cam->setPreDrawCallback(new GaussianSortCallback(this));

	return geode;
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readSplatFile(const std::string& filename)
{
	std::vector<MI_GaussianPoint> points;
	std::ifstream file(filename, std::ios::binary);
	if (!file)
	{
		std::cerr << "Failed to open file: " << filename << std::endl;
		return points;
	}

	struct Splat {
		float pos[3];
		float scale[3];
		uint8_t color[4];
		uint8_t rot[4];
	};

	Splat splat;
	while (file.read(reinterpret_cast<char*>(&splat), sizeof(Splat)))
	{
		MI_GaussianPoint g;
		g.position.set(splat.pos[0], splat.pos[1], splat.pos[2]);

		g.color.set(splat.color[0] / 255.0f, splat.color[1] / 255.0f,
		            splat.color[2] / 255.0f, splat.color[3] / 255.0f);

		// Store raw scale (already in linear space for .splat format)
		g.scale.set(splat.scale[0], splat.scale[1], splat.scale[2]);

		// Decode quaternion from uint8
		float rx = (splat.rot[0] - 128.0f) / 128.0f;
		float ry = (splat.rot[1] - 128.0f) / 128.0f;
		float rz = (splat.rot[2] - 128.0f) / 128.0f;
		float rw = (splat.rot[3] - 128.0f) / 128.0f;

		// Normalize quaternion
		float len = std::sqrt(rx * rx + ry * ry + rz * rz + rw * rw);
		if (len > 0.0f) { rx /= len; ry /= len; rz /= len; rw /= len; }
		g.rotation.set(rx, ry, rz, rw);

		bounds.expandBy(g.position);
		points.push_back(g);
	}

	std::cout << "Loaded " << points.size() << " Gaussian points from .splat" << std::endl;
	return points;
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readPlyFile(const std::string& ply_path)
{
	std::vector<MI_GaussianPoint> points;

	const float SH_C0 = 0.28209479177387814f;

	const std::vector<std::string> properties = {
		"x", "y", "z",
		"f_dc_0", "f_dc_1", "f_dc_2",
		"opacity",
		"scale_0", "scale_1", "scale_2",
		"rot_0", "rot_1", "rot_2", "rot_3",
	};

	std::vector<std::string> shProperties;
	shProperties.reserve(45);
	for (int i = 0; i < 45; ++i)
		shProperties.push_back("f_rest_" + std::to_string(i));

	miniply::PLYReader reader(ply_path.c_str());
	if (!reader.valid()) {
		std::cerr << "Failed to open " << ply_path << std::endl;
		return points;
	}

	miniply::PLYElement* elem = reader.get_element(reader.find_element("vertex"));
	if (!elem) {
		std::cerr << "No vertex element found" << std::endl;
		return points;
	}

	size_t num_gaussians = reader.num_rows();
	points.reserve(num_gaussians);

	std::vector<uint32_t> propIdx(properties.size());
	for (size_t i = 0; i < properties.size(); ++i)
		propIdx[i] = elem->find_property(properties[i].c_str());

	std::vector<uint32_t> validShIdx;
	std::vector<int> validShPos;
	for (int i = 0; i < 45; ++i) {
		uint32_t idx = elem->find_property(shProperties[i].c_str());
		if (idx != miniply::kInvalidIndex) {
			validShIdx.push_back(idx);
			validShPos.push_back(i);
		}
	}
	bool fileSH = !validShIdx.empty();

	reader.load_element();
	std::vector<float> filedata(properties.size() * num_gaussians);
	reader.extract_properties(propIdx.data(), (uint32_t)properties.size(),
	                          miniply::PLYPropertyType::Float, filedata.data());

	std::vector<float> shdata;
	if (fileSH) {
		int nValid = (int)validShIdx.size();
		std::vector<float> rawSH(nValid * num_gaussians, 0.0f);
		reader.extract_properties(validShIdx.data(), nValid,
		                          miniply::PLYPropertyType::Float, rawSH.data());
		shdata.resize(45 * num_gaussians, 0.0f);
		for (size_t i = 0; i < num_gaussians; ++i)
			for (int k = 0; k < nValid; ++k)
				shdata[i * 45 + validShPos[k]] = rawSH[i * nValid + k];
	}

	for (size_t i = 0; i < num_gaussians; ++i)
	{
		int offset = (int)i * (int)properties.size();
		MI_GaussianPoint g;

		g.position.set(filedata[offset + 0], filedata[offset + 1], filedata[offset + 2]);

		// DC color from SH_C0
		g.color.set(
			filedata[offset + 3] * SH_C0 + 0.5f,
			filedata[offset + 4] * SH_C0 + 0.5f,
			filedata[offset + 5] * SH_C0 + 0.5f,
			1.0f / (1.0f + std::exp(-filedata[offset + 6]))
		);

		// Store scale in log-space exponentiated
		g.scale.set(
			std::exp(filedata[offset + 7]),
			std::exp(filedata[offset + 8]),
			std::exp(filedata[offset + 9])
		);

		// Quaternion (w, x, y, z) — normalize
		float qw = filedata[offset + 10];
		float qx = filedata[offset + 11];
		float qy = filedata[offset + 12];
		float qz = filedata[offset + 13];
		float qlen = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		if (qlen > 0.0f) { qw /= qlen; qx /= qlen; qy /= qlen; qz /= qlen; }
		g.rotation.set(qx, qy, qz, qw);

		if (fileSH) {
			int shOffset = (int)i * 45;
			for (int k = 0; k < 45; ++k)
				g.sh[k] = shdata[shOffset + k];
		}

		bounds.expandBy(g.position);
		points.push_back(g);
	}

	this->hasSH = fileSH;

	std::cout << "Loaded " << points.size() << " Gaussian points from .ply" << std::endl;
	return points;
}

std::vector<MI_GaussianPoint> GaussianDrawObj::readWithGaussForge(
	const std::string& filename, const std::string& ext)
{
	std::vector<MI_GaussianPoint> points;

	std::ifstream file(filename, std::ios::binary | std::ios::ate);
	if (!file) {
		std::cerr << "Failed to open file: " << filename << std::endl;
		return points;
	}

	size_t fileSize = (size_t)file.tellg();
	file.seekg(0);
	std::vector<uint8_t> buffer(fileSize);
	file.read(reinterpret_cast<char*>(buffer.data()), fileSize);
	file.close();

	gf::IORegistry registry;
	gf::IGaussReader* reader = registry.ReaderForExt(ext);
	if (!reader) {
		std::cerr << "No GaussForge reader for extension: " << ext << std::endl;
		return points;
	}

	gf::ReadOptions opts;
	auto result = reader->Read(buffer.data(), buffer.size(), opts);
	if (!result.ok()) {
		std::cerr << "GaussForge read error: " << result.error().message << std::endl;
		return points;
	}

	gf::GaussianCloudIR& ir = result.value();
	int N = ir.numPoints;
	int shPerPoint = (N > 0 && !ir.sh.empty()) ? (int)(ir.sh.size() / N) : 0;

	points.reserve(N);
	const float SH_C0 = 0.28209479177387814f;

	for (int i = 0; i < N; ++i)
	{
		MI_GaussianPoint g;

		g.position.set(
			ir.positions[i * 3 + 0],
			ir.positions[i * 3 + 1],
			ir.positions[i * 3 + 2]
		);

		g.scale.set(
			std::exp(ir.scales[i * 3 + 0]),
			std::exp(ir.scales[i * 3 + 1]),
			std::exp(ir.scales[i * 3 + 2])
		);

		// IR stores quaternion as [w, x, y, z], we store as [x, y, z, w]
		float qw = ir.rotations[i * 4 + 0];
		float qx = ir.rotations[i * 4 + 1];
		float qy = ir.rotations[i * 4 + 2];
		float qz = ir.rotations[i * 4 + 3];
		float qlen = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
		if (qlen > 0.0f) { qw /= qlen; qx /= qlen; qy /= qlen; qz /= qlen; }
		g.rotation.set(qx, qy, qz, qw);

		// Color: SH DC coefficient -> RGB [0,1] + sigmoid opacity
		g.color.set(
			0.5f + SH_C0 * ir.colors[i * 3 + 0],
			0.5f + SH_C0 * ir.colors[i * 3 + 1],
			0.5f + SH_C0 * ir.colors[i * 3 + 2],
			1.0f / (1.0f + std::exp(-ir.alphas[i]))
		);

		// SH coefficients: GaussForge stores RGB-interleaved per coefficient
		// Our shader expects channel-grouped: [R0..R14, G0..G14, B0..B14]
		std::memset(g.sh, 0, sizeof(g.sh));
		if (shPerPoint > 0)
		{
			int numCoeffs = std::min(shPerPoint / 3, 15);
			for (int c = 0; c < numCoeffs; ++c)
			{
				int srcBase = i * shPerPoint + c * 3;
				g.sh[c]            = ir.sh[srcBase + 0]; // R channel
				g.sh[c + 15]       = ir.sh[srcBase + 1]; // G channel
				g.sh[c + 30]       = ir.sh[srcBase + 2]; // B channel
			}
		}

		bounds.expandBy(g.position);
		points.push_back(g);
	}

	this->hasSH = (shPerPoint > 0);
	std::cout << "Loaded " << points.size() << " Gaussian points via GaussForge (."
	          << ext << ")" << std::endl;
	return points;
}

osg::ref_ptr<osg::Geometry> GaussianDrawObj::createQuadGeometry()
{
	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(4);
	(*vertices)[0].set( 1.0f,  1.0f, 0.0f);
	(*vertices)[1].set(-1.0f,  1.0f, 0.0f);
	(*vertices)[2].set( 1.0f, -1.0f, 0.0f);
	(*vertices)[3].set(-1.0f, -1.0f, 0.0f);
	quad->setVertexArray(vertices);

	osg::ref_ptr<osg::DrawElementsUShort> de =
		new osg::DrawElementsUShort(GL_TRIANGLES);
	de->push_back(0); de->push_back(1); de->push_back(2);
	de->push_back(1); de->push_back(3); de->push_back(2);
	de->setNumInstances(nNum);
	quad->addPrimitiveSet(de);

	quad->setCullingActive(false);
	quad->setUseDisplayList(false);
	quad->setUseVertexBufferObjects(true);
	return quad;
}

// ======================== Global Camera ========================

osg::Camera* g_camera = nullptr;
void setMainCamera(osg::Camera* pC) { g_camera = pC; }
osg::Camera* getCamera() { return g_camera; }
