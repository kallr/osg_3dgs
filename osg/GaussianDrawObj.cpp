// TH3DGraphLibTestView.cpp : CTHGW3DGraphView 类的实现
//
#include "GaussianDrawObj.h"

#include "osg/Material"
#include "osg/Geode"
#include "osg/Geometry"
#include "osg/TextureBuffer"
#include "osg/MatrixTransform" 
#include "osg/MatrixTransform"
#include "osgUtil/CullVisitor"
#include "osg/ComputeBoundsVisitor"
#include "osgDB/ReadFile"
#include <osg/BlendFunc>
#include <osg/BlendEquation>
#include "tools/miniply.h"

#include "tools/tools.h"


#include <glm/common.hpp>

#define GLM_ENABLE_EXPERIMENTAL  // waow
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/string_cast.hpp>


class SSCallback : public osg::StateSet::Callback
{
public:
	SSCallback(GaussianDrawObj* pBatchObj)
	{
		pBatchObj_ = pBatchObj;
	}
	virtual void operator()(osg::StateSet* ss, osg::NodeVisitor* nv)
	{
		osg::StateSet* stateSet = dynamic_cast<osg::StateSet*>(ss);
		if (stateSet)
		{
			osg::Camera* camera = getCamera();
			if (camera)
			{
				osg::Matrixf viewMatrix = camera->getViewMatrix();
				ss->getOrCreateUniform("view", osg::Uniform::FLOAT_MAT4)->set(viewMatrix);

				osg::Matrixf projMatrix = camera->getProjectionMatrix();
				ss->getOrCreateUniform("proj", osg::Uniform::FLOAT_MAT4)->set(projMatrix);

				osg::Vec2 vs;
				vs[0] = camera->getViewport()->width();
				vs[1] = camera->getViewport()->height();
				ss->getOrCreateUniform("viewport_size", osg::Uniform::FLOAT_VEC2)->set(vs);
			}
		}
	}
	GaussianDrawObj* pBatchObj_ = nullptr;
};


//updae index iamge
class BatchObjTextureCBEx : public osg::StateAttributeCallback
{
public:
	BatchObjTextureCBEx(GaussianDrawObj* pBatchObj)
	{
		pBatchObj_ = pBatchObj;
	}

	virtual void operator()(osg::StateAttribute* ss, osg::NodeVisitor* nv)
	{ 
		if (!pBatchObj_)return;

		if (pBatchObj_->getDirty() )
		{
			pBatchObj_->setDirty(false);

			osg::ref_ptr<osg::TextureBuffer> tbo = dynamic_cast<osg::TextureBuffer*>(ss);
			osg::Image* indexImage = tbo->getImage();
			if (!indexImage)
				return;


			osg::Matrixd curViewMat = getCamera()->getProjectionMatrix()* getCamera()->getViewMatrix();

			double  dot =
				viewMatrix(0, 2) * curViewMat(0, 2) +
				viewMatrix(1, 2) * curViewMat(1, 2) +
				viewMatrix(2, 2) * curViewMat(2, 2);

			if (abs(dot - 1) < 0.01) {
				return;
			}

			viewMatrix = curViewMat;
			pBatchObj_->runSortAndUpdate(viewMatrix,indexImage);
		}
	}

private:
	GaussianDrawObj* pBatchObj_ = nullptr;
	int nCount = 0;
	osg::Matrixd viewMatrix;
};

GaussianDrawObj::GaussianDrawObj(const std::string& name)
{  
	if (name.find(".ply") != -1)
		gaussianPoints = readFlyFile(name);
	else if (name.find(".splat"))
		gaussianPoints = readSplatFile(name);

	nNum = gaussianPoints.size();

	distances.resize(nNum );
	depthIndex.resize(nNum);

	for (int i = 0; i < nNum; i++)
		depthIndex[i] = i;
}

GaussianDrawObj::~GaussianDrawObj()
{

}

bool GaussianDrawObj::getDirty()
{
	return bDirty;
}
void  GaussianDrawObj::setDirty(bool flag)
{
	bDirty = flag;
}


// 输入：当前视图投影矩阵,和索引
void GaussianDrawObj::runSortAndUpdate(const osg::Matrix& viewProj,osg::Image* paramsImage)
{ 
	osg::Vec3f cam_pos;
	osg::Vec3f center;
	osg::Vec3f up;
	getCamera()->getViewMatrixAsLookAt(cam_pos, center, up);

	const size_t n_buckets = 65535;
	std::vector<size_t> count(n_buckets + 1, 0);

	int num_gaussians = gaussianPoints.size();
  
	std::vector<int> output(num_gaussians, 0);

	float max_dist = 1.2f *2 * bounds.radius();
	max_dist *= max_dist;

 	for (int i = 0; i< gaussianPoints.size(); i++)
	{
		const auto& g = gaussianPoints[i];
		auto v = -cam_pos - g.position;
		float d = v.x() * v.x() + v.y() * v.y() + v.z() * v.z();  // dot product
		float d_normalized = n_buckets * d / max_dist;  // between 0 and n_buckets
		size_t d_int = std::min(d_normalized, (float)n_buckets - 1);
		++count[d_int];
		distances[i]=d_int;
	}

	for (int i = 1; i < count.size(); ++i) {
		count[i] = count[i] + count[i - 1];
	}

	for (int i = num_gaussians - 1; i >= 0; --i)
	{
		size_t j = distances[i];
		--count[j];
		output[count[j]] = i;
	}
	std::swap(output, depthIndex);
	updateIndexImage(paramsImage);
}

  

void GaussianDrawObj::updateImage(osg::Image* paramsImage)
{
	for(int i = 0; i< nNum; i++)
	{
		const MI_GaussianPoint& gsPos = gaussianPoints[i];
		osg::Vec4f* ptr = (osg::Vec4f*)paramsImage->data(5 * i);
		if(ptr)
		{
			ptr[0].set(gsPos.position[0], gsPos.position[1], gsPos.position[2], 1);
			//color
			ptr[1] = gsPos.color;
			//con
			ptr[2] = gsPos.sigma1;
			ptr[3] = gsPos.sigma2;
			ptr[4] = gsPos.sigma3;
		}
 	}

	paramsImage->dirty();
}

void GaussianDrawObj::updateIndexImage(osg::Image* paramsImage)
{
	int num = depthIndex.size();

	osg::Vec4f* ptr0 = (osg::Vec4f*)paramsImage->data(0);

 	for (int i = 0; i< num ;i++ )
	{
 		ptr0->set(depthIndex[i],0,0,0 );
		ptr0++;
	}

	paramsImage->dirty();
}


void GaussianDrawObj::loadShader(osg::StateSet* ss)
{
	osg::ref_ptr<osg::Program> gsProgram = new osg::Program;
	if(gsProgram)
	{
		std::string strDir = osg_tools::getAppDir() +  "\\shader\\";
		osg::Shader* vertex_shader = osgDB::readShaderFile(  osg::Shader::VERTEX,   strDir + "gaussian.vert");
		osg::Shader* fragment_shader = osgDB::readShaderFile(osg::Shader::FRAGMENT, strDir + "gaussian.frag");
		gsProgram->addShader(vertex_shader);
		gsProgram->addShader(fragment_shader);
	}
 	ss->setAttribute(gsProgram);
}

 
osg::ref_ptr<osg::Node> GaussianDrawObj::getNode()
{
	osg::ref_ptr<osg::Geode> geode = new osg::Geode; 
 
	osg::ref_ptr<osg::Geometry> pInstance = createQuadGeometry(); 
	geode->addChild(pInstance);


	//index buffer
	{
		osg::ref_ptr<osg::Image> paramsImage = new osg::Image;
		int transPos = nNum;
		paramsImage->allocateImage(transPos, 1, 1, GL_RGBA, GL_FLOAT);
		updateIndexImage(paramsImage);

		osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
		tbo->setImage(paramsImage.get());
		tbo->setInternalFormat(GL_RGBA32F_ARB);
		pInstance->getOrCreateStateSet()->setTextureAttribute(0, tbo.get());
		tbo->setUpdateCallback(new BatchObjTextureCBEx(this));
	}

	//data buffer
	{
		osg::ref_ptr<osg::Image> paramsImage = new osg::Image;
		int transPos = 5 * nNum;
		paramsImage->allocateImage(transPos, 1, 1, GL_RGBA, GL_FLOAT);
		updateImage(paramsImage);

		osg::ref_ptr<osg::TextureBuffer> tbo = new osg::TextureBuffer;
		tbo->setImage(paramsImage.get());
		tbo->setInternalFormat(GL_RGBA32F_ARB);
		pInstance->getOrCreateStateSet()->setTextureAttribute(1, tbo.get());
	}

	//ss
	osg::StateSet* stateset = pInstance->getOrCreateStateSet();
	if (stateset)
	{
		loadShader(stateset);

		stateset->setUpdateCallback(new SSCallback(this));


		osg::Uniform* indexBufferSampler = new osg::Uniform("indexBuffer", int(0));
		stateset->addUniform(indexBufferSampler);

		osg::Uniform* dataBufferSampler = new osg::Uniform("dataBuffer", int(1));
		stateset->addUniform(dataBufferSampler);


		//禁用深度测试
		stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

		// 启用混合
		stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
		osg::ref_ptr<osg::BlendFunc> blendFunc = new osg::BlendFunc();
		blendFunc->setFunction(
			osg::BlendFunc::DST_ALPHA, osg::BlendFunc::ONE, // RGB
			osg::BlendFunc::ZERO, osg::BlendFunc::ONE_MINUS_SRC_ALPHA  // Alpha
		);

		stateset->setAttributeAndModes(blendFunc, osg::StateAttribute::ON);
	}

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


	// 假设每个点的数据布局
	struct Splat {
		float pos[3];    // 位置 (x, y, z)
		float scale[3];  // 缩放 (sx, sy, sz)
		uint8_t color[4];  // 颜色 (r, g, b,a)
		uint8_t rot[4];    // 旋转四元数 (w, x, y, z)
	};

	Splat splat;
	while (file.read(reinterpret_cast<char*>(&splat), sizeof(Splat)))
	{
		MI_GaussianPoint point;
		point.position.set(splat.pos[0], splat.pos[1], splat.pos[2]);

		point.color.set(splat.color[0], splat.color[1], splat.color[2], splat.color[3]);
		point.color /= 255.0;

		glm::vec3 scale = {
						splat.scale[0],
						splat.scale[1],
						splat.scale[2]
		};

		glm::mat4 scale_mat = glm::scale(glm::mat4(1.0f), scale);

		float rx = (1.0 * splat.rot[0] - 128.0) / 128.0;
		float ry = (1.0 * splat.rot[1] - 128.0) / 128.0;
		float rz = (1.0 * splat.rot[2] - 128.0) / 128.0;
		float rw = (1.0 * splat.rot[3] - 128.0) / 128.0;

		glm::quat rot = { rx,ry,rz,rw };

		glm::mat4 rot_mat = glm::mat4(glm::mat3(rot));

		auto rot_scale = rot_mat * scale_mat;
		glm::mat4 sigma = rot_scale * glm::transpose(rot_scale);

		point.sigma1 = osg::Vec4f(sigma[0][0], sigma[0][1], sigma[0][2], sigma[0][3]);
		point.sigma2 = osg::Vec4f(sigma[1][0], sigma[1][1], sigma[1][2], sigma[1][3]);
		point.sigma3 = osg::Vec4f(sigma[2][0], sigma[2][1], sigma[2][2], sigma[2][3]);

		bounds.expandBy(point.position);

		points.push_back(point);
	}

	std::cout << "Loaded " << points.size() << " Gaussian points" << std::endl;
	return points; 
}


std::vector<MI_GaussianPoint> GaussianDrawObj::readFlyFile(const std::string& ply_path)
{
	std::vector<MI_GaussianPoint> points;

 	const float SH_0 = 0.28209479177387814f;
	const std::vector<std::string> properties = {
	"x",
	"y",
	"z",
	"f_dc_0",
	"f_dc_1",
	"f_dc_2",
	"opacity",
	"scale_0",
	"scale_1",
	"scale_2",
	"rot_0",
	"rot_1",
	"rot_2",
	"rot_3",
	};
	miniply::PLYReader reader(ply_path.c_str());
	if (!reader.valid()) {
		std::cerr << "Failed to open " << ply_path << std::endl;
	}
	// Check we have a vertex element to pull data from
	miniply::PLYElement* gsSplatEl = reader.get_element(reader.find_element("vertex"));
	if (gsSplatEl == nullptr) {
		std::cerr << "no vertex element could be found" << std::endl;
	}

	size_t num_gaussians = reader.num_rows();
	points.reserve(num_gaussians);

	// Getting all the indices where the relevant splat data is stored in the file
	std::vector<uint32_t> gaussSplatIdx(properties.size());

	for (int i = 0; i < properties.size(); ++i) {
		gaussSplatIdx[i] = gsSplatEl->find_property(properties[i].c_str());
	}

	reader.load_element();
	float* filedata = new float[properties.size() * num_gaussians];
	reader.extract_properties(
		gaussSplatIdx.data(), properties.size(), miniply::PLYPropertyType::Float, filedata);

	// Creating gaussian splats based on the data we read in
	for (size_t i = 0; i < num_gaussians; ++i) {

		int offset = i * properties.size();
		MI_GaussianPoint g;

		g.position = {
			filedata[offset + 0],
			filedata[offset + 1],
			filedata[offset + 2]
		};

		g.color = {
			filedata[offset + 3],
			filedata[offset + 4],
			filedata[offset + 5],
			0.0f,
		};

		// Extract base color from spherical harmonics
		g.color = g.color*SH_0 + osg::Vec4f(0.5, 0.5, 0.5, 0.5);
		g.color[3] = 1.0f / (1.0f + exp(-(filedata[offset + 6])));

		glm::vec3 scale = {
					glm::exp(filedata[offset + 7]),
					glm::exp(filedata[offset + 8]),
					glm::exp(filedata[offset + 9]),
		};
		glm::mat4 scale_mat = glm::scale(glm::mat4(1.0f), scale);

		glm::quat rot = {
			filedata[offset + 10],
			filedata[offset + 11],
			filedata[offset + 12],
			filedata[offset + 13],
		};
		glm::mat4 rot_mat = glm::mat4(glm::mat3(rot));

		auto rot_scale = rot_mat * scale_mat;
		glm::mat4 sigma = rot_scale * glm::transpose(rot_scale);

		g.sigma1 = osg::Vec4f(sigma[0][0], sigma[0][1], sigma[0][2], sigma[0][3]);
		g.sigma2 = osg::Vec4f(sigma[1][0], sigma[1][1], sigma[1][2], sigma[1][3]);
		g.sigma3 = osg::Vec4f(sigma[2][0], sigma[2][1], sigma[2][2], sigma[2][3]);


		bounds.expandBy(g.position);
		points.push_back(g);

	}
	// Clean up the float array as the gaussians are now stored in the vec "data"
	delete[] filedata;

	return points; 
}

osg::ref_ptr<osg::Geometry> GaussianDrawObj::createQuadGeometry()
{ 
	// 创建四边形几何体（用于实例化渲染）
	osg::ref_ptr<osg::Geometry> quad = new osg::Geometry;

	// 顶点位置（单位四边形）
	osg::ref_ptr<osg::Vec2Array> vertices = new osg::Vec2Array;
	vertices->push_back(osg::Vec2(-2.0f, -2.0f)); // 左下
	vertices->push_back(osg::Vec2(2.0f, -2.0f));  // 右下
	vertices->push_back(osg::Vec2(2.0f, 2.0f));   // 右上
	vertices->push_back(osg::Vec2(-2.0f, 2.0f));  // 左上

	quad->setVertexArray(vertices);
	quad->setVertexAttribArray(0, vertices, osg::Array::BIND_PER_VERTEX);

	// 顶点索引
	osg::ref_ptr<osg::DrawArrays> indices = new osg::DrawArrays(GL_TRIANGLE_FAN, 0, 4, nNum);

	quad->addPrimitiveSet(indices);

	quad->setCullingActive(false);
	quad->setUseDisplayList(false);
	quad->setUseVertexBufferObjects(true);

	return quad;
}


osg::Camera* g_camera = nullptr;
void setMainCamera(osg::Camera* pC) {
	g_camera = pC;
}
osg::Camera* getCamera() {
	return g_camera;
}

