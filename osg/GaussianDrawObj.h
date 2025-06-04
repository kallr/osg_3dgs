 
#ifndef BATCHOBJ_MANAGER_H
#define BATCHOBJ_MANAGER_H

#include "osg\Matrixf"
#include "osg\BoundingBox"
#include "osg\Node"
#include "osg\Group"
#include "osg\matrixTransform"
 

// 高斯点数据结构
struct MI_GaussianPoint {
	osg::Vec3f position;
	osg::Vec4f color;

	osg::Vec4f sigma1;
	osg::Vec4f sigma2;
	osg::Vec4f sigma3;
};

class  GaussianDrawObj
{
public:
	GaussianDrawObj(const std::string& name);
	~GaussianDrawObj();
 
	bool getDirty();
	void  setDirty(bool flag);

 
	osg::ref_ptr<osg::Node> getNode();

 	void addPt(MI_GaussianPoint* objPos, int id);

	void updateUniforms(osg::Vec4f* ptr,int id);

	void updateImage(osg::Image* paramsImage);
	void updateIndexImage(osg::Image* paramsImage);



	void runSort(const osg::Matrix& viewProj);

	void updateInstanceCount(int newCount);
private:
	void loadShader(osg::StateSet* ss);
	std::vector<MI_GaussianPoint> readSplatFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readFlyFile(const std::string& filename);
	osg::ref_ptr<osg::Geometry> createQuadGeometry();
private:
	bool bDirty = false;
	bool bInit=false;
	bool flag = false;
	int nNum = 0;


	std::vector< MI_GaussianPoint> gaussianPoints;
	std::vector<int> depthIndex;
	std::vector<size_t> distances;
	osg::BoundingBox bounds;
};

void setMainCamera(osg::Camera* pC);
osg::Camera* getCamera();

#endif


  