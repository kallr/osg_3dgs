 
#ifndef BATCHOBJ_MANAGER_H
#define BATCHOBJ_MANAGER_H

#include "osg\Matrixf"
#include "osg\BoundingBox"
#include "osg\Node"
#include "osg\Group"
#include "osg\matrixTransform"
 

//gaussian point
struct MI_GaussianPoint {
	osg::Vec3f position; //position
	osg::Vec4f color;    //color, rgba (DC项基础色，rgba)

	//协方差矩阵
	osg::Vec4f sigma1;
	osg::Vec4f sigma2;
	osg::Vec4f sigma3;

	// 球谐系数 1~3阶，RGB各15个，共45个 (f_rest_0 ~ f_rest_44)
	// 布局: [R1,R2,...,R15, G1,...,G15, B1,...,B15]
	float sh[45] = {};
};

//gaussian draw obj
class  GaussianDrawObj
{
public:
	GaussianDrawObj(const std::string& name);
	~GaussianDrawObj();
 
	bool getDirty();
	void setDirty(bool flag);
 
	//return osg node
	osg::ref_ptr<osg::Node> getNode(); 
	//sort
	void runSortAndUpdate(const osg::Matrix& viewProj, osg::Image* paramsImage);
private:
	void updateImage(osg::Image* paramsImage);
	void updateSHImage(osg::Image* paramsImage);
	void loadShader(osg::StateSet* ss);
	void updateIndexImage(osg::Image* paramsImage);

	std::vector<MI_GaussianPoint> readSplatFile(const std::string& filename);
	std::vector<MI_GaussianPoint> readFlyFile(const std::string& filename);
	osg::ref_ptr<osg::Geometry> createQuadGeometry();
private:
	bool bDirty = false;
	bool bInit=false;
	bool flag = false;
	bool hasSH = false;
	int nNum = 0;
	std::vector< MI_GaussianPoint> gaussianPoints;
	std::vector<int> depthIndex;
	std::vector<size_t> distances;
	osg::BoundingBox bounds;
};

void setMainCamera(osg::Camera* pC);
osg::Camera* getCamera();

#endif


  