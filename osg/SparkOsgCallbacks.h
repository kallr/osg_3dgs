#ifndef SPARK_OSG_CALLBACKS_H
#define SPARK_OSG_CALLBACKS_H

#include <osg/Drawable>
#include <osg/RenderInfo>

#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
#endif
#ifndef GL_TEXTURE_FETCH_BARRIER_BIT
#define GL_TEXTURE_FETCH_BARRIER_BIT 0x00000008
#endif

class MemoryBarrierDrawable : public osg::Drawable
{
public:
    MemoryBarrierDrawable();
    MemoryBarrierDrawable(GLbitfield barriers);
    MemoryBarrierDrawable(const MemoryBarrierDrawable& rhs, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    META_Node(osg, MemoryBarrierDrawable);

    virtual osg::BoundingBox computeBoundingBox() const;
    virtual void drawImplementation(osg::RenderInfo& renderInfo) const;

private:
    GLbitfield _barriers;
};

#endif
