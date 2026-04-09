#ifndef TEST_SCENE_H
#define TEST_SCENE_H

#include <maya/MPxNode.h>
#include <maya/MTypeId.h>
#include <maya/MPlug.h>
#include <maya/MDataBlock.h>

#include "SE3Transform.h"
#include "Vector6D.h"
#include "helpers.h"
#include "FlowIntegrator.h"

class testScene : public MPxNode {
public:
    testScene() {};
    virtual ~testScene() {};

    static void* creator();
    static MStatus initialize();

    virtual MStatus compute(const MPlug& plug, MDataBlock& data) override;

    static MTypeId id;

    static MObject inMesh;
    static MObject fluidDensity;
    static MObject dragCoeff;
    static MObject mass;
    static MObject outTransform;
    static MObject inTime;

private:
    double m_previousTime = 0.0; 
    SE3Transform m_currentG;
    Vector6D m_currentMu;
    Matrix6d m_cachedK;
    double m_cachedVolume = 0.0;
    std::vector<Vector3d> m_cachedVertices;
    bool m_isInitialized = false;
};

#endif // TEST_SCENE_H