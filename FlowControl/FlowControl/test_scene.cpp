#include "test_scene.h"
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnUnitAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include "meshData.h"

MTypeId testScene::id(0x8000f);
MObject testScene::inMesh;
MObject testScene::fluidDensity;
MObject testScene::dragCoeff;
MObject testScene::mass;
MObject testScene::inTime;
MObject testScene::outTransform;


void* testScene::creator()
{
	return new testScene();
}

MStatus testScene::initialize()
{
	MStatus status;


	// define attrs
	MFnUnitAttribute uAttr; // inTime
	MFnMatrixAttribute mAttr; // outTransform
	MFnNumericAttribute nAttr; // fluidDensity, dragCoefficient, mass
	MFnTypedAttribute sAttr; // inMesh

	inTime = uAttr.create("inTime", "inTime", MFnUnitAttribute::kTime);
	uAttr.setStorable(false);
	uAttr.setConnectable(true);
	uAttr.setKeyable(true);
	addAttribute(inTime);

	inMesh = sAttr.create("inMesh", "inMesh", MFnData::kMesh);
	sAttr.setStorable(true);
    sAttr.setConnectable(true);
	addAttribute(inMesh);

	outTransform = mAttr.create("outTransform", "outTransform");
	mAttr.setStorable(true);
	mAttr.setConnectable(true);
	mAttr.setKeyable(true);
	addAttribute(outTransform);

	fluidDensity = nAttr.create("fluidDensity", "fd", MFnNumericData::kDouble, 0.1f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(fluidDensity);

	dragCoeff = nAttr.create("dragCoeff", "dragCoeff", MFnNumericData::kDouble, 10.0f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(dragCoeff);

	mass = nAttr.create("mass", "mass", MFnNumericData::kDouble, 1.0);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
    addAttribute(mass);

	attributeAffects(testScene::inTime, testScene::outTransform);
	attributeAffects(testScene::fluidDensity, testScene::outTransform);
	attributeAffects(testScene::dragCoeff, testScene::outTransform);
	attributeAffects(testScene::mass, testScene::outTransform);
	attributeAffects(testScene::inMesh, testScene::outTransform);

	return status;
}

MStatus testScene::compute(const MPlug& plug, MDataBlock& data)
{
    if (plug != outTransform && plug != inTime)
        return MS::kUnknownParameter;

    MTime currentTime = data.inputValue(inTime).asTime();
    double currentFrame = currentTime.value();

    MStatus status;
    MStatus returnStatus;

    MGlobal::displayInfo(MString(">> compute called | plug: ") + plug.name());

    double mass_body = data.inputValue(testScene::mass, &returnStatus).asDouble();
    double C_d = data.inputValue(testScene::dragCoeff, &returnStatus).asDouble();
    double rho_fluid = data.inputValue(testScene::fluidDensity, &returnStatus).asDouble();
    MObject meshObj = data.inputValue(inMesh).asMesh();

    if (plug == outTransform || plug == inTime) {
        //MTime currentTime = data.inputValue(inTime).asTime();
        //double currentFrame = currentTime.value();

        // 1. INITIALIZATION (Frame 1 or earlier)
        if (!m_isInitialized || currentFrame <= 1.0) {
  
            // Read mesh from MDataBlock
            MeshData meshData(meshObj);
            
            // Set mass before computing properties so centroid/inertia have weights
            // (Assuming you have a mass attribute, or hardcode for now)
            meshData.setMassDensity(mass_body, MassDensityType::UNIFORM, nullptr);
            meshData.computeProperties();
          
            std::vector<Vector3d> eigenVertices;
            eigenVertices.reserve(meshData.m_vertices.length());
            for (unsigned int i = 0; i < meshData.m_vertices.length(); ++i) {
                eigenVertices.push_back(Vector3d(meshData.m_vertices[i].x, meshData.m_vertices[i].y, meshData.m_vertices[i].z));
            }

            // Compute and CACHE the inertia matrix
            FlowIntegrator flowC;
            m_cachedK = flowC.compute_body_inertia(eigenVertices, meshData.m_massDensity);
            m_cachedVolume = meshData.m_totalVolume;
            m_currentG = identity();
            m_currentMu = Vector6D(vec6::Zero());

            MGlobal::displayInfo(MString("INIT FRAME 1. Mass: ") + mass_body + " Volume: " + m_cachedVolume);

            m_previousTime = currentFrame;
            m_isInitialized = true;

        }
        else if (m_isInitialized && currentFrame > m_previousTime) {
            // Calculate dt in seconds (Assuming 24 fps, you could also read this from Maya's MTime::uiUnit())
            double dt = (currentFrame - m_previousTime) / 24.0;

            // Flow Integrator setup
            FlowIntegrator integrator;

            double ref_area = 1.0;    // Reference area for drag
            bool include_drag = true;

            // Compute forces and integrate
            // Note: We need the current velocity (Y), which is K_inv * mu
            Vector6D Y = Vector6D(m_cachedK.inverse() * m_currentMu.data);

            Vector6D F = integrator.compute_total_force(Y, mass_body, m_cachedVolume, rho_fluid, ref_area, C_d, include_drag);

            NewtonResult result = integrator.integrate_step_newton(m_currentG, m_currentMu, m_cachedK, Vector6D(), F, dt);

            MGlobal::displayInfo(MString("SIM STEP | Frame: ") + currentFrame +
                " | Force Y: " + F.vel()[1] +
                " | Residual: " + result.residual);

            // Update internal Node State
            m_currentG = result.g_next;
            m_currentMu = result.mu_next;
            m_previousTime = currentFrame;

            MGlobal::displayInfo(MString("NEW POS Y: ") + m_currentG.t()[1]);
        }
        else {
            // This branch means currentFrame <= m_previousTime (scrubbing backwards)
            MGlobal::displayWarning(MString("Backwards scrub detected at frame: ") + currentFrame
                + " prevTime was: " + m_previousTime);
            // Reset so next forward play re-initializes
            m_isInitialized = false;
        }



        MMatrix outMat;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                outMat[r][c] = m_currentG.data(r, c);
            }
        }

        outMat[3][0] = m_currentG.data(0, 3);
        outMat[3][1] = m_currentG.data(1, 3);
        outMat[3][2] = m_currentG.data(2, 3);
        outMat[0][3] = 0.0;
        outMat[1][3] = 0.0;
        outMat[2][3] = 0.0;
        outMat[3][3] = 1.0;


        //// OUTPUT THE MATRIX
        //MMatrix outMat;
        //// Convert Eigen::Matrix4d (m_currentG.data) to Maya's MMatrix
        //for (int r = 0; r < 4; ++r) {
        //    for (int c = 0; c < 4; ++c) {
        //        // Maya matrices are accessed as [row][col]
        //        //outMat[r][c] = m_currentG.data(c, r);
        //        outMat[r][c] = m_currentG.data(r, c);
        //    }
        //}

        /*if (plug != outTransform && plug != inTime) {
            return MS::kUnknownParameter;
        }*/

        MDataHandle outHandle = data.outputValue(outTransform);
        outHandle.setMMatrix(outMat);
        data.setClean(plug);

        /*if (plug == outTransform) {
            MDataHandle outHandle = data.outputValue(outTransform);
            outHandle.setMMatrix(outMat);
            data.setClean(plug);
        }*/


        return MS::kSuccess;
    }
    return MS::kUnknownParameter; 
}