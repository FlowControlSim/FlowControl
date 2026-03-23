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

	fluidDensity = nAttr.create("fluidDensity", "fd", MFnNumericData::kFloat, 0.1f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(fluidDensity);

	dragCoeff = nAttr.create("dragCoeff", "dragCoeff", MFnNumericData::kFloat, 10.0f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(dragCoeff);

	mass = nAttr.create("mass", "mass", MFnNumericData::kFloat, 1.0f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
    addAttribute(mass);

	attributeAffects(testScene::inTime, testScene::outTransform);
	attributeAffects(testScene::fluidDensity, testScene::outTransform);
	attributeAffects(testScene::dragCoeff, testScene::outTransform);
	attributeAffects(testScene::mass, testScene::outTransform);
	attributeAffects(testScene::inMesh, testScene::outTransform);


	return MStatus::kSuccess;


	return status;
}

MStatus testScene::compute(const MPlug& plug, MDataBlock& data)
{
    MStatus status;

    if (plug == outTransform) {
        MTime currentTime = data.inputValue(inTime).asTime();
        double currentFrame = currentTime.value();

        // 1. INITIALIZATION (Frame 1 or earlier)
        if (currentFrame <= 1.0) {
            // CRITICAL FIX: Do NOT declare these as new variables (e.g., Vector6d m_currentG = ...). 
            // You must assign to the class member variables, otherwise the state is lost when the block ends!
            m_currentG = identity(); // identity() is from SE3Transform.h
            m_currentMu = Vector6D(vec6::Zero());

            // Read mesh from MDataBlock
            MObject meshObj = data.inputValue(inMesh).asMesh();
            MeshData meshData;
            meshData.extractMeshData(meshObj);

            // Set mass before computing properties so centroid/inertia have weights
            // (Assuming you have a mass attribute, or hardcode for now)
            double totalMass = 1.0;
            meshData.setMassDensity(totalMass, MassDensityType::UNIFORM, nullptr);
            meshData.computeProperties();

            // CONVERSION: Maya MPointArray to Eigen std::vector<Vector3d>
            std::vector<Vector3d> eigenVertices;
            eigenVertices.reserve(meshData.m_vertices.length());
            for (unsigned int i = 0; i < meshData.m_vertices.length(); ++i) {
                eigenVertices.push_back(Vector3d(meshData.m_vertices[i].x, meshData.m_vertices[i].y, meshData.m_vertices[i].z));
            }

            // Compute and CACHE the inertia matrix
            FlowIntegrator flowC;
            m_cachedK = flowC.compute_body_inertia(eigenVertices, meshData.m_massDensity);

            // Cache volume for buoyancy calculations later
            m_cachedVolume = meshData.m_totalVolume;

            m_previousTime = currentFrame;
        }
        // 2. SIMULATION STEP (Moving forward in time)
        else if (currentFrame > m_previousTime) {
            // Calculate dt in seconds (Assuming 24 fps, you could also read this from Maya's MTime::uiUnit())
            double dt = (currentFrame - m_previousTime) / 24.0;

            // Flow Integrator setup
            FlowIntegrator integrator;

            // TODO: Read these from node attributes in the future. Hardcoded for testing.
            double mass_body = 1.0;
            double rho_fluid = 1.225; // Air density
            double ref_area = 1.0;    // Reference area for drag
            double C_d = 0.5;         // Drag coefficient
            bool include_drag = true;

            // Compute forces and integrate
            // Note: We need the current velocity (Y), which is K_inv * mu
            Vector6D Y = Vector6D(m_cachedK.inverse() * m_currentMu.data);

            Vector6D F = integrator.compute_total_force(Y, mass_body, m_cachedVolume, rho_fluid, ref_area, C_d, include_drag);

            NewtonResult result = integrator.integrate_step_newton(m_currentG, m_currentMu, m_cachedK, Vector6D(), F, dt);

            // Update internal Node State
            m_currentG = result.g_next;
            m_currentMu = result.mu_next;
            m_previousTime = currentFrame;
        }

        // 3. OUTPUT THE MATRIX
        MMatrix outMat;
        // Convert Eigen::Matrix4d (m_currentG.data) to Maya's MMatrix
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                // Maya matrices are accessed as [row][col]
                outMat[r][c] = m_currentG.data(c, r);
            }
        }

        // Write to output plug
        MDataHandle outHandle = data.outputValue(outTransform);
        outHandle.setMMatrix(outMat);
        data.setClean(plug);

        return MS::kSuccess;
    }
    return MS::kUnknownParameter; 
}