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
MObject testScene::liftCoeff;
MObject testScene::angularDrag;

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

	fluidDensity = nAttr.create("fluidDensity", "fd", MFnNumericData::kDouble, 1.225f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(fluidDensity);

	dragCoeff = nAttr.create("dragCoeff", "dragCoeff", MFnNumericData::kDouble, 1.2f);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
	addAttribute(dragCoeff);

	mass = nAttr.create("mass", "mass", MFnNumericData::kDouble, 0.003);
	nAttr.setStorable(true);
	nAttr.setConnectable(true);
	nAttr.setKeyable(true);
    addAttribute(mass);

    liftCoeff = nAttr.create("liftCoeff", "lc", MFnNumericData::kFloat, 0.05f);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(liftCoeff);

    angularDrag = nAttr.create("angularDrag", "ad", MFnNumericData::kFloat, 2.0f);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(angularDrag);

    attributeAffects(testScene::liftCoeff, testScene::outTransform);
    attributeAffects(testScene::angularDrag, testScene::outTransform);
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

    //MGlobal::displayInfo(MString(">> compute called | plug: ") + plug.name());

    double mass_body = data.inputValue(testScene::mass, &returnStatus).asDouble();
    double C_d = data.inputValue(testScene::dragCoeff, &returnStatus).asDouble();
    double C_l = data.inputValue(testScene::liftCoeff, &returnStatus).asDouble();
    double k_ang = data.inputValue(testScene::angularDrag, &returnStatus).asDouble();
    double rho_fluid = data.inputValue(testScene::fluidDensity, &returnStatus).asDouble();
    MObject meshObj = data.inputValue(inMesh).asMesh();

    if (plug == outTransform || plug == inTime) {
        //MTime currentTime = data.inputValue(inTime).asTime();
        //double currentFrame = currentTime.value();

        // 1. INITIALIZATION (Frame 1 or earlier)
        if (!m_isInitialized || currentFrame <= 1.0) {
  
            // Read mesh from MDataBlock
            MeshData meshData(meshObj);

            const double MAYA_TO_METERS = 0.01;
            const double VOLUME_SCALE = MAYA_TO_METERS * MAYA_TO_METERS * MAYA_TO_METERS; // 1e-6

            m_cachedVolume = meshData.m_totalVolume * VOLUME_SCALE;

            double tilt = M_PI / 12.0; // 15 degrees
            Matrix4d tiltMat = Matrix4d::Identity();
            tiltMat(0, 0) = cos(tilt);
            tiltMat(0, 1) = -sin(tilt);
            tiltMat(1, 0) = sin(tilt);
            tiltMat(1, 1) = cos(tilt);

            SE3Transform tiltedG;
            tiltedG.data = tiltMat;
            m_currentG = tiltedG;

            // Keep a small seed angular momentum
            vec6 init;
            init << 0.0001, 0.0002, 0.0001,   // tiny angular seed in all 3 axes
                0.0, 0.0, 0.0;
            m_currentMu = Vector6D(init);

            
            // Set mass before computing properties so centroid/inertia have weights
            // (Assuming you have a mass attribute, or hardcode for now)
            meshData.setMassDensity(mass_body, MassDensityType::UNIFORM, nullptr);
            meshData.computeProperties();

            double delta = meshData.computeInverseAverageMeanCurvature();
          
            std::vector<Vector3d> eigenVertices;
            eigenVertices.reserve(meshData.m_vertices.length());
            for (unsigned int i = 0; i < meshData.m_vertices.length(); ++i) {
                eigenVertices.push_back(Vector3d(meshData.m_vertices[i].x * MAYA_TO_METERS, meshData.m_vertices[i].y * MAYA_TO_METERS, meshData.m_vertices[i].z * MAYA_TO_METERS));
            }

            // Compute and CACHE the inertia matrix
            FlowIntegrator flowC;
            m_cachedK = flowC.compute_body_inertia(eigenVertices, meshData.m_massDensity);
            //m_cachedVolume = meshData.m_totalVolume;
            m_cachedDelta = delta;

            //MGlobal::displayInfo(MString("INIT FRAME 1. Mass: ") + mass_body + " Volume: " + m_cachedVolume);

            m_previousTime = currentFrame;
            m_isInitialized = true;

        }
        else if (m_isInitialized && currentFrame > m_previousTime) {
            // Calculate dt in seconds (Assuming 24 fps, you could also read this from Maya's MTime::uiUnit())
            double dt = (currentFrame - m_previousTime) / 24.0;
            int substeps = 10;
            double dt_sub = dt / substeps;

            // Flow Integrator setup
            FlowIntegrator integrator;

            double ref_area = 1.0;    // Reference area for drag
            bool include_drag = true;

            // Compute forces and integrate
            // Note: We need the current velocity (Y), which is K_inv * mu
            Vector6D Y = Vector6D(m_cachedK.inverse() * m_currentMu.data);
            Vector3d body_normal(0.0, 1.0, 0.0);
            Matrix3d R = m_currentG.data.block<3, 3>(0, 0);  // rotation part of SE3
            Vector3d leaf_normal_world = R * body_normal;

            // Transform body velocity to world frame for force computation
            Vector6d Y_world_data;
            Y_world_data << R * Y.omega(), R* Y.vel();
            Vector6D Y_world(Y_world_data);

            Vector6D F = integrator.compute_total_force(
                Y_world, mass_body, m_cachedVolume, rho_fluid,
                ref_area, C_d, C_l, k_ang,
                leaf_normal_world, include_drag
            );

            Vector6d F_body_data;
            F_body_data << R.transpose() * F.omega(),
                R.transpose()* F.vel();
            Vector6D F_body(F_body_data);
			//Vector6d F_world(F);

            if (plug == outTransform) {

                for (int s = 0; s < substeps; ++s) {
                    // Recompute R and normals each substep since G changes
                    R = m_currentG.data.block<3, 3>(0, 0);
                    leaf_normal_world = R.col(1);

                    Y = Vector6D(m_cachedK.inverse() * m_currentMu.data);
                    Y_world_data << R * Y.omega(), R* Y.vel();
                    Y_world = Vector6D(Y_world_data);

                    F = integrator.compute_total_force(
                        Y_world, mass_body, m_cachedVolume, rho_fluid,
                        ref_area, C_d, C_l, k_ang, leaf_normal_world, include_drag
                    );

                    F_body_data << R.transpose() * F.omega(), R.transpose()* F.vel();
                    F_body = Vector6D(F_body_data);

                    NewtonResult result = integrator.integrate_step_newton(
                        m_currentG, m_currentMu, m_cachedK, Vector6D(), F_body, dt_sub
                    );

                    if (!result.converged) break;

                    // Correct clamp - work in velocity space:
                    Matrix6d K_inv = m_cachedK.inverse();
                    Vector6d Y_next = K_inv * result.mu_next.data;
                    Vector3d ang_vel = Y_next.head<3>();
                    double max_ang_speed = M_PI; // 0.5 rev/sec

                    if (ang_vel.norm() > max_ang_speed) {
                        Vector6d Y_clamped = Y_next;
                        Y_clamped.head<3>() = ang_vel.normalized() * max_ang_speed;
                        // Convert back to momentum
                        m_currentMu = Vector6D(m_cachedK * Y_clamped);
                    }
                    else {
                        m_currentMu = result.mu_next;
                    }

                    m_currentG = result.g_next;
                }

                // Diagnostics using last substep's values
                MGlobal::displayInfo(MString("LeafNormal: ") +
                    leaf_normal_world.x() + " " + leaf_normal_world.y() + " " + leaf_normal_world.z());
                MGlobal::displayInfo(MString("NEW POS Y: ") + m_currentG.t()[1]);

                m_previousTime = currentFrame;
            }
        }
        else {
            // This branch means currentFrame <= m_previousTime (scrubbing backwards)
            MGlobal::displayWarning(MString("Backwards scrub detected at frame: ") + currentFrame
                + " prevTime was: " + m_previousTime);
            // Reset so next forward play re-initializes
            m_isInitialized = false;
        }

        //// OUTPUT THE MATRIX

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



        MDataHandle outHandle = data.outputValue(outTransform);
        outHandle.setMMatrix(outMat);
        data.setClean(plug);


        return MS::kSuccess;
    }
    return MS::kUnknownParameter; 
}