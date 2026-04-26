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
MObject testScene::targetMatrix;
MObject testScene::animStiffness;
MObject testScene::animDamping;
MObject testScene::windVelocity;
MObject testScene::initLinearVel;
MObject testScene::initAngularVel;

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

    liftCoeff = nAttr.create("liftCoeff", "lc", MFnNumericData::kFloat, 0.8f);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(liftCoeff);

    angularDrag = nAttr.create("angularDrag", "ad", MFnNumericData::kFloat, 1.0f);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(angularDrag);

    targetMatrix = mAttr.create("targetMatrix", "tm");
    mAttr.setStorable(true);
    mAttr.setConnectable(true);
    addAttribute(targetMatrix);

    animStiffness = nAttr.create("animStiffness", "as", MFnNumericData::kDouble, 0.0);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(animStiffness);

    animDamping = nAttr.create("animDamping", "ad2", MFnNumericData::kDouble, 0.0);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(animDamping);

    windVelocity = nAttr.create("windVelocity", "wv", MFnNumericData::k3Double);
    nAttr.setDefault(0.0, 0.0, 0.0);
    nAttr.setStorable(true);
    nAttr.setConnectable(true);
    nAttr.setKeyable(true);
    addAttribute(windVelocity);

    initLinearVel = nAttr.create("initialLinearVel", "ilv", MFnNumericData::k3Double);
    nAttr.setDefault(0.0, 0.0, 0.0);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    addAttribute(initLinearVel);

    initAngularVel = nAttr.create("initialAngularVel", "iav", MFnNumericData::k3Double);
    nAttr.setDefault(0.001, 0.00, 0.0);
    nAttr.setStorable(true);
    nAttr.setKeyable(true);
    addAttribute(initAngularVel);

    attributeAffects(testScene::liftCoeff, testScene::outTransform);
    attributeAffects(testScene::angularDrag, testScene::outTransform);
    attributeAffects(testScene::inTime, testScene::outTransform);
    attributeAffects(testScene::fluidDensity, testScene::outTransform);
    attributeAffects(testScene::dragCoeff, testScene::outTransform);
    attributeAffects(testScene::mass, testScene::outTransform);
    attributeAffects(testScene::inMesh, testScene::outTransform);
    attributeAffects(testScene::targetMatrix, testScene::outTransform);
    attributeAffects(testScene::animStiffness, testScene::outTransform);
    attributeAffects(testScene::animDamping, testScene::outTransform);
    attributeAffects(testScene::windVelocity, testScene::outTransform);
    attributeAffects(testScene::initLinearVel, testScene::outTransform);
    attributeAffects(testScene::initAngularVel, testScene::outTransform);


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


    if (plug == outTransform || plug == inTime) {
        //MTime currentTime = data.inputValue(inTime).asTime();
        //double currentFrame = currentTime.value();

        const double MAYA_TO_METERS = 1.0; // 0.01

        double mass_body = data.inputValue(testScene::mass, &returnStatus).asDouble();
        double C_d = data.inputValue(testScene::dragCoeff, &returnStatus).asDouble();
        double C_l = data.inputValue(testScene::liftCoeff, &returnStatus).asDouble();
        double k_ang = data.inputValue(testScene::angularDrag, &returnStatus).asDouble();
        double rho_fluid = data.inputValue(testScene::fluidDensity, &returnStatus).asDouble();
        MObject meshObj = data.inputValue(inMesh).asMesh();
        double stiffness = data.inputValue(testScene::animStiffness).asDouble();
        double anim_damp = data.inputValue(testScene::animDamping).asDouble();
        MMatrix targetMat = data.inputValue(testScene::targetMatrix).asMatrix();
        Vector3d target_pos(targetMat[3][0] * MAYA_TO_METERS, targetMat[3][1] * MAYA_TO_METERS, targetMat[3][2] * MAYA_TO_METERS);


        // 1. INITIALIZATION (Frame 1 or earlier)
        if (!m_isInitialized || currentFrame <= 1.0) {

            // Read mesh from MDataBlock
            MeshData meshData(meshObj);

            // Set mass before computing properties so centroid/inertia have weights
            // (Assuming you have a mass attribute, or hardcode for now)
            meshData.setMassDensity(mass_body, MassDensityType::VOLUME_WEIGHTED, nullptr);
            meshData.computeProperties();

            m_cachedVertices.clear();
            m_cachedVertices.reserve(meshData.m_vertices.length());
            for (unsigned int i = 0; i < meshData.m_vertices.length(); ++i) {
                m_cachedVertices.push_back(Vector3d(meshData.m_vertices[i].x * MAYA_TO_METERS, 
                    meshData.m_vertices[i].y * MAYA_TO_METERS,
                    meshData.m_vertices[i].z * MAYA_TO_METERS));
            }

            // Compute and CACHE the inertia matrix
            FlowIntegrator flowC;
            Matrix6d I_body = flowC.compute_body_inertia(m_cachedVertices, meshData.m_massDensity);
            Matrix6d I_fluid = flowC.compute_added_mass_tensor(meshData, rho_fluid);
            m_cachedK = I_body + I_fluid;

            double3& iav = data.inputValue(testScene::initAngularVel).asDouble3();
            double3& ilv = data.inputValue(testScene::initLinearVel).asDouble3();

            m_cachedVolume = meshData.m_totalVolume * MAYA_TO_METERS * MAYA_TO_METERS * MAYA_TO_METERS;
            m_currentG = identity();
            vec6 init;
            init << iav[0], iav[1], iav[2],   // small angular momentum (wx, wy, wz) // 0.05, 0.01, 0.0, 
                ilv[0] * MAYA_TO_METERS,
                ilv[1] * MAYA_TO_METERS,
                ilv[2] * MAYA_TO_METERS;  // zero linear momentum
            m_currentMu = Vector6D(init);

            //MGlobal::displayInfo(MString("INIT FRAME 1. Mass: ") + mass_body + " Volume: " + m_cachedVolume);
            MGlobal::displayInfo(MString("CACHED VOLUME (m^3): ") + m_cachedVolume);

            m_previousTime = currentFrame;
            m_isInitialized = true;

        }
        else if (m_isInitialized && currentFrame > m_previousTime) {
            // Calculate dt in seconds (Assuming 24 fps, you could also read this from Maya's MTime::uiUnit())
            double dt = (currentFrame - m_previousTime) / 24.0;

            // get vertices at current frame
            MeshData meshData(meshObj);
            meshData.setMassDensity(mass_body, MassDensityType::UNIFORM, nullptr);
            meshData.computeProperties();

            std::vector<Vector3d> vertices_k1;
            vertices_k1.reserve(meshData.numVertices());
            for (unsigned int i = 0; i < meshData.numVertices(); ++i) {
                vertices_k1.emplace_back(meshData.m_vertices[i].x * MAYA_TO_METERS,
                    meshData.m_vertices[i].y * MAYA_TO_METERS,
                    meshData.m_vertices[i].z * MAYA_TO_METERS);
            }

            // Flow Integrator setup
            FlowIntegrator integrator;

            double ref_area = 1.0;    // Reference area for drag
            bool include_drag = true;

            // compute all forces from algorithms
            Matrix6d I_body = integrator.compute_body_inertia(vertices_k1, meshData.m_massDensity);
            Vector6D mu0_body = integrator.compute_body_momentum(m_cachedVertices, vertices_k1, meshData.m_massDensity, dt);

            Matrix6d I_fluid = integrator.compute_added_mass_tensor(meshData, rho_fluid);
            Vector6D mu0_fluid = integrator.compute_fluid_momentum(m_cachedVertices, vertices_k1, meshData, rho_fluid, dt);

            const Matrix6d K_t = I_body + I_fluid;
            const Vector6D mu0 = mu0_body + mu0_fluid;

            // current velocity Y, which is K_inv * mu
            const Vector6D Y = Vector6D(K_t.inverse() * (m_currentMu.data - mu0.data));
            Matrix3d R = m_currentG.data.block<3, 3>(0, 0);
            double net_weight = mass_body - m_cachedVolume * rho_fluid;
            Vector3d g_body = R.transpose() * Vector3d(0.0, -9.81 * net_weight, 0.0);
            Vector6d F_grav_data;
            F_grav_data << Vector3d::Zero(), g_body;

            // pos_error is in world space
            Vector3d pos_error = target_pos - m_currentG.t();
            Vector3d v_world = R * Y.vel();
            Vector3d f_guide_world = (stiffness * pos_error) - (anim_damp * v_world);

            // Transform final force into body frame
            Vector3d f_guide_body = R.transpose() * f_guide_world;
            Vector6d F_guide_data;
            F_guide_data << Vector3d::Zero(), f_guide_body;

            // only y-independent forces for now
            Vector6D F_external(F_grav_data + F_guide_data);

            // Read wind � authored in Maya world space cm/s
            double3& windRaw = data.inputValue(testScene::windVelocity).asDouble3();
            Vector3d wind_world(windRaw[0] * MAYA_TO_METERS,
                windRaw[1] * MAYA_TO_METERS,
                windRaw[2] * MAYA_TO_METERS);

            // Rotate into body frame � lift/drag operates in body frame
            Vector3d wind_body = R.transpose() * wind_world;


            NewtonResult result = integrator.integrate_step_newton(m_currentG, m_currentMu, K_t, mu0, F_external, dt, meshData,
                m_cachedVertices, vertices_k1,
                rho_fluid, k_ang, wind_body);


            /*
            MGlobal::displayInfo(MString("SIM STEP | Frame: ") + currentFrame +
            " | Force Y: " + F.vel()[1] +
            " | Residual: " + result.residual);
            */
            // Update internal Node State
            m_currentG = result.g_next;
            m_currentMu = result.mu_next;
            m_cachedVertices = vertices_k1;
            m_cachedK = K_t;
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

        const double M_INV = 1.0; // 100.0
        //// OUTPUT THE MATRIX

        MMatrix outMat;
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                outMat[r][c] = m_currentG.data(c, r);
            }
        }

        outMat[3][0] = m_currentG.data(0, 3) * M_INV;
        outMat[3][1] = m_currentG.data(1, 3) * M_INV;
        outMat[3][2] = m_currentG.data(2, 3) * M_INV;
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