#include "meshData.h"

MeshData::MeshData(const MDagPath& dagPath)
    : m_totalMass(0.0)
    , m_totalVolume(0.0)
    , m_centroid(0, 0, 0)
    , m_propertiesComputed(false)
    , m_hasMass(false)
{
    extractMeshData(dagPath);
}

MeshData::MeshData()
    : m_totalMass(0.0)
    , m_totalVolume(0.0)
    , m_centroid(0, 0, 0)
    , m_propertiesComputed(false)
    , m_hasMass(false)
{}

MStatus MeshData::extractMeshData(const MDagPath& dagPath) {
    MStatus status;

    // Store DAG path
    m_dagPath = dagPath;

    // Get mesh function set
    status = m_meshFn.setObject(dagPath);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get mesh function set");
        return status;
    }

    // Extract vertices in world space
    status = m_meshFn.getPoints(m_vertices, MSpace::kWorld);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get vertex positions");
        return status;
    }

    // Extract polygon connectivity
    status = m_meshFn.getVertices(m_polygonFaces, m_polygonFaces);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get polygon connectivity");
        return status;
    }

    // Triangulate (convert n-gons to triangles)
    status = triangulateMesh();
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Triangulation failed");
        return status;
    }

    MGlobal::displayInfo(MString("MeshData: Extracted ") +
        m_vertices.length() + " vertices, " +
        numTriangles() + " triangles");

    return MS::kSuccess;
}

MStatus MeshData::triangulateMesh() {
    MStatus status;
    m_triangleFaces.clear();

    MItMeshPolygon polyIter(m_dagPath, MObject::kNullObj, &status);
    CHECK_MSTATUS_AND_RETURN_IT(status);

    for (; !polyIter.isDone(); polyIter.next()) {
        // Get polygon vertex indices
        MIntArray vertexList;
        polyIter.getVertices(vertexList);

        unsigned int numVerts = vertexList.length();

        if (numVerts == 3) {
            // Already a triangle
            m_triangleFaces.append(vertexList[0]);
            m_triangleFaces.append(vertexList[1]);
            m_triangleFaces.append(vertexList[2]);
        }
        else if (numVerts == 4) {
            // Quad → 2 triangles
            // Triangle 1: v0, v1, v2
            m_triangleFaces.append(vertexList[0]);
            m_triangleFaces.append(vertexList[1]);
            m_triangleFaces.append(vertexList[2]);

            // Triangle 2: v0, v2, v3
            m_triangleFaces.append(vertexList[0]);
            m_triangleFaces.append(vertexList[2]);
            m_triangleFaces.append(vertexList[3]);
        }
        else {
            // N-gon → fan triangulation from first vertex
            for (unsigned int i = 1; i < numVerts - 1; ++i) {
                m_triangleFaces.append(vertexList[0]);
                m_triangleFaces.append(vertexList[i]);
                m_triangleFaces.append(vertexList[i + 1]);
            }
        }
    }

    return MS::kSuccess;
}

MStatus MeshData::setMassDensity(double totalMass,
    MassDensityType type,
    const std::vector<double>* customDensity) {
    unsigned int N = m_vertices.length();
    m_massDensity.resize(N);
    m_totalMass = totalMass;

    if (type == UNIFORM) {
        // Equal mass per vertex
        double massPerVertex = totalMass / N;
        for (unsigned int i = 0; i < N; ++i) {
            m_massDensity[i] = massPerVertex;
        }
    }
    else if (type == VOLUME_WEIGHTED) {
        // Mass proportional to local area (Voronoi approximation)
        std::vector<double> vertexAreas(N, 0.0);

        // Sum adjacent face areas (divided by 3)
        unsigned int numTris = numTriangles();
        for (unsigned int i = 0; i < numTris; ++i) {
            unsigned int v0 = m_triangleFaces[3 * i];
            unsigned int v1 = m_triangleFaces[3 * i + 1];
            unsigned int v2 = m_triangleFaces[3 * i + 2];

            double area = triangleArea(m_vertices[v0],
                m_vertices[v1],
                m_vertices[v2]);

            vertexAreas[v0] += area / 3.0;
            vertexAreas[v1] += area / 3.0;
            vertexAreas[v2] += area / 3.0;
        }

        // Normalize to total mass
        double totalArea = 0.0;
        for (unsigned int i = 0; i < N; ++i) {
            totalArea += vertexAreas[i];
        }

        for (unsigned int i = 0; i < N; ++i) {
            m_massDensity[i] = totalMass * vertexAreas[i] / totalArea;
        }
    }
    else if (type == CUSTOM) {
        if (!customDensity || customDensity->size() != N) {
            MGlobal::displayError("MeshData: Custom density array size mismatch");
            return MS::kFailure;
        }
        m_massDensity = *customDensity;
    }

    m_hasMass = true;
    return MS::kSuccess;
}

double MeshData::getMassDensity(unsigned int i) const {
    if (i >= m_massDensity.size()) {
        return 0.0;
    }
    return m_massDensity[i];
}

const MVector& MeshData::getVertexNormal(unsigned int i) const {
    return m_vertexNormals[i];
}

const MVector& MeshData::getFaceNormal(unsigned int i) const {
    return m_faceNormals[i];
}

double MeshData::getFaceArea(unsigned int i) const {
    return m_faceAreas[i];
}

const MPoint& MeshData::getFaceCenter(unsigned int i) const {
    return m_faceCenters[i];
}

double MeshData::getMeanCurvature(unsigned int i) const {
    if (i >= m_meanCurvatures.size()) return 0.0;
    return m_meanCurvatures[i];
}

MStatus MeshData::computeProperties() {
    MStatus status;

    // Compute in order (dependencies)
    status = computeFaceProperties();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = computeVertexNormals();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = computeMeanCurvatures();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = computeVolume();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    status = computeCentroid();
    CHECK_MSTATUS_AND_RETURN_IT(status);

    m_propertiesComputed = true;

    MGlobal::displayInfo("MeshData: Properties computed successfully");
    return MS::kSuccess;
}

MStatus MeshData::computeFaceProperties() {
    unsigned int numTris = numTriangles();

    m_faceNormals.setLength(numTris);
    m_faceAreas.resize(numTris);
    m_faceCenters.setLength(numTris);

    for (unsigned int i = 0; i < numTris; ++i) {
        unsigned int v0 = m_triangleFaces[3 * i];
        unsigned int v1 = m_triangleFaces[3 * i + 1];
        unsigned int v2 = m_triangleFaces[3 * i + 2];

        MPoint p0 = m_vertices[v0];
        MPoint p1 = m_vertices[v1];
        MPoint p2 = m_vertices[v2];

        // Edge vectors
        MVector edge1 = p1 - p0;
        MVector edge2 = p2 - p0;

        // Normal from cross product
        MVector cross = edge1 ^ edge2;  // ^ is cross product in Maya
        double area = 0.5 * cross.length();

        m_faceAreas[i] = area;

        if (area > 1e-10) {
            m_faceNormals[i] = cross.normal();  // Normalize
        }
        else {
            m_faceNormals[i] = MVector(0, 1, 0);  // Degenerate triangle
        }

        // Centroid
        m_faceCenters[i] = (p0 + p1 + p2) / 3.0;
    }

    return MS::kSuccess;
}

MStatus MeshData::computeVertexNormals() {
    unsigned int N = m_vertices.length();
    m_vertexNormals.setLength(N);

    // Initialize to zero
    for (unsigned int i = 0; i < N; ++i) {
        m_vertexNormals[i] = MVector(0, 0, 0);
    }

    // Area-weighted average of adjacent face normals
    unsigned int numTris = numTriangles();
    for (unsigned int i = 0; i < numTris; ++i) {
        unsigned int v0 = m_triangleFaces[3 * i];
        unsigned int v1 = m_triangleFaces[3 * i + 1];
        unsigned int v2 = m_triangleFaces[3 * i + 2];

        MVector weightedNormal = m_faceNormals[i] * m_faceAreas[i];

        m_vertexNormals[v0] += weightedNormal;
        m_vertexNormals[v1] += weightedNormal;
        m_vertexNormals[v2] += weightedNormal;
    }

    // Normalize
    for (unsigned int i = 0; i < N; ++i) {
        double length = m_vertexNormals[i].length();
        if (length > 1e-10) {
            m_vertexNormals[i] /= length;
        }
        else {
            m_vertexNormals[i] = MVector(0, 1, 0);
        }
    }

    return MS::kSuccess;
}

MStatus MeshData::computeMeanCurvatures() {
    unsigned int N = m_vertices.length();
    m_meanCurvatures.resize(N, 0.0);

    // Simplified mean curvature approximation
    // For production: use cotangent Laplacian from libigl or similar

    MItMeshVertex vertIter(m_dagPath);
    for (unsigned int i = 0; !vertIter.isDone(); vertIter.next(), ++i) {
        // Get 1-ring neighbors
        MIntArray connectedVertices;
        vertIter.getConnectedVertices(connectedVertices);

        if (connectedVertices.length() < 2) {
            m_meanCurvatures[i] = 0.0;
            continue;
        }

        MPoint vi = m_vertices[i];

        // Average distance to neighbors (approximate local radius)
        double avgDist = 0.0;
        for (unsigned int j = 0; j < connectedVertices.length(); ++j) {
            MPoint vj = m_vertices[connectedVertices[j]];
            avgDist += vi.distanceTo(vj);
        }
        avgDist /= connectedVertices.length();

        // Mean curvature ≈ 1 / local_radius
        // (Simplified - for accurate results, use cotangent weights)
        if (avgDist > 1e-10) {
            m_meanCurvatures[i] = 1.0 / avgDist;
        }
        else {
            m_meanCurvatures[i] = 0.0;
        }
    }

    return MS::kSuccess;

}

MStatus MeshData::computeVolume() {
    // Signed volume using divergence theorem
    // V = (1/6) Σ (v0 · (v1 × v2))

    m_totalVolume = 0.0;
    unsigned int numTris = numTriangles();

    for (unsigned int i = 0; i < numTris; ++i) {
        MPoint v0 = m_vertices[m_triangleFaces[3 * i]];
        MPoint v1 = m_vertices[m_triangleFaces[3 * i + 1]];
        MPoint v2 = m_vertices[m_triangleFaces[3 * i + 2]];

        MVector v0vec(v0.x, v0.y, v0.z);
        MVector v1vec(v1.x, v1.y, v1.z);
        MVector v2vec(v2.x, v2.y, v2.z);

        // Scalar triple product
        m_totalVolume += (v0vec * (v1vec ^ v2vec)) / 6.0;
    }

    m_totalVolume = fabs(m_totalVolume);
    return MS::kSuccess;
}

MStatus MeshData::computeCentroid() {
    if (!m_hasMass) {
        MGlobal::displayWarning("MeshData: Computing centroid without mass (using geometric center)");

        // Geometric center
        MVector sum(0, 0, 0);
        unsigned int N = m_vertices.length();
        for (unsigned int i = 0; i < N; ++i) {
            sum += MVector(m_vertices[i]);
        }
        m_centroid = MPoint(sum / N);
    }
    else {
        // Center of mass
        MVector sum(0, 0, 0);
        for (unsigned int i = 0; i < m_vertices.length(); ++i) {
            sum += MVector(m_vertices[i]) * m_massDensity[i];
        }
        m_centroid = MPoint(sum / m_totalMass);
    }

    return MS::kSuccess;
}

MStatus MeshData::validate() const {
    if (m_vertices.length() == 0) {
        MGlobal::displayError("MeshData: No vertices");
        return MS::kFailure;
    }

    if (m_triangleFaces.length() == 0) {
        MGlobal::displayError("MeshData: No triangles");
        return MS::kFailure;
    }

    // Check face indices
    unsigned int maxIdx = 0;
    for (unsigned int i = 0; i < m_triangleFaces.length(); ++i) {
        if (m_triangleFaces[i] > maxIdx) {
            maxIdx = m_triangleFaces[i];
        }
    }

    if (maxIdx >= m_vertices.length()) {
        MGlobal::displayError(MString("MeshData: Invalid face index ") + maxIdx);
        return MS::kFailure;
    }

    if (!m_propertiesComputed) {
        MGlobal::displayWarning("MeshData: Properties not computed");
    }

    if (!m_hasMass) {
        MGlobal::displayWarning("MeshData: Mass not set");
    }

    MGlobal::displayInfo("MeshData: Validation passed");
    return MS::kSuccess;
}

void MeshData::printInfo() const {
    std::ostringstream oss;
    oss << "══════════════════════════════════════════════════\n";
    oss << "MESH INFORMATION\n";
    oss << "══════════════════════════════════════════════════\n";
    oss << "Vertices:  " << m_vertices.length() << "\n";
    oss << "Triangles: " << numTriangles() << "\n";
    oss << "Volume:    " << m_totalVolume << " m³\n";

    if (m_hasMass) {
        oss << "Mass:      " << m_totalMass << " kg\n";
        oss << "Density:   " << (m_totalMass / m_totalVolume) << " kg/m³\n";
    }

    oss << "Centroid:  (" << m_centroid.x << ", "
        << m_centroid.y << ", " << m_centroid.z << ")\n";
    oss << "Properties computed: " << (m_propertiesComputed ? "Yes" : "No") << "\n";
    oss << "══════════════════════════════════════════════════";

    MGlobal::displayInfo(MString(oss.str().c_str()));
}

void MeshData::skewSymmetric(const MVector& v, double skew[9]) {
    skew[0] = 0.0;
    skew[1] = -v.z;
    skew[2] = v.y;
    skew[3] = v.z;
    skew[4] = 0.0;
    skew[5] = -v.x;
    skew[6] = -v.y;
	skew[7] = v.x;
    skew[8] = 0.0;
}

double MeshData::triangleArea(const MPoint& p0, const MPoint& p1, const MPoint& p2) {

    return 0.5 * ((p1 - p0) ^ (p2 - p0)).length();
}


/*
* 
* 
* 
* 
* 
* 
// ══════════════════════════════════════════════════════════════════════════
// PHYSICS COMPUTATIONS
// ══════════════════════════════════════════════════════════════════════════

MStatus MeshData::computeBodyInertia(double K[36]) const {
    if (!m_hasMass) {
        MGlobal::displayError("MeshData: Cannot compute inertia without mass");
        return MS::kFailure;
    }

    // Initialize to zero
    for (int i = 0; i < 36; ++i) K[i] = 0.0;

    // K = [[I_ωω, I_ωv],
    //      [I_vω, I_vv]]
    //
    // I_ωω = Σ ρ_i [γ_i]×^T [γ_i]×  (3×3)
    // I_ωv = Σ ρ_i [γ_i]×            (3×3)
    // I_vv = Σ ρ_i I                 (3×3)

    unsigned int N = m_vertices.length();

    for (unsigned int i = 0; i < N; ++i) {
        MVector gamma(m_vertices[i].x, m_vertices[i].y, m_vertices[i].z);
        double rho = m_massDensity[i];

        // Skew-symmetric matrix [γ]×
        double gammaX[9];
        skewSymmetric(gamma, gammaX);

        // I_ωω contribution: [γ]×^T [γ]×
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                double sum = 0.0;
                for (int k = 0; k < 3; ++k) {
                    sum += gammaX[k*3 + row] * gammaX[k*3 + col];  // Transpose × matrix
                }
                K[row*6 + col] += rho * sum;
            }
        }

        // I_ωv contribution: [γ]×
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                K[row*6 + (col+3)] += rho * gammaX[row*3 + col];
            }
        }

        // I_vv contribution: identity matrix
        K[3*6 + 3] += rho;  // (3,3)
        K[4*6 + 4] += rho;  // (4,4)
        K[5*6 + 5] += rho;  // (5,5)
    }

    // I_vω = I_ωv^T (by symmetry)
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            K[(row+3)*6 + col] = K[row*6 + (col+3)];
        }
    }

    return MS::kSuccess;
}

MStatus MeshData::computeBodyMomentum(const MeshData& prevMesh,
                                      double dt,
                                      double mu[6]) const {
    if (!m_hasMass) {
        MGlobal::displayError("MeshData: Cannot compute momentum without mass");
        return MS::kFailure;
    }

    if (prevMesh.numVertices() != numVertices()) {
        MGlobal::displayError("MeshData: Vertex count mismatch for momentum computation");
        return MS::kFailure;
    }

    // Initialize
    for (int i = 0; i < 6; ++i) mu[i] = 0.0;

    // μ⁰ = [Σ ρ γ × γ̇,  Σ ρ γ̇]
    // where γ̇ = (γ_{k+1} - γ_k) / dt

    unsigned int N = numVertices();

    MVector angularMomentum(0, 0, 0);
    MVector linearMomentum(0, 0, 0);

    for (unsigned int i = 0; i < N; ++i) {
        MVector gamma_k(prevMesh.m_vertices[i].x,
                       prevMesh.m_vertices[i].y,
                       prevMesh.m_vertices[i].z);

        MVector gamma_k1(m_vertices[i].x,
                        m_vertices[i].y,
                        m_vertices[i].z);

        MVector gamma_dot = (gamma_k1 - gamma_k) / dt;
        double rho = m_massDensity[i];

        // Angular: γ × γ̇
        angularMomentum += (gamma_k ^ gamma_dot) * rho;

        // Linear: γ̇
        linearMomentum += gamma_dot * rho;
    }

    mu[0] = angularMomentum.x;
    mu[1] = angularMomentum.y;
    mu[2] = angularMomentum.z;
    mu[3] = linearMomentum.x;
    mu[4] = linearMomentum.y;
    mu[5] = linearMomentum.z;

    return MS::kSuccess;
}

MStatus MeshData::computeAddedMass(double rhoFluid, double M_added[36]) const {
    // Initialize to zero
    for (int i = 0; i < 36; ++i) M_added[i] = 0.0;

    // Brennen formula: M_added = ρ_fluid ∫ H² n̂ ⊗ n̂ dS
    //
    // For translational added mass only (rotational typically negligible):
    // M_vv = ρ_fluid Σ H_i² (n̂_i ⊗ n̂_i) A_i

    unsigned int N = numVertices();

    for (unsigned int i = 0; i < N; ++i) {
        double H = m_meanCurvatures[i];
        MVector n = m_vertexNormals[i];

        // Approximate area per vertex (1/N of total surface area)
        double totalArea = 0.0;
        for (unsigned int j = 0; j < numTriangles(); ++j) {
            totalArea += m_faceAreas[j];
        }
        double areaPerVertex = totalArea / N;

        // Tensor product: n ⊗ n
        // Contribution to M_vv (lower-right 3×3 block)
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                double n_i = (row == 0) ? n.x : (row == 1) ? n.y : n.z;
                double n_j = (col == 0) ? n.x : (col == 1) ? n.y : n.z;

                M_added[(row+3)*6 + (col+3)] += rhoFluid * H*H * n_i*n_j * areaPerVertex;
            }
        }
    }

    return MS::kSuccess;
}


// ══════════════════════════════════════════════════════════════════════════
// EXTERNAL FORCE COMPUTATIONS (Free Functions)
// ══════════════════════════════════════════════════════════════════════════

void computeGravityBuoyancy(const MeshData& mesh,
                           double rhoFluid,
                           const MVector& gravity,
                           double force[6]) {
    // F = (m_body - m_displaced) * g
    // where m_displaced = ρ_fluid * V

    double massDisplaced = rhoFluid * mesh.getTotalVolume();
    double netMass = mesh.getTotalMass() - massDisplaced;

    MVector F = gravity * netMass;

    // No torque from uniform gravity (acts at center of mass)
    force[0] = 0.0;
    force[1] = 0.0;
    force[2] = 0.0;
    force[3] = F.x;
    force[4] = F.y;
    force[5] = F.z;
}

void computeLiftDragMesh(const MeshData& mesh,
                        const double velocity[6],
                        double rhoFluid,
                        double Cd,
                        double Cl,
                        double force[6]) {
    // Initialize
    for (int i = 0; i < 6; ++i) force[i] = 0.0;

    MVector omega(velocity[0], velocity[1], velocity[2]);
    MVector v(velocity[3], velocity[4], velocity[5]);

    MVector totalForce(0, 0, 0);
    MVector totalTorque(0, 0, 0);

    unsigned int numTris = mesh.numTriangles();

    for (unsigned int i = 0; i < numTris; ++i) {
        // Position of face center
        MPoint center = mesh.getFaceCenter(i);
        MVector r(center.x, center.y, center.z);

        // Velocity at this point: v_point = v + ω × r
        MVector v_point = v + (omega ^ r);
        double v_norm = v_point.length();

        if (v_norm < 1e-6) continue;

        // Normal and tangent components
        MVector n = mesh.getFaceNormal(i);
        MVector v_normal = (v_point * n) * n;
        MVector v_tangent = v_point - v_normal;

        // Dynamic pressure
        double q = 0.5 * rhoFluid * v_norm * v_norm;
        double area = mesh.getFaceArea(i);

        // Drag (normal) and lift (tangent)
        MVector F_drag(0, 0, 0);
        MVector F_lift(0, 0, 0);

        double v_normal_len = v_normal.length();
        if (v_normal_len > 1e-10) {
            F_drag = -(Cd * q * area / v_normal_len) * v_normal;
        }

        double v_tangent_len = v_tangent.length();
        if (v_tangent_len > 1e-10) {
            F_lift = -(Cl * q * area / v_tangent_len) * v_tangent;
        }

        MVector F_face = F_drag + F_lift;

        // Accumulate
        totalForce += F_face;
        totalTorque += (r ^ F_face);  // r × F
    }

    force[0] = totalTorque.x;
    force[1] = totalTorque.y;
    force[2] = totalTorque.z;
    force[3] = totalForce.x;
    force[4] = totalForce.y;
    force[5] = totalForce.z;
}

void computeTotalForce(const MeshData& mesh,
                      const double velocity[6],
                      double rhoFluid,
                      const MVector& gravity,
                      double Cd,
                      double Cl,
                      bool includeDrag,
                      double force[6]) {
    double F_grav[6];
    computeGravityBuoyancy(mesh, rhoFluid, gravity, F_grav);

    if (includeDrag) {
        double F_drag[6];
        computeLiftDragMesh(mesh, velocity, rhoFluid, Cd, Cl, F_drag);

        for (int i = 0; i < 6; ++i) {
            force[i] = F_grav[i] + F_drag[i];
        }
    } else {
        for (int i = 0; i < 6; ++i) {
            force[i] = F_grav[i];
        }
    }
}


*/

