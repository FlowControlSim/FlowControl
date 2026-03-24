#include "meshData.h"

MeshData::MeshData(const MObject& meshObj)
    : m_totalMass(0.0)
    , m_totalVolume(0.0)
    , m_centroid(0, 0, 0)
    , m_propertiesComputed(false)
    , m_hasMass(false)
{
    extractMeshData(meshObj);
}

MeshData::MeshData()
    : m_totalMass(0.0)
    , m_totalVolume(0.0)
    , m_centroid(0, 0, 0)
    , m_propertiesComputed(false)
    , m_hasMass(false)
{}

MStatus MeshData::extractMeshData(const MObject& meshObj) {
    MStatus status;

    status = m_meshFn.setObject(meshObj);
    CHECK_MSTATUS_AND_RETURN_IT(status);
    // Extract vertices in world space
    status = m_meshFn.getPoints(m_vertices, MSpace::kWorld);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get vertex positions");
        return status;
    }

    // Extract polygon connectivity
    status = m_meshFn.getVertices(m_triangleFaces, m_polygonFaces);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get polygon vertex connectivity");
        return status;
    }

    MIntArray triangleCounts; 
    status = m_meshFn.getTriangles(triangleCounts, m_triangleFaces);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to get polygon triangle connectivity");
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

    MObject meshObj = m_meshFn.object();
    MItMeshPolygon polyIter(meshObj, &status);
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

    MStatus status;

    // Simplified mean curvature approximation
    // For production: use cotangent Laplacian from libigl or similar
    MObject localMeshObj = m_meshFn.object();

    MItMeshVertex vertIter(localMeshObj, &status);
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
    /*
    // Check face indices
    unsigned int maxIdx = 0;
    for (unsigned int i = 0; i < m_triangleFaces.length(); ++i) {
        if (m_triangleFaces[i*3] > maxIdx) {
            maxIdx = m_triangleFaces[i];
        }
    }

    if (maxIdx >= m_vertices.length()) {
        MGlobal::displayError(MString("MeshData: Invalid face index ") + maxIdx);
        return MS::kFailure;
    }*/

    if (!m_propertiesComputed) {
        MGlobal::displayWarning("MeshData: Properties not computed");
    }

    if (!m_hasMass) {
        MGlobal::displayWarning("MeshData: Mass not set");
    }

    MGlobal::displayInfo("MeshData: Validation passed");
    return MS::kSuccess;
}
/*
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
}*/

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

