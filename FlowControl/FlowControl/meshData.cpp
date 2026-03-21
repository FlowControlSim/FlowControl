#include "meshData.h"
/*
MeshData::MeshData(const MDagPath& dagPath) :
    m_totalMass(0.0),
    m_totalVolume(0.0),
    m_centroid(0.0, 0.0, 0.0),
    m_hasMass(false),
    m_propertiesComputed(false) {
    MStatus status = extractMeshData(dagPath);
    if (status != MS::kSuccess) {
        MGlobal::displayError("MeshData: Failed to extract mesh data");
    }
}

MeshData::MeshData() :
	m_totalMass(0.0),
	m_totalVolume(0.0),
	m_centroid(0.0, 0.0, 0.0),
	m_hasMass(false),
	m_propertiesComputed(false)
{};

MStatus MeshData::setMassDensity(double totalMass,
    MassDensityType type = UNIFORM,
    const std::vector<double>* customDensity = nullptr) {
    if (totalMass <= 0) {
        MGlobal::displayError("MeshData: Total mass must be positive");
        return MS::kFailure;
	}
    if (type == CUSTOM && (customDensity == nullptr || customDensity->size() != m_vertices.length())) {
        return MS::kFailure;
    }
    if (type == VOLUME_WEIGHTED && !m_propertiesComputed) {
        MGlobal::displayError("MeshData: Must compute properties before setting volume-weighted mass density");
        return MS::kFailure;
	}

}


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
*/