#ifndef MESHDATA_H
#define MESHDATA_H

#include <maya/MFnMesh.h>
#include <maya/MDagPath.h>
#include <maya/MPoint.h>
#include <maya/MVector.h>
#include <maya/MPointArray.h>
#include <maya/MVectorArray.h>
#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>
#include <maya/MMatrix.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MStatus.h>
#include <maya/MGlobal.h>


#include "helpers.h"

enum MassDensityType {
    UNIFORM,          // Equal mass per vertex
    VOLUME_WEIGHTED,  // Mass proportional to Voronoi volume
    CUSTOM            // User-provided array
};

class MeshData {
public:
  
    MeshData(const MDagPath& dagPath); 
    MeshData();
    ~MeshData() {};

    // CORE GEOMETRY

    const MPointArray& getVertices() const { return m_vertices; }
    const MIntArray& getFaces() const { return m_triangleFaces; }

    unsigned int numVertices() const { return m_vertices.length(); }
    unsigned int numTriangles() const { return m_triangleFaces.length() / 3; }

    // MASS DISTRIBUTION

    /// Set mass density distribution
    /// @param totalMass Total mass in kg
    /// @param type Distribution type (UNIFORM, VOLUME_WEIGHTED, CUSTOM)
    /// @param customDensity Custom density array (only if type == CUSTOM)
    MStatus setMassDensity(double totalMass,
        MassDensityType type = UNIFORM,
        const std::vector<double>* customDensity = nullptr);

	double getMassDensity(unsigned int i) const {
        if (i >= m_massDensity.size()) {
            return 0.0;
        }
        return m_massDensity[i]; }
    double getTotalMass() const { return m_totalMass; }

    // COMPUTED PROPERTIES

    /// Compute all derived properties (normals, areas, curvature, volume)
    /// Call this after setting mass density
    MStatus computeProperties();

    const MVector& getVertexNormal(unsigned int i) const;
    const MVector& getFaceNormal(unsigned int i) const;
    double getFaceArea(unsigned int i) const;
    const MPoint& getFaceCenter(unsigned int i) const;
    double getMeanCurvature(unsigned int i) const;
    double getTotalVolume() const { return m_totalVolume; }
    const MPoint& getCentroid() const { return m_centroid; }

    MStatus validate() const;
    void printInfo() const;
    bool hasProperties() const { return m_propertiesComputed; }

private:

    MStatus extractMeshData(const MDagPath& dagPath);     /// Extract mesh data from Maya
    MStatus triangulateMesh();                            /// Triangulate Maya polygons (handles n-gons)
    MStatus computeFaceProperties();                      /// Compute face normals and areas
    MStatus computeVertexNormals();                       /// Compute vertex normals (area-weighted average)
    MStatus computeMeanCurvatures();                      /// Compute mean curvature using cotangent Laplacian
    MStatus computeVolume();                              /// Compute mesh volume (for closed meshes)
    MStatus computeCentroid();                            /// Compute center of mass

    static void skewSymmetric(const MVector& v, double skew[9]);     /// Helper: Compute 3×3 skew-symmetric matrix from vector
    static double triangleArea(const MPoint& p0, const MPoint& p1, const MPoint& p2);     /// Helper: Triangle area from 3 points

    // Core geometry
    MPointArray     m_vertices;        // (N,) vertex positions in world space
    MIntArray       m_polygonFaces;    // Maya polygon vertex indices
    MIntArray       m_triangleFaces;   // Triangulated faces (every 3 = 1 triangle)

    MFnMesh         m_meshFn;
    MDagPath        m_dagPath;

    std::vector<double> m_massDensity; // (N,) mass at each vertex (kg)
    double              m_totalMass;   // Total mass (kg)

    // Computed surface properties
    MVectorArray    m_vertexNormals;   // (N,) normals at vertices
    MVectorArray    m_faceNormals;     // (F,) normals per triangle
    std::vector<double> m_faceAreas;   // (F,) area per triangle (m²)
    MPointArray     m_faceCenters;     // (F,) centroid per triangle
    std::vector<double> m_meanCurvatures; // (N,) mean curvature H at vertices

    // Derived quantities
    double          m_totalVolume;     // Enclosed volume (m³)
    MPoint          m_centroid;        // Center of mass

    // Status flags
    bool            m_propertiesComputed;
    bool            m_hasMass;
};

#endif // MESHDATA_H
