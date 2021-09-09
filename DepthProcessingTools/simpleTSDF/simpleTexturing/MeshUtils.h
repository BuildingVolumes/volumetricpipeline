#pragma once

#include <OpenMesh/Core/Mesh/Attributes.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Tools/Subdivider/Uniform/CatmullClarkT.hh>
#include <OpenMesh/Tools/Subdivider/Adaptive/Composite/CompositeT.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>

#define LARGE_F 1000000.f
struct MyTraits : public OpenMesh::DefaultTraits
{
    typedef OpenMesh::Vec3f Point;
    VertexAttributes(OpenMesh::Attributes::Normal | OpenMesh::Attributes::Color | OpenMesh::Attributes::TexCoord2D );
    HalfedgeAttributes(OpenMesh::Attributes::PrevHalfedge);
    FaceAttributes(OpenMesh::Attributes::Normal);
    FaceTraits
    {
    public:
        float energy;
        int camera;
        FaceT() : energy(LARGE_F), camera(-1) {}
        /* const float energy() const { return energy; }
         const int camera() const { return camera; }
         void set_energy(const float& f) { energy = f; }
         void set_camera(const int& ind) { camera = ind; }*/
    };
    VertexTraits
    {
    public:
        OpenMesh::FaceHandle fh;
    };


};

typedef OpenMesh::TriMesh_ArrayKernelT<MyTraits> MyMesh;

static unsigned int get_vhandles(MyMesh& mesh_, OpenMesh::FaceHandle _fh, std::vector<OpenMesh::VertexHandle>& _vhandles) {
    unsigned int count(0);
    _vhandles.clear();
    for (MyMesh::CFVIter fv_it = mesh_.cfv_iter(_fh); fv_it.is_valid(); ++fv_it) {
        _vhandles.push_back(*fv_it);
        ++count;
    }
    return count;
}



// spatial hashing
// adapting from sgh1.net/posts/spatial-hashing-1.md
// I hate template programming, so I'm going to make it specific to this implementation
// In each bin I need to store a VertexHandle so I can get access to the vertex data in my mesh
// I also need access to the mesh itself

// keys to the hash (3d indices)
class int3 {
public:
    int i, j, k;
};
static bool operator<(const int3& lhs, const int3& rhs)
{
    if (lhs.i < rhs.i) return true;
    if (lhs.i > rhs.i) return false;

    if (lhs.j < rhs.j) return true;
    if (lhs.j > rhs.j) return false;

    if (lhs.k < rhs.k) return true;
    if (lhs.k > rhs.k) return false;
    return false;
}


//template <typename T>
class SpatialHashTable {
public:
    SpatialHashTable(MyMesh& mesh, float radius, int voxel_res) {
        m_voxel_res = voxel_res;
        m_mesh = &mesh;
        m_radius = radius;
        // get bounding box size
        int nV = m_mesh->n_vertices();
        OpenMesh::Vec3f min, max;
        min[0] = min[1] = min[2] = LARGE_F;
        max[0] = max[1] = max[2] = -LARGE_F;
        for (int i = 0; i < nV; ++i)
        {
            OpenMesh::VertexHandle vh = OpenMesh::VertexHandle(i);
            OpenMesh::Vec3f v = m_mesh->point(vh);
            for (int j = 0; j < 3; j++) min[j] = fminf(min[j], v[j]);
            for (int j = 0; j < 3; j++) max[j] = fmaxf(max[j], v[j]);
        }
        // min/max are bounding box
        // compute bin size along each dimension
        OpenMesh::Vec3f binSize;
        for (int j = 0; j < 3; j++) { binSize[j] = (max[j] - min[j]) / m_voxel_res; }
        m_d = binSize;
        std::cout << "SpatialHashTable::constructor" << std::endl;
        std::cout << "radius= " << m_radius << std::endl;
        std::cout << "Voxel_res= " << m_voxel_res << std::endl;
        std::cout << "Min(" << min[0] << "," << min[1] << "," << min[2] << ")" << std::endl;
        std::cout << "Max(" << max[0] << "," << max[1] << "," << max[2] << ")" << std::endl;
        std::cout << "BinSize(" << m_d[0] << "," << m_d[1] << "," << m_d[2] << ")" << std::endl;

    }

    int3 hash_func(const OpenMesh::Vec3f& v) {
        int3 ret;
        //OpenMesh::Vec3f v = m_mesh->point(vh);
        ret.i = v[0] / m_d[0];
        ret.j = v[1] / m_d[1];
        ret.k = v[2] / m_d[2];

        // not sure about this, need to verify how this works
        if (v[0] < 0.0) { ret.i--; }
        if (v[1] < 0.0) { ret.j--; }
        if (v[2] < 0.0) { ret.k--; }

        return ret;
    }
    int3 hash_func(const OpenMesh::VertexHandle &vh) {
        OpenMesh::Vec3f v = m_mesh->point(vh);
        return hash_func(v);
    }
    void insert(const OpenMesh::VertexHandle& vh) {
        int3 idx = hash_func(vh);
        m_bins[idx].push_back(vh);

       // auto p = m_mesh->point(vh);
       // std::cout << "p(" << p[0] << "," << p[1] << "," << p[2] << ")-->" << "idx:(" << idx.i << "," << idx.j << "," << idx.k << ")" << std::endl;
    }
    std::vector<OpenMesh::VertexHandle> &GetBinHandles(OpenMesh::Vec3f& p) {
        // given a point p
        // find which bin it should map to
        int3 binIDX = hash_func(p);
        // return the bin which is a list of vertex handles
        return m_bins[binIDX];
    }
    std::vector<OpenMesh::VertexHandle>& GetBinHandles(int3 &binIDX) {
        // return the bin which is a list of vertex handles
        return m_bins[binIDX];
    }
    int3 GetBinKey(OpenMesh::Vec3f& p) {
        // given a point p
        // find which bin it should map to
        int3 binIDX = hash_func(p);
        return binIDX;
    }
    int3 GetBinKey(OpenMesh::VertexHandle& vh) {
        // find which bin it should map to
        int3 binIDX = hash_func(vh);
        return binIDX;
    }
    std::vector<int3> GetValidNeighbours(int3& bin, int win) {
        int3 bin2;
        std::vector<int3> neighbours;
        for (int i = -win; i < 0; i++) {
            for (int j = -win; j < 0; j++) {
                for (int k = -win; k < 0; k++) {
                    bin2.i = bin.i+i; 
                    bin2.j = bin.j+j; 
                    bin2.k = bin.k+k;
                    if (m_bins.find(bin2) != m_bins.end()) {
                        // found it 
                        neighbours.push_back(bin2);
                    }
                }
            }
        }
        // excluding the current bin (0,0,0)
        for (int i = 1; i < win; i++) {
            for (int j = 1; j < win; j++) {
                for (int k = 1; k < win; k++) {
                    bin2.i = bin.i + i;
                    bin2.j = bin.j + j;
                    bin2.k = bin.k + k;
                    if (m_bins.find(bin2) != m_bins.end()) {
                        // found it 
                        neighbours.push_back(bin2);
                    }
                }
            }
        }
        std::cout << "found: " << neighbours.size() << " valid neighbours" << std::endl;
        return neighbours;
    }


    std::vector<OpenMesh::VertexHandle> GetClosestPointsWithinRadius(OpenMesh::VertexHandle& pvh, float radius) {
        std::vector<OpenMesh::VertexHandle> validPoints;
        OpenMesh::Vec3f p1 = m_mesh->point(pvh);
         /* */
        
       
        //m_mesh->data(*f_it).camera
        OpenMesh::FaceHandle fh1 = m_mesh->data(pvh).fh;
        int camera1 = m_mesh->data(fh1).camera;

        float r2 = radius * radius;
        /* get bin */
        int3 binkey = GetBinKey(pvh);
        /* now get all vertex handles in this bin */
        std::vector<OpenMesh::VertexHandle> binhandles= GetBinHandles(binkey);
        int numBH = binhandles.size();
        //std::cout << "numBH:" << numBH << std::endl;
        for (int i = 0; i < numBH; i++) {
            OpenMesh::VertexHandle ivh = binhandles[i];
            if (pvh == ivh) {
                continue;
            }
            OpenMesh::FaceHandle fh2 = m_mesh->data(pvh).fh;
            int camera2 = m_mesh->data(fh2).camera;
            /* compute distance between point and this point */
            OpenMesh::Vec3f p2 = m_mesh->point(ivh);
            float d2 = (p1[0] - p2[0]) * (p1[0] - p2[0]) +
                       (p1[1] - p2[1]) * (p1[1] - p2[1]) +
                       (p1[2] - p2[2]) * (p1[2] - p2[2]);
            if (d2 < r2 && camera1 == camera2) {
                /* point is within distance */
                validPoints.push_back(ivh);
               // std::cout << "same camera" << std::endl;
            }
            else {
             //   std::cout << "different camera" << std::endl;
            //    //std::cout << "i="<<i<<" d2= " << d2 << " ,r2 =" << r2 << std::endl;
            }
        }
        return validPoints;
    }

    // members 
    OpenMesh::Vec3f m_d; // size of each bin in the hash
    std::map<int3, std::vector<OpenMesh::VertexHandle>> m_bins; // the bins in the hashtable

    float m_radius;
    int m_voxel_res;// voxel resolution, i.e. 64 = 64x64x64
    MyMesh* m_mesh;
};
