#pragma once
#include <iostream>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "polygonizedata.h"

typedef struct {
	Eigen::Vector3d p[3];
	Eigen::Vector3d c;
} TRIANGLE;

typedef struct {
	Eigen::Vector3d p[8];
	double val[8];
} GRIDCELL;

enum {
	VOXEL_EMPTY=0,
	VOXEL_FULL=1,
	VOXEL_UNSEEN=2
};
static float VOXEL_MAXDIST = 0.5f;

/* a very very naive and simple version of a TSDF Voxel grid
- currently just a dense grid to get basic silhouette carving and sdf estimation working
*/
#define IND2LINEAR(i,j,k, w,h,d) (((k)*(w)*(h)) + ((j)*(w)) + (i))

class Voxel {
public:
	/* store sdf and weight/flag */
	int flag;
	float sdf;
	float weight;
	Eigen::Vector3d c;
	float r, g, b;
};

class TSDFVolume
{
public:
	/* constructor
	- takes in values of resolution (x,y,z), and world size (x,y,z)
	*/

	TSDFVolume(int resX, int resY, int resZ, Eigen::Vector3d &_center, Eigen::Vector3d &_sz) {
		res[0] = resX;
		res[1] = resY;
		res[2] = resZ;
		center = _center;
		sz = _sz;
		grid = NULL;
		vSize[0] = sz[0] / (float)res[0];
		vSize[1] = sz[1] / (float)res[1];
		vSize[2] = sz[2] / (float)res[2];

		this->AllocateDense();
	}

	~TSDFVolume() {}

	void makeSphereSDF(float radius) {
		for (int k = 0; k < res[2]; k++) {
			for (int j = 0; j < res[1]; j++) {
				for (int i = 0; i < res[0]; i++) {
					int ind = IND2LINEAR(i, j, k, res[0], res[1], res[2]);
					// compute distance from center of voxel to surface of the sphere 
					grid[ind].sdf = (center - grid[ind].c).norm() - radius;
				}
			}
		}
	}

	bool AllocateDense() {
		if (grid) return false;
		grid = new Voxel[res[0]*res[1]*res[2]];
		if (!grid)
		{
			std::cout << "VOXEL: allocateDense() not enough memory " << std::endl;
			return false;
		}
		return true;
	}
	bool AllocateDense2() {
		if (grid2) return false;
		grid2 = new Voxel[res[0] * res[1] * res[2]];
		if (!grid2)
		{
			std::cout << "VOXEL: allocateDense2() not enough memory " << std::endl;
			return false;
		}
		return true;
	}
	void SetAllVoxels(int flag, float sdf, float weight) {
		int numVoxels = res[0] * res[1] * res[2];
		for (int i = 0; i < numVoxels; i++) {
			grid[i].flag = flag;
			grid[i].sdf = sdf;
			grid[i].weight = weight;
		}
	}
	void ComputeAllVoxelCenters() {
		for (int k = 0; k < res[2]; k++) {
			for (int j = 0; j < res[1]; j++) {
				for (int i = 0; i < res[0]; i++) {
					Eigen::Vector3d c;
					GetVoxelCoordsFromIndex(i, j, k, c);
					get(i, j, k).c = c;
				}
			}
		}
	}
	void Smooth(int wSize) {
		this->AllocateDense2();
		// BUG: CAN't do it in place like this.... 
		for (int k = wSize; k < res[2]-wSize; k++) {
			for (int j = wSize; j < res[1]-wSize; j++) {
				for (int i = wSize; i < res[0]-wSize; i++) {
					int ind = IND2LINEAR(i, j, k, res[0], res[1], res[2]);
					// compute distance from center of voxel to surface of the sphere 
					if (grid[ind].flag != VOXEL_EMPTY) {
						float sum = 0;
						int num = 0;
						for (int wi = -wSize; wi < wSize; wi++) {
							for (int wj = -wSize; wj < wSize; wj++) {
								for (int wk = -wSize; wk < wSize; wk++) {
									int ind2 = IND2LINEAR(i + wi, j+wj, k+wk, res[0], res[1], res[2]);
									//if (grid[ind2].flag != VOXEL_EMPTY) 
									{
										sum += grid[ind2].sdf;
										num++;
									}
								}
							}
							
						}
						float v = sum / num;
						grid2[ind].sdf = v;
					}
				}
			}
		}

		for (int k = wSize; k < res[2] - wSize; k++) {
			for (int j = wSize; j < res[1] - wSize; j++) {
				for (int i = wSize; i < res[0] - wSize; i++) {
					int ind = IND2LINEAR(i, j, k, res[0], res[1], res[2]);
					if (grid[ind].flag != VOXEL_EMPTY) {
						grid[ind].sdf = grid[ind].sdf*0.2 + 0.8*(grid2[ind].sdf);
					}
				}
			}
		}


	}
	void GetVoxelCoordsFromIndex(int i, int j, int k, Eigen::Vector3d& vCenter) {
	//	int ind = IND2LINEAR(i, j, k, res[0], res[1], res[2]);


		vCenter[0] = ((((float)(i) / (float)res[0])-0.5f) * sz[0]) + vSize[0]/2 + center[0];
		vCenter[1] = ((((float)(j) / (float)res[1])-0.5f) * sz[1]) + vSize[1]/2 + center[1];
		vCenter[2] = ((((float)(k) / (float)res[2])-0.5f) * sz[2]) + vSize[2]/2 + center[2];
		//std::cout << "("<<vCenter[0]<<","<<vCenter[1]<<","<<vCenter[2]<<")" << std::endl;
	}
	void GetVoxelCornersFromIndex(int i, int j, int k, std::vector<Eigen::Vector3d> &corners) {
		// get center 
		Eigen::Vector3d c;
		GetVoxelCoordsFromIndex(i, j, k, c);
		float sX = this->vSize[0]/2.f;
		float sY = this->vSize[1]/2.f;
		float sZ = this->vSize[2]/2.f;

		// the center
		corners.push_back(Eigen::Vector3d(c[0], c[1], c[2]));
		corners.push_back(Eigen::Vector3d(c[0]-sX, c[1]-sY, c[2]-sZ));
		corners.push_back(Eigen::Vector3d(c[0]+sX, c[1]-sY, c[2]-sZ));
		corners.push_back(Eigen::Vector3d(c[0]+sX, c[1]-sY, c[2]+sZ));
		corners.push_back(Eigen::Vector3d(c[0]-sX, c[1]-sY, c[2]+sZ));
		corners.push_back(Eigen::Vector3d(c[0] - sX, c[1] + sY, c[2] - sZ));
		corners.push_back(Eigen::Vector3d(c[0] + sX, c[1] + sY, c[2] - sZ));
		corners.push_back(Eigen::Vector3d(c[0] + sX, c[1] + sY, c[2] + sZ));
		corners.push_back(Eigen::Vector3d(c[0] - sX, c[1] + sY, c[2] + sZ));


	}
	Voxel& get(int i, int j, int k) {
		int ind = IND2LINEAR(i, j, k, res[0], res[1], res[2]);
		return this->grid[ind];
	}

	void set(int i, int j, int k, int flag) {
		Voxel &v=get(i, j, k);
		v.flag = flag;
	}


	/*
	   Linearly interpolate the position where an isosurface cuts
	   an edge between two vertices, each with their own scalar value
	*/
	Eigen::Vector3d VertexInterp2(double isolevel, Eigen::Vector3d& p1, Eigen::Vector3d& p2, float valp1, float valp2)
	{
		double mu;
		Eigen::Vector3d p;

		//if (fabs(isolevel - valp1) < isolevel)
		//	return(p1);
		//if (fabs(isolevel - valp2) < isolevel)
		//	return(p2);
		//if (fabs(valp1 - valp2) < isolevel)
		//	return(p1);
		mu = (-valp1) / (valp2 - valp1);
		p(0) = p1(0) + mu * (p2(0) - p1(0));
		p(1) = p1(1) + mu * (p2(1) - p1(1));
		p(2) = p1(2) + mu * (p2(2) - p1(2));

		//p = p1 + (-valp1 / (valp2 - valp1)) * (p2 - p1);

		return(p);
	}

	/*
	   Linearly interpolate the position where an isosurface cuts
	   an edge between two vertices, each with their own scalar value
	*/
	Eigen::Vector3d VertexInterp(Eigen::Vector3d &p1, Eigen::Vector3d & p2, float valp1, float valp2)
	{
		double mu;
		Eigen::Vector3d p;

		//if (fabs(isolevel - valp1) < 0.00001)
		//	return(p1);
		//if (fabs(isolevel - valp2) < 0.00001)
		//	return(p2);
		//if (fabs(valp1 - valp2) < 0.00001)
		//	return(p1);
		mu = (- valp1) / (valp2 - valp1);
		p(0) = p1(0) + mu * (p2(0) - p1(0));
		p(1) = p1(1) + mu * (p2(1) - p1(1));
		p(2) = p1(2) + mu * (p2(2) - p1(2));

		//p = p1 + (-valp1 / (valp2 - valp1)) * (p2 - p1);

		return(p);
	}
	void PolygoniseTri(int ii0, int ii1, int ii2, int ii3, float isolevel, std::vector<TRIANGLE> &triangles) {
		/* sample the grid here */
		float val0 = grid[ii0].sdf;
		float val1 = grid[ii1].sdf;
		float val2 = grid[ii2].sdf;
		float val3 = grid[ii3].sdf;
		
		

		// the centers of each voxel
		Eigen::Vector3d p0 = grid[ii0].c;
		Eigen::Vector3d p1 = grid[ii1].c;
		Eigen::Vector3d p2 = grid[ii2].c;
		Eigen::Vector3d p3 = grid[ii3].c;


		TRIANGLE t0, t1;
		Eigen::Vector3d v0, v1, v2;
		Eigen::Vector3d vv0, vv1, vv2;

		int cubeIndex = 0;
		if (val0 < isolevel) cubeIndex |= 1;
		if (val1 < isolevel) cubeIndex |= 2;
		if (val2 < isolevel) cubeIndex |= 4;
		if (val3 < isolevel) cubeIndex |= 8;

		// interpolate the vertices and append the triangle here
		switch (cubeIndex) {
		case 0:
		case 15:
			break;
		case 14:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p0, p2, val0, val2);
			v2 = VertexInterp(p0, p3, val0, val3);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;

			triangles.push_back(t0);
			
			break;
		case 1:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p0, p2, val0, val2);
			v2 = VertexInterp(p0, p3, val0, val3);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;

			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;

			triangles.push_back(t0);
			break;

		case 13:
			v0 = VertexInterp(p1, p0, val1, val0);
			v1 = VertexInterp(p1, p3, val1, val3);
			v2 = VertexInterp(p1, p2, val1, val2);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			break;

		case 2:
			v0 = VertexInterp(p1, p0, val1, val0);
			v1 = VertexInterp(p1, p3, val1, val3);
			v2 = VertexInterp(p1, p2, val1, val2);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			break;
		case 3:
			v0 = VertexInterp(p0, p3, val0, val3);
			v1 = VertexInterp(p0, p2, val0, val2);
			v2 = VertexInterp(p1, p3, val1, val3);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);

			vv0 = VertexInterp(p1, p3, val1, val3);
			vv1 = VertexInterp(p1, p2, val1, val2);
			vv2 = VertexInterp(p0, p2, val0, val2);

			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;

			t1.p[0] = vv2;
			t1.p[1] = vv1;
			t1.p[2] = vv0;
			triangles.push_back(t1);
			break;
		case 12:

			v0 = VertexInterp(p0, p3, val0, val3);
			v1 = VertexInterp(p0, p2, val0, val2);
			v2 = VertexInterp(p1, p3, val1, val3);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;

			triangles.push_back(t0);
			
			vv0 = VertexInterp(p1, p3, val1, val3);
			vv1 = VertexInterp(p1, p2, val1, val2);
			vv2 = VertexInterp(p0, p2, val0, val2);


			t1.p[0] = vv0;
			t1.p[1] = vv1;
			t1.p[2] = vv2;
			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;

			triangles.push_back(t1);

			break;
			
		case 11:
			v0 = VertexInterp(p2, p0, val2, val0);
			v1 = VertexInterp(p2, p1, val2, val1);
			v2 = VertexInterp(p2, p3, val2, val3);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;

			triangles.push_back(t0);
			break;
		case 4:
			v0 = VertexInterp(p2, p0, val2, val0);
			v1 = VertexInterp(p2, p1, val2, val1);
			v2 = VertexInterp(p2, p3, val2, val3);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;

			triangles.push_back(t0);
			break;


		case 10:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p2, p3, val2, val3);
			v2 = VertexInterp(p0, p3, val0, val3);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			vv0 = v0;// VertexInterp(p0, p1, val0, val1);
			vv1 = VertexInterp(p1, p2, val1, val2);
			vv2 = v1;// VertexInterp(p2, p3, val2, val3);

			t1.p[0] = vv0;
			t1.p[1] = vv1;
			t1.p[2] = vv2;
			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;
			triangles.push_back(t1);


			break;
		case 5:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p2, p3, val2, val3);
			v2 = VertexInterp(p0, p3, val0, val3);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			vv0 = v0;// VertexInterp(p0, p1, val0, val1);
			vv1 = VertexInterp(p1, p2, val1, val2);
			vv2 = v1;// VertexInterp(p2, p3, val2, val3);

			t1.p[0] = vv2;
			t1.p[1] = vv1;
			t1.p[2] = vv0;
			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;
			triangles.push_back(t1);


			break;
			
		case 9:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p1, p3, val1, val3);
			v2 = VertexInterp(p2, p3, val2, val3);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			vv0 = v0;// VertexInterp(p0, p1, val0, val1);
			vv1 = VertexInterp(p0, p2, val0, val2);
			vv2 = v2;// VertexInterp(p2, p3, val2, val3);

			t1.p[0] = vv0;
			t1.p[1] = vv1;
			t1.p[2] = vv2;
			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;
			triangles.push_back(t1);

			break;
		case 6:
			v0 = VertexInterp(p0, p1, val0, val1);
			v1 = VertexInterp(p1, p3, val1, val3);
			v2 = VertexInterp(p2, p3, val2, val3);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);


			vv0 = v0;// VertexInterp(p0, p1, val0, val1);
			vv1 = VertexInterp(p0, p2, val0, val2);
			vv2 = v2;// VertexInterp(p2, p3, val2, val3);

			t1.p[0] = vv2;
			t1.p[1] = vv1;
			t1.p[2] = vv0;
			t1.c[0] = 1;
			t1.c[1] = 1;
			t1.c[2] = 1;
			triangles.push_back(t1);

			break;
			
		case 7:
			v0 = VertexInterp(p3, p0, val3, val0);
			v1 = VertexInterp(p3, p2, val3, val2);
			v2 = VertexInterp(p3, p1, val3, val1);

			t0.p[0] = v2;
			t0.p[1] = v1;
			t0.p[2] = v0;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);
			break;
		case 8:
			v0 = VertexInterp(p3, p0, val3, val0);
			v1 = VertexInterp(p3, p2, val3, val2);
			v2 = VertexInterp(p3, p1, val3, val1);

			t0.p[0] = v0;
			t0.p[1] = v1;
			t0.p[2] = v2;
			t0.c[0] = 1;
			t0.c[1] = 1;
			t0.c[2] = 1;
			triangles.push_back(t0);
			break;
		}
	}

	int Polygonise(float isolevel, std::vector<TRIANGLE> &triangles) {
		int numTris = 0;
		for (int i =0; i < res[0]-1;i++) {
			for (int j = 0; j < res[1]-1; j++) {
				for (int k = 0; k < res[2]-1; k++) {
					// polygonize this cell
					// probably want to check the flag to see if we've carved it or not
					int indices[24]; // 0 2 3 7
					indices[0] = IND2LINEAR(i+0, j+0, k+0, res[0], res[1], res[2]);
					indices[1] = IND2LINEAR(i+1, j+0, k+1, res[0], res[1], res[2]);
					indices[2] = IND2LINEAR(i+0, j+0, k+1, res[0], res[1], res[2]);
					indices[3] = IND2LINEAR(i+0, j+1, k+1, res[0], res[1], res[2]);

					// 0 2 6 7
					indices[4] = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
					indices[5] = IND2LINEAR(i + 1, j + 0, k + 1, res[0], res[1], res[2]);
					indices[6] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
					indices[7] = IND2LINEAR(i + 0, j + 1, k + 1, res[0], res[1], res[2]);

					// 0 4 6 7
					indices[8]  = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
					indices[9]  = IND2LINEAR(i + 0, j + 1, k + 0, res[0], res[1], res[2]);
					indices[10] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
					indices[11] = IND2LINEAR(i + 0, j + 1, k + 1, res[0], res[1], res[2]);

					// 0 6 1 2
					indices[12] = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
					indices[13] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
					indices[14] = IND2LINEAR(i + 1, j + 0, k + 0, res[0], res[1], res[2]);
					indices[15] = IND2LINEAR(i + 1, j + 0, k + 1, res[0], res[1], res[2]);

					// 0 6 1 4
					indices[16] = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
					indices[17] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
					indices[18] = IND2LINEAR(i + 1, j + 0, k + 0, res[0], res[1], res[2]);
					indices[19] = IND2LINEAR(i + 0, j + 1, k + 0, res[0], res[1], res[2]);

					// 5 6 1 4
					indices[20] = IND2LINEAR(i + 1, j + 1, k + 0, res[0], res[1], res[2]);
					indices[21] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
					indices[22] = IND2LINEAR(i + 1, j + 0, k + 0, res[0], res[1], res[2]);
					indices[23] = IND2LINEAR(i + 0, j + 1, k + 0, res[0], res[1], res[2]);


					PolygoniseTri(indices[0], indices[1], indices[2], indices[3], isolevel, triangles);
					PolygoniseTri(indices[4], indices[5], indices[6], indices[7], isolevel, triangles);
					PolygoniseTri(indices[8], indices[9], indices[10], indices[11], isolevel, triangles);
					PolygoniseTri(indices[12], indices[13], indices[14], indices[15], isolevel, triangles);
					PolygoniseTri(indices[16], indices[17], indices[18], indices[19], isolevel, triangles);
					PolygoniseTri(indices[20], indices[21], indices[22], indices[23], isolevel, triangles);
				}
			}
		}
		numTris = triangles.size();
		std::cout << "MC: NumTris:" << numTris << std::endl;
		return numTris;
	}

	int PolygoniseMC(float isolevel, std::vector<TRIANGLE>& triangles) {
		int numTris = 0;
		for (int i = 0; i < res[0] - 1; i++) {
			for (int j = 0; j < res[1] - 1; j++) {
				for (int k = 0; k < res[2] - 1; k++) {
					int ind = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
					PolygoniseCellMC(isolevel, triangles, i,j,k);
				}
			}
		}
		numTris = triangles.size();
		std::cout << "MC: NumTris:" << numTris << std::endl;
		return numTris;
	}
	int PolygoniseCellMC(float isolevel, std::vector<TRIANGLE> &triangles, int xi, int yi, int zi) {
		int ntriang;
		int cubeindex;
		Eigen::Vector3d vertlist[12];
		int indices[24];
		int i, j, k;
		i = xi;
		j = yi;
		k = zi;

		indices[0] = IND2LINEAR(i + 0, j + 0, k + 0, res[0], res[1], res[2]);
		indices[1] = IND2LINEAR(i + 1, j + 0, k + 0, res[0], res[1], res[2]);
		indices[2] = IND2LINEAR(i + 1, j + 0, k + 1, res[0], res[1], res[2]);
		indices[3] = IND2LINEAR(i + 0, j + 0, k + 1, res[0], res[1], res[2]);
		indices[4] = IND2LINEAR(i + 0, j + 1, k + 0, res[0], res[1], res[2]);
		indices[5] = IND2LINEAR(i + 1, j + 1, k + 0, res[0], res[1], res[2]);
		indices[6] = IND2LINEAR(i + 1, j + 1, k + 1, res[0], res[1], res[2]);
		indices[7] = IND2LINEAR(i + 0, j + 1, k + 1, res[0], res[1], res[2]);
		/*
	  Determine the index into the edge table which
	  tells us which vertices are inside of the surface
   */
		cubeindex = 0;
		/* get the neighbours */
		// we are in a particular voxel, so our neighbours are
		Eigen::Vector3d p[8], rgb;
		float v[8];
		v[0] = grid[indices[0]].sdf;
		v[1] = grid[indices[1]].sdf;
		v[2] = grid[indices[2]].sdf;
		v[3] = grid[indices[3]].sdf;
		v[4] = grid[indices[4]].sdf;
		v[5] = grid[indices[5]].sdf;
		v[6] = grid[indices[6]].sdf;
		v[7] = grid[indices[7]].sdf;

		//std::cout << "v[0]=" << v[0] << std::endl;

		p[0] = grid[indices[0]].c;
		p[1] = grid[indices[1]].c;
		p[2] = grid[indices[2]].c;
		p[3] = grid[indices[3]].c;
		p[4] = grid[indices[4]].c;
		p[5] = grid[indices[5]].c;
		p[6] = grid[indices[6]].c;
		p[7] = grid[indices[7]].c;
		rgb[0] = grid[indices[0]].r;
		rgb[1] = grid[indices[0]].g;
		rgb[2] = grid[indices[0]].b;
		//for (int i = 0; i < 8; i++) {
		//	int f = grid[indices[i]].flag;
		//	//if ( f != VOXEL_FULL) return 0;
		//	//if (fabs(v[i]) > 0.04) return 0;
		//}

		if ((v[0]) < isolevel) cubeindex |= 1;
		if ((v[1]) < isolevel) cubeindex |= 2;
		if ((v[2]) < isolevel) cubeindex |= 4;
		if ((v[3]) < isolevel) cubeindex |= 8;
		if ((v[4]) < isolevel) cubeindex |= 16;
		if ((v[5]) < isolevel) cubeindex |= 32;
		if ((v[6]) < isolevel) cubeindex |= 64;
		if ((v[7]) < isolevel) cubeindex |= 128;
		/* Cube is entirely in/out of the surface */
		if (edgeTable[cubeindex] == 0)
			return(0);
		/* Find the vertices where the surface intersects the cube */
		if (edgeTable[cubeindex] & 1)
			vertlist[0] = VertexInterp2(isolevel, p[0], p[1], v[0], v[1]);
		if (edgeTable[cubeindex] & 2)
			vertlist[1] = VertexInterp2(isolevel, p[1], p[2], v[1], v[2]);
		if (edgeTable[cubeindex] & 4)
			vertlist[2] = VertexInterp2(isolevel, p[2], p[3], v[2], v[3]);
		if (edgeTable[cubeindex] & 8)
			vertlist[3] = VertexInterp2(isolevel, p[3], p[0], v[3], v[0]);
		if (edgeTable[cubeindex] & 16)
			vertlist[4] = VertexInterp2(isolevel, p[4], p[5], v[4], v[5]);
		if (edgeTable[cubeindex] & 32)
			vertlist[5] = VertexInterp2(isolevel, p[5], p[6], v[5], v[6]);
		if (edgeTable[cubeindex] & 64)
			vertlist[6] = VertexInterp2(isolevel, p[6], p[7], v[6], v[7]);
		if (edgeTable[cubeindex] & 128)
			vertlist[7] = VertexInterp2(isolevel, p[7], p[4], v[7], v[4]);
		if (edgeTable[cubeindex] & 256)
			vertlist[8] = VertexInterp2(isolevel, p[0], p[4], v[0], v[4]);
		if (edgeTable[cubeindex] & 512)
			vertlist[9] = VertexInterp2(isolevel, p[1], p[5], v[1], v[5]);
		if (edgeTable[cubeindex] & 1024)
			vertlist[10] = VertexInterp2(isolevel, p[2], p[6],v[2], v[6]);
		if (edgeTable[cubeindex] & 2048)
			vertlist[11] = VertexInterp2(isolevel, p[3], p[7], v[3], v[7]);
		/* Create the triangle */
		ntriang = 0;
		for (i = 0; triTable[cubeindex][i] != -1; i += 3) {
			TRIANGLE t;
			t.p[0] = vertlist[triTable[cubeindex][i]];
			t.p[1] = vertlist[triTable[cubeindex][i + 1]];
			t.p[2] = vertlist[triTable[cubeindex][i + 2]];
			t.c[0] = rgb[0] / 255.f;
			t.c[1] = rgb[1]/255.f;
			t.c[2] = rgb[2]/255.f;

			triangles.push_back(t);
			ntriang++;
		}

		return(ntriang);
	}


	/// <summary>
	/// ////members
	/// </summary>
	int res[3];
	Voxel *grid; // linear-grid of voxels
	Voxel* grid2; // temp grid
	Eigen::Vector3d center;
	Eigen::Vector3d sz; // grid size
	Eigen::Vector3d vSize; // size of one voxel in the grid

};

