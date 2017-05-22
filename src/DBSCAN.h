/*******************************************************************************
 * DBSCAN.h
 *
 *  Created on: Apr 20, 2017
 *      Author: Chen, EeHeng
 * 		Detail: Implementation of DBSCAN algorithm. Original code is written in
 * 				C by Gagarine Yaikhom and modified by author.
 * 				(Copyright 2015 Gagarine Yaikhom MIT License).
 *
 ******************************************************************************/

#ifndef DBSCAN_H_
#define DBSCAN_H_

#include <iostream>
#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

#define CLUSTER_LIMIT 0.20
#define CONTACT_TRIGGER_RATIO 0.65

#define UNCLASSIFIED 	-1
#define NOISE 			-2
#define CORE_POINT 		 1
#define NOT_CORE_POINT 	 0
#define SUCCESS 		 0
#define FAILURE 		-3

class DBSCAN
{

private:

struct point_d { double x, y, z, l; };
struct node_t { unsigned int index; DBSCAN::node_t *next; };
struct epsilon_neighbours_t { unsigned int num_members; DBSCAN::node_t *head, *tail; };

point_d AddPoint(
		point_d A,
		point_d B);
point_d MinusPoint(
		point_d A,
		point_d B);
point_d MultiPoint(
		point_d A,
		double  B);
double  l2Norm(
		point_d A);
void vectorToArray(
		vector<point_d> A,
		point_d *B);
void ArrayTovector(
		point_d *A,
		int size,
		vector<point_d> &B);
Vector4d PointToVector4d(
		point_d A);
point_d Vector4dToPoint(
		Vector4d A);

public:

DBSCAN();
virtual ~DBSCAN();

node_t *create_node(
		unsigned int index);
int append_at_end(
		unsigned int index,
		epsilon_neighbours_t *en);
epsilon_neighbours_t *get_epsilon_neighbours(
		unsigned int index,
		point_d *points,
		unsigned int num_points,
		double epsilon);
void destroy_epsilon_neighbours(
			epsilon_neighbours_t *en);
void dbscan(
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts);
int expand(
		unsigned int index,
		unsigned int cluster_id,
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts);
int spread(
		unsigned int index,
		epsilon_neighbours_t *seeds,
		unsigned int cluster_id,
		point_d *points,
		unsigned int num_points,
		double epsilon,
		unsigned int minpts);
double euclidean_dist(
		point_d *a,
		point_d *b);

void DBSCANCluster(
		double epsilon,
		unsigned int minpts,
		unsigned int num_points,
		point_d *p);
void Clustering(
		vector<Vector4d> &points_,
		vector<Vector4d> &centroids_,
		vector<int> &locations_flag_,
		vector<int> contact_flag_,
		double epsilon,
		unsigned int minpts);
void CombineNearCluster(
		vector<point_d> &points_,
		vector<point_d> &locations_,
		vector<int> &locations_flags_,
		vector<int> contact_);

};

#endif /* DBSCAN_H_ */
