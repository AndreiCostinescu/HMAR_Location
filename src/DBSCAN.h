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
#include <memory>
#include <Eigen/Eigen>

// CLUSTER_LIMIT is used to control the proximity in which 2 clusters merge.
#ifdef DATA1
#define CLUSTER_LIMIT 0.20
#elif DATA2
#define CLUSTER_LIMIT 0.10
#else
#define CLUSTER_LIMIT 0.10
#endif

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

	struct point_d
	{
		double x, y, z, l;

		point_d(
				double x_ = 0.0,
				double y_ = 0.0,
				double z_ = 0.0,
				double l_ = 0.0)
				: x(x_),
						y(y_),
						z(z_),
						l(l_)
		{
		}

		point_d operator+(
				const point_d &B)
		{
			return point_d(x + B.x, y + B.y, z + B.z, l);
		}

		point_d operator-(
				const point_d &B)
		{
			return point_d(x - B.x, y - B.y, z - B.z, l);
		}

		point_d operator*(
				const double &B)
		{
			return point_d(x * B, y * B, z * B, l);
		}
	};

	struct node_t
	{
		unsigned int index;
		DBSCAN::node_t *next;
	};

	struct epsilon_neighbours_t
	{
		unsigned int num_members;
		DBSCAN::node_t *head, *tail;
	};

//	virtual point_d AddPoint(
//			point_d A,
//			point_d B);
//	virtual point_d MinusPoint(
//			point_d A,
//			point_d B);
//	virtual point_d MultiPoint(
//			point_d A,
//			double B);

	virtual double l2Norm(
			point_d A);
	virtual void vectorToArray(
			std::vector<point_d> A,
			point_d *B);
	virtual void ArrayTovector(
			point_d *A,
			int size,
			std::vector<point_d> &B);
	virtual Eigen::Vector4d PointToVector4d(
			point_d A);
	virtual point_d Vector4dToPoint(
			Eigen::Vector4d A);

public:

	DBSCAN();
	virtual ~DBSCAN();

	virtual void test()
	{
		point_d A(1.0, 2.0, 3.0, 4.0);
		point_d B(5.0, 6.0, 7.0, 1.0);
		point_d C = A + B;
		std::cout << C.x << C.y << C.z << C.l << std::endl;
	}

	virtual node_t *create_node(
			unsigned int index);
	virtual int append_at_end(
			unsigned int index,
			epsilon_neighbours_t *en);
	virtual epsilon_neighbours_t *get_epsilon_neighbours(
			unsigned int index,
			point_d *points,
			unsigned int num_points,
			double epsilon);
	virtual void destroy_epsilon_neighbours(
			epsilon_neighbours_t *en);
	virtual void dbscan(
			point_d *points,
			unsigned int num_points,
			double epsilon,
			unsigned int minpts);
	virtual int expand(
			unsigned int index,
			unsigned int cluster_id,
			point_d *points,
			unsigned int num_points,
			double epsilon,
			unsigned int minpts);
	virtual int spread(
			unsigned int index,
			epsilon_neighbours_t *seeds,
			unsigned int cluster_id,
			point_d *points,
			unsigned int num_points,
			double epsilon,
			unsigned int minpts);
	virtual double euclidean_dist(
			point_d *a,
			point_d *b);

	virtual void DBSCANCluster(
			double epsilon,
			unsigned int minpts,
			unsigned int num_points,
			point_d *p);
	virtual void Clustering(
			std::shared_ptr<std::vector<Eigen::Vector4d> > points_,
			std::shared_ptr<std::vector<Eigen::Vector4d> > centroids_,
			std::shared_ptr<std::vector<int> > locations_flag_,
			std::shared_ptr<std::vector<int> > contact_flag_,
			const double &epsilon,
			const unsigned int &minpts);
	virtual void CombineNearCluster(
			std::vector<point_d> &points_,
			std::vector<point_d> &locations_,
			std::vector<int> &locations_flag_,
			const std::vector<int> &contact_);

};

#endif /* DBSCAN_H_ */
