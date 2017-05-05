/*
 * DBSCAN.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef DBSCAN_H_
#define DBSCAN_H_

#include "dataDeclaration.h"
#include "algo.h"

// wrappper for dbscan algorithm
class DBSCAN
{
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

// ============================================================================
// [ADD-ONS]
// ============================================================================
		void DBSCANCluster(
			double 			epsilon,
			unsigned int 	minpts,
			unsigned int 	num_points,
			point_d 		*p);
		void Clustering(
			vector<point_d> &points_,
			double 			epsilon,
			unsigned int 	minpts);
		void CombineNearCluster(
			vector<point_d> &points_,
			vector<point_d> &locations_,
			vector<int> 	&locations_flags_,
			vector<int> 	contact_);
};

#endif /* DBSCAN_H_ */
