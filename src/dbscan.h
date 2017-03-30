/*
 * dbscan.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef DBSCAN_H_
#define DBSCAN_H_

#include "dataDeclaration.h"
#include "algo.h"

node_t *create_node(
	unsigned int index);
int append_at_end(
	unsigned int index,
	epsilon_neighbours_t *en);
epsilon_neighbours_t *get_epsilon_neighbours(
	unsigned int index,
	point_d *points,
	unsigned int num_points,
	double epsilon,
	double (*dist)(point_d *a, point_d *b));
void destroy_epsilon_neighbours(
		epsilon_neighbours_t *en);
void dbscan(
	point_d *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_d *a, point_d *b));
int expand(
	unsigned int index,
	unsigned int cluster_id,
	point_d *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_d *a, point_d *b));
int spread(
	unsigned int index,
	epsilon_neighbours_t *seeds,
	unsigned int cluster_id,
	point_d *points,
	unsigned int num_points,
	double epsilon,
	unsigned int minpts,
	double (*dist)(point_d *a, point_d *b));
double euclidean_dist(
	point_d *a,
	point_d *b);

// ============================================================================
// ADD-ONS
// ============================================================================
void dbscanCluster(
	double 			epsilon,
	unsigned int 	minpts,
	unsigned int 	num_points,
	point_d 		*p);
void combineNearCluster(
	vector<point_d> &points_,
	vector<point_d> &locations_,
	vector<int> 	contact_);
int clustering(
	vector<point_d> &points_,
	double 			epsilon,
	unsigned int 	minpts);


#endif /* DBSCAN_H_ */
