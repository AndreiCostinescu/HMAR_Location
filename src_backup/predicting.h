/*
 * predicting.h
 *
 *  Created on: Mar 19, 2017
 *      Author: chen
 */

#ifndef PREDICTING_H_
#define PREDICTING_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "dbscan.h"
#include "labeling_loc.h"
#include "labeling_sec.h"

int predictFromNode(
	Graph Graph_,
	point_d &point_);

int decideMovement(
	Graph Graph_,
	point_d vel_,
	predict_t &predict_);

int decideCurvature(
	Graph Graph_,
	point_d point_,
	vector<point_d> &curve_mem_,
	predict_t &predict_,
	int num_points_);

double decideLocSecInt(
	Graph &Graph_,
	point_d point_,
	point_d &delta_t_,
	int &sec_idx_,
	int &loc_idx_,
	int &loc_last_idx_,
	int label1_,
	int label2_);

bool decideGoal(
	predict_t &predict_,
	int label2_,
	double &sm_i_, //sectormap single
	double delta_t_,
	double loc_error_);

int decideWindow(
	predict_t &predict_,
	vector<double> sm_,
	int loc_idx_,
	int label2_);

int decideSectorMapChangeRate(
	predict_t &predict_,
	vector<double> sm_,
	vector<double> sm2_,
	int loc_idx_,
	int label2_);

int decideOscillate(
	predict_t &predict_,
	point_d vel_,
	vector<double> loc_mem_,
	vector<double> sec_mem_,
	vector<point_d> tan_tmp_,
	vector<point_d> nor_tmp_,
	int loc_idx_,
	int label2_);

int predictFromSectorMap(
	Graph &Graph_,
	Graph &Graph_update_,
	point_d point_,
	point_d vel_,
	predict_t &predict_,
	vector<int> &loc_last_idxs_,
	int label1_);

int evaluatePrediction(
	Graph &Graph_,
	predict_t &predict_);

int predictFromEdge(
	Graph &Graph_,
	Graph &Graph_update_,
	point_d pos_,
	point_d vel_,
	vector<point_d> &curve_mem_,
	predict_t &predict_,
	vector<int> &last_loc_,
	int label1_);

int predictAction(
	Graph &Graph_,
	Graph &Graph_update_,
	int contact_,
	vector<point_d> &pva_avg_, //MOTION
	vector<point_d> &curve_mem_,
	predict_t &predict_,
	int &label1_,
	vector<int> &last_loc_,
	bool learn_=false);



#endif /* PREDICTING_H_ */
