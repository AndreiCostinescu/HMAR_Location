/*
 * TrainSM.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINSM_H_
#define TRAINSM_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>
#include "core.h"
#include "CGraph.h"
#include "CKB.h"
#include "VTKExtra.h"
#include "print.h"

// number of fit coefficients
// nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 (for cubic b-spline)
#define NCOEFFS	15
#define NBREAK 	(NCOEFFS - 2)
#define DEGREE 10 //k+1

class TrainSM
{
public:
	TrainSM();
	virtual ~TrainSM();

	void ClearSM();
	int InitSM(
			int loc_int_,
			int sec_int_,
			int f_win_);

	int FitCurve(
			vector<Vector4d> &points_est_,
			vector<Vector3d> &coeffs_);
	int DecideSectorIntervalExt(
			CGraph::edge_t	*edge_,
			Vector4d point_,
			Vector3d &delta_t_,
			int &sec_idx_,
			int loc_idx_);
	double DecideLocationIntervalExt(
			CGraph::edge_t	*edge_,
			Vector4d point_,
			int &loc_idx_,
			int loc_last_idx_,
			int loc_offset_,
			bool loc_init_);
	int AdjustSectorMap(
			CGraph::edge_t *edge_,
			Vector4d point_,
			int &loc_last_idx_,
			int &loc_curr_idx_,
			double &delta_t_mem_,
			bool &loc_init_,
			int loc_offset_);
	int AdjustCurve(
			CGraph::edge_t *edge_,
			Vector4d point_,
			int &loc_last_idx_,
			bool &loc_init_,
			int loc_offset_);
	int AdjustCurveExt(
			CGraph *Graph_,
			vector<Vector3d> coeffs_); // from fit curve
	int FitSectorMap(
			CGraph::edge_t *edge_,
			Vector4d point_,
			int &loc_last_idx_,
			int loc_offset_,
			bool &loc_init_);
	int FitSectorMapInit(
			CGraph *Graph_,
			vector<Vector4d> &points_,
			int loc_offset_);
	int FitSectorMapExt(
			CGraph *Graph_,
			int loc_offset_);
	int FindWindowConstraint(
			CGraph *Graph_){return EXIT_SUCCESS;}
	int UpdateSectorMap(
			CGraph *Graph_);
	int BuildSectorMap(
			CGraph *Graph_,
			CKB *kb_,
			vector<vector<Vector4d> > *pva_avg_,
			vector<int> *contact_);

	void SetLabel1SM(int label1_) {label1_sm = label1_;}
	void SetLabel2SM(int label2_) {label2_sm = label2_;}

private:
	// label1_sm	 : start location
	// label2_sm	 : goal location
	// label_idx_sm : index of data point for label1_sm
	int label1_sm, label2_sm, label_idx_sm;
	vector<Vector4d> pos_sm, pos_ind_sm, vel_sm;
	VTKExtra *VTK;

protected:
	int loc_int;
	int sec_int;
	int f_win;

};

#endif /* TRAINSM_H_ */
