/*
 * TrainSM.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef TRAINSM_H_
#define TRAINSM_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "vtkExtra.h"
#include "misc.h"
#include "core.h"

class TrainSM
{
	public:
		TrainSM();
		virtual ~TrainSM();

		void ClearSM();

		int FitCurve(
			vector<point_d> points_avg_,
			vector<point_d> &points_est_,
			vector<point_d> &coeffs_);

		int DecideSectorIntervalExt(
			edge_tt	edge_,
			point_d point_,
			point_d &delta_t_,
			int &sec_idx_,
			int loc_idx_);

		double DecideLocationIntervalExt(
			edge_tt	edge_,
			point_d point_,
			int &loc_idx_,
			int loc_last_idx_,
			int loc_offset_,
			bool loc_init_);

		int AdjustSectorMap(
			edge_tt &edge_,
			point_d &point_,
			int &loc_last_idx_,
			int &loc_curr_idx_,
			double &delta_t_mem_,
			bool &loc_init_,
			int loc_offset_);

		int AdjustCurve(
			edge_tt &edge_,
			point_d &point_,
			int &loc_last_idx_,
			bool &loc_init_,
			int loc_offset_);

		int AdjustCurveExt(
			Graph *Graph_,
			vector<point_d> coeffs_,
			vector<point_d> pts_);

		int FitSectorMap(
			edge_tt &edge_,
			point_d point_,
			int &loc_last_idx_,
			int loc_offset_,
			bool &loc_init_);

		int FitSectorMapInit(
			Graph *Graph_,
			vector<point_d> &points_,
			int loc_offset_);

		int FitSectorMapExt(
			Graph *Graph_,
			vector<point_d> &points_,
			int loc_offset_);

		int FindWindowConstraint(
			Graph *Graph_){return EXIT_SUCCESS;}

		int UpdateSectorMap(
			Graph *Graph_,
			vector<point_d> points_avg_);

		int BuildSectorMap(
			Graph *Graph_,
			vector<vector<point_d> > pva_avg_,
			vector<int> contact_);

		void SetLabel1SM(int label1_) {label1_sm = label1_;}
		void SetLabel2SM(int label2_) {label2_sm = label2_;}

	private:
		// label1_sm	 : start location
		// label2_sm	 : goal location
		// label_idx_sm : index of data point for label1_sm
		int label1_sm, label2_sm, label_idx_sm;
		vector<point_d> pos_sm, pos_ind_sm, vel_sm;
};

#endif /* TRAINSM_H_ */
