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
			vector<Vector4d> points_avg_,
			vector<Vector4d> &points_est_,
			vector<Vector3d> &coeffs_);

		int DecideSectorIntervalExt(
			edge_tt	edge_,
			Vector4d point_,
			Vector3d &delta_t_,
			int &sec_idx_,
			int loc_idx_);

		double DecideLocationIntervalExt(
			edge_tt	edge_,
			Vector4d point_,
			int &loc_idx_,
			int loc_last_idx_,
			int loc_offset_,
			bool loc_init_);

		int AdjustSectorMap(
			edge_tt &edge_,
			Vector4d &point_,
			int &loc_last_idx_,
			int &loc_curr_idx_,
			double &delta_t_mem_,
			bool &loc_init_,
			int loc_offset_);

		int AdjustCurve(
			edge_tt &edge_,
			Vector4d point_,
			int &loc_last_idx_,
			bool &loc_init_,
			int loc_offset_);

		int AdjustCurveExt(
			Graph *Graph_,
			vector<Vector3d> coeffs_, // from fit curve
			vector<Vector4d> pts_);

		int FitSectorMap(
			edge_tt &edge_,
			Vector4d point_,
			int &loc_last_idx_,
			int loc_offset_,
			bool &loc_init_);

		int FitSectorMapInit(
			Graph *Graph_,
			vector<Vector4d> &points_,
			int loc_offset_);

		int FitSectorMapExt(
			Graph *Graph_,
			vector<Vector4d> &points_,
			int loc_offset_);

		int FindWindowConstraint(
			Graph *Graph_){return EXIT_SUCCESS;}

		int UpdateSectorMap(
			Graph *Graph_,
			vector<Vector4d> points_avg_);

		int BuildSectorMap(
			Graph *Graph_,
			kb_t kb_,
			vector<vector<Vector4d> > pva_avg_,
			vector<int> contact_);

		void SetLabel1SM(int label1_) {label1_sm = label1_;}
		void SetLabel2SM(int label2_) {label2_sm = label2_;}

	private:
		// label1_sm	 : start location
		// label2_sm	 : goal location
		// label_idx_sm : index of data point for label1_sm
		int label1_sm, label2_sm, label_idx_sm;
		vector<Vector4d> pos_sm, pos_ind_sm, vel_sm;
};

#endif /* TRAINSM_H_ */
