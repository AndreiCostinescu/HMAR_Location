/*
 * ActionPrediction.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef ACTIONPREDICTION_H_
#define ACTIONPREDICTION_H_

#include "Evaluate.h"
#include "TrainSM.h"

class ActionPrediction : public Evaluate, public TrainSM
{
	public:
		ActionPrediction(Graph *Graph_, bool learn_);

		virtual ~ActionPrediction();

		void PredictExt(
			vector<point_d> &pva_avg_,
			int contact_);

		void PredictInit();

		int Predict();

		int ContactTrigger();

		int DecideBoundaryClosestExt(
			point_d &point_,
			vector<point_d> centroids_);

		int DecideBoundarySphereExt(
			point_d &point_,
			vector<point_d> centroids_);

		int DecideBoundaryCuboidExt(
			point_d &point_,
			point_d box_min_,
			point_d box_max_);

		int NodePrediction();

		int EdgePrediction();

		int DecideMovement(bool x_);

		int PredictFromSectorMap();

		double DecideLocSecInt(
			point_d &delta_t_,
			int &sec_idx_,
			int &loc_idx_,
			int &loc_last_idx_,
			int &init_);

		bool DecideGoal(
			int label2_,
			double &sm_i_, //sectormap single
			double delta_t_,
			double loc_error_);

		int DecideWindow(
			vector<double> sm_,
			int loc_idx_,
			int label2_);

		int EvaluateNodePrediction();

		int EvaluateEdgePrediction();

		int RebuildSectorMap(
			vector<vector<point_d> > pva_avg_,
			int	label1_,
			int label2_);

	private:

		int label1_ap;
		int label2_ap;
		bool learn;
		bool la_sm_change; // change flag from sm to la

		node_tt node_ap; //from label1

		predict_e pred_sm; // edge_prediction
		predict_n pred_la; // node_prediction

		state_t state; // action state

		vector<int> last_loc;
		vector<int> init;

		vector<point_d> pva_avg;
		vector<predict_e> pred_sm_mem;
		vector<predict_n> pred_la_mem;
		vector<vector<point_d> > pva_avg_mem;

		vector<int> range_in;

};

#endif /* ACTIONPREDICTION_H_ */
