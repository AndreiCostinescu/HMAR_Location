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

		int PredictFromNode();

		int PredictFromEdge();

		int DecideMovement();

		int PredictFromSectorMap();

		double DecideLocSecInt(
			edge_tt edge_,
			point_d &delta_t_,
			int &sec_idx_,
			int &loc_idx_,
			int &loc_last_idx_,
			bool &init_);

		bool DecideGoal(
			int label2_,
			double &sm_i_, //sectormap single
			double delta_t_,
			double loc_error_);

		int DecideWindow(
			vector<double> sm_,
			int loc_idx_,
			int label2_);

		int EvaluatePrediction();

		int RebuildSectorMap(
			vector<vector<point_d> > pva_avg_,
			int	label1_,
			int label2_);

	private:

		int label1_ap;
		bool learn;

		predict_t predict;
		state_t state;

		vector<int> last_loc;
		vector<bool> init;

		vector<point_d> pva_avg;
		vector<predict_t> predict_mem;
		vector<vector<point_d> > pva_avg_mem;

		vector<int> range_in;

};

#endif /* ACTIONPREDICTION_H_ */
