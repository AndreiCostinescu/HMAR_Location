/*
 * ActionPrediction.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#ifndef ACTIONPREDICTION_H_
#define ACTIONPREDICTION_H_

#include "vtkExtra.h"
//#include "Prediction.h"
#include "DataContainer.h"
#include "ReadFile.h"
#include "WriteFile.h"
#include "Evaluate.h"

class ActionPrediction :public ReadFile,
						public WriteFile,
						public DataContainer,
						public Evaluate
{
	public:
		ActionPrediction();

		virtual ~ActionPrediction();

		void PredictExt();

		void PredictInit(
				bool learn_);

		int Predict();

		int ContactTrigger();

		int DecideBoundaryClosestExt();

		int DecideBoundarySphereExt();

		int DecideBoundaryCuboidExt(
			Vector4d &point_,
			Vector3d box_min_,
			Vector3d box_max_);

		int NodePrediction();

		int EdgePrediction();

		int DecideMovement(bool x_);

		int PredictFromSectorMap();

		double DecideLocSecInt(
			Vector3d &delta_t_,
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

//		int RebuildSectorMap(
//			vector<vector<point_d> > pva_avg_,
//			int	label1_,
//			int label2_);

	private:

		int label1_ap;
		int label2_ap;

		bool learn;
		bool la_sm_change; // change flag from sm to la

		node_tt node_ap; //from label1

		predict_e pred_sm; // edge_prediction
		predict_n pred_la; // node_prediction

		vector<int> last_loc;
		vector<int> init;

		vector<predict_e> pred_sm_mem;
		vector<predict_n> pred_la_mem;

		vector<vector<Vector4d> > pva_avg_mem; // rebuild SM

		vector<int> range_in;

};

#endif /* ACTIONPREDICTION_H_ */
