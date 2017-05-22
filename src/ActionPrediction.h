/*
 * ActionPrediction.h
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 *      Detail: SM prediction and LA contact check.
 */

#ifndef ACTIONPREDICTION_H_
#define ACTIONPREDICTION_H_

#include "core.h"
#include "CData.h"
#include "Evaluate.h"

#define RANGE_EXCEED	3
#define RANGE_OUT		2
#define	RANGE_IN		1
#define RANGE_NULL		0

#define RELEASE			0
#define GRABBED			1
#define RELEASE_CLOSE	2
#define GRABBED_CLOSE	3

#define P_WIN_VAR 0.001
#define P_ERR_VAR 0.01

class ActionPrediction : public CData,
						 public Evaluate
{

private:
	struct predict_n // prediction for node
	{
		double 			acc; // acc
		double 			vel; // velocity limit 0/1
		double 			surface_dist; // surface distance
		double 			curvature; // curvature value : 1 for line
	};
	struct predict_e // prediction for edge
	{
		double 			acc; // acc
		double 			vel; // velocity limit 0/1
		double 			curvature; // curvature value : 1 for line
		vector<double>	range; // in or outside
		vector<double> 	err; // prediction error = diff from the sectormap
		vector<double> 	pct_err; // prob shared between multiple predictions of inside
		vector<double>	err_diff; // change in the error compared to original
		vector<double>	pct_err_diff; // change in the error compared to original
		vector<double>	window; // knot in trajectory
	};

	int label1_ap; // start
	int label2_ap; // goal
	bool learn;
	bool la_sm_change; // change flag from sm to la
	CGraph::node_t node_ap; //from label1
	predict_e pred_sm; // edge_prediction
	predict_n pred_la; // node_prediction
	vector<int> last_loc;
	vector<int> init;
	vector<int> range_in;
	vector<predict_e> pred_sm_mem;
	vector<predict_n> pred_la_mem;
	vector<vector<Vector4d> > pva_avg_mem; // rebuild SM

	void reshapePredictEdge(
			predict_e &P_,
			int size);
	void reshapePredictNode(
			predict_n &P_,
			int size);

public:
	ActionPrediction();
	virtual ~ActionPrediction();

	void PredictExt();
	void Init(
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
	double DecideLocationIntervalExt(
			int &loc_idx_,
			int loc_last_idx_,
			Vector4d point_,
			vector<Vector4d> mid_,
			vector<double> len_,
			vector<Vector3d> tangent_,
			int loc_offset_,
			int loc_init_);
	int DecideSectorIntervalExt(
			int &sec_idx_,
			int loc_idx_,
			Vector3d &delta_t_,
			Vector4d point_,
			vector<Vector4d> mid_,
			vector<Vector3d> tangent_,
			vector<Vector3d> normal_);
	bool DecideGoal(
			int label2_,
			double sm_i_, //sectormap single
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

};

#endif /* ACTIONPREDICTION_H_ */
