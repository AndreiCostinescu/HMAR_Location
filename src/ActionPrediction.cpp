/*
 * ActionPrediction.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "ActionPrediction.h"

ActionPrediction::ActionPrediction() :
						label1_ap(-1),
						label2_ap(-1),
						learn(false),
						la_sm_change(true),
						node_ap{},
						pred_sm{},
						pred_la{}
{
}

ActionPrediction::~ActionPrediction() { }

void ActionPrediction::PredictInit(
		bool learn_)
{
	learn = learn_;

	la_sm_change = true;

	reshapeVector(init, 		G->GetNumberOfNodes());
	reshapeVector(range_in, 	G->GetNumberOfNodes());
	reshapeVector(last_loc, 	G->GetNumberOfNodes());
	reshapePredictEdge(pred_sm, G->GetNumberOfNodes());
	reshapePredictNode(pred_la, G->GetNumberOfNodes());

	al_eval = KB.al;
	ac_eval = KB.ac;

	state_eval = G->GetActionState();

	state_eval.grasp 	= RELEASE;
	state_eval.label1 	= label1_ap;
	state_eval.label2 	= label1_ap;
	state_eval.mov 		=  0.0;
	state_eval.pct_err 	= -1;
	state_eval.sur 		= -1;
	state_eval.sur_dist 	= 0.0;

	for(int i=KB.ac["GEOMETRIC"].first;i<KB.ac["GEOMETRIC"].second+1;i++)
	{
		state_eval.goal	[KB.al[i]] = 0.0;
		state_eval.window[KB.al[i]] = 0.0;
	}

	G->SetActionState(state_eval);
}

void ActionPrediction::PredictExt()
{
	state_eval = G->GetActionState();
	if (contact==1)
	{
		if (state_eval.grasp==RELEASE)
		{
			state_eval.grasp = GRABBED_CLOSE;
			G->SetActionState(state_eval);
			this->Predict();
		}
		else
		{
			state_eval.grasp = GRABBED;
			G->SetActionState(state_eval);
			this->Predict();
		}
	}
	else
	{
		if (state_eval.grasp==GRABBED)
		{
			state_eval.grasp = RELEASE_CLOSE;
			G->SetActionState(state_eval);
			this->Predict();
		}
		else
		{
			state_eval.grasp 	= RELEASE;
			state_eval.label2 	= state_eval.label1;
			state_eval.mov 		= 0.0;
			// state_eval.pct_err 	= -1; // should be -1 because it is in LA
			// state_eval.sur 		= 0; // should just follow whatever that was determined beforehand
			state_eval.sur_dist 	= 0.0;
			for(int i=KB.ac["GEOMETRIC"].first;
					i<KB.ac["GEOMETRIC"].second+1;
					i++)
			{
				state_eval.goal[KB.al[i]] = 0.0;
				state_eval.window[KB.al[i]] = 0.0;
			}
			G->SetActionState(state_eval);
		}
	}
}

int ActionPrediction::Predict()
{
	// 1. Contact trigger
	// 1.1 Check if the object is within a sphere volume of the location areas
	this->ContactTrigger();

	// 2. Prediction during motion
	if (pva[0][3] < 0)
	{
		if (!la_sm_change) { pred_la_mem.clear(); }
		la_sm_change = true;
		pva_avg_mem.push_back(pva); // rebuild SM
		this->EdgePrediction();
	}

	// 3. Prediction within location area
	else
	{
		if (la_sm_change)
		{

//			// ### TODO not really correct because the clusters are still there
//			// just to show how the sectormap changes with time
//			if (label1_ap!=pva_avg[0].l && label1_ap>=0)
//			{
//				pred_sm_mem.clear();
//				reshapeVector(last_loc, G->GetNumberOfNodes());
//
//				if (learn)
//				{
//					this->RebuildSectorMap(pva_avg_mem, label1_ap, pva_avg[0].l);
//					pva_avg_mem.clear();
//				}
//			}

			// i. clear SM prediction, reset SM initial and last location flags
			{
				pred_sm_mem.clear();
				reshapeVector(init, G->GetNumberOfNodes());
				reshapeVector(last_loc, G->GetNumberOfNodes());
			}

			// ii. update label 1
			{
				label1_ap 	= pva[0][3];
				node_ap		= G->GetNode(label1_ap);
			}

			// iii. update action state_eval
			{
				state_eval = G->GetActionState();
				for(int i=0;i<KB.al.size();i++)
				{
					if (!strcmp(
							node_ap.name.c_str(),
							KB.al[i].c_str()))
					{ state_eval.label1 = state_eval.label2 = i; }
				}
				for(int i=KB.ac["GEOMETRIC"].first;
						i<KB.ac["GEOMETRIC"].second+1;
						i++)
				{
					state_eval.goal[KB.al[i]] = 0.0;
					state_eval.window[KB.al[i]] = 0.0;
				}
				state_eval.mov 		= V4d3d(pva[1]).norm();
				state_eval.pct_err 	= -1;
				state_eval.sur 		= node_ap.surface;
				G->SetActionState(state_eval);
			}

		}

		this->NodePrediction();

		la_sm_change = false;
	}

	// 4. OUTPUT
	// says what i am currently doing
	return EXIT_SUCCESS;
}

int ActionPrediction::ContactTrigger()
{
	// initial case
	if (label1_ap < 0)
	{
		this->DecideBoundaryClosestExt();
		state_eval = G->GetActionState();
		state_eval.grasp = GRABBED_CLOSE;
		G->SetActionState(state_eval);
	}
	else
	{
		// give a starting location during change from release to move and vice versa
		if ((G->GetActionState().grasp==GRABBED_CLOSE) ||
			(G->GetActionState().grasp==RELEASE_CLOSE))
		{
			this->DecideBoundaryClosestExt();
		}
		else
		{
			this->DecideBoundarySphereExt();

			if (pva[0][3]>=0)
			{
//				if (node_tmp.surface>0)
//				{
//					if (!decideSurface(pva_avg[0], G->GetSurfaceEq()[node_tmp.surface-1], 0.2))
//						pva_avg[0].l = UNCLASSIFIED;
//				}

				if (G->GetNode((int)pva[0][3]).surface > 0)
				{
					Vector4d point_rot, point_rot2;
					Matrix4d T;
					T << KB.surface_rot[G->GetNode((int)pva[0][3]).surface-1].row(0), 0,
						 KB.surface_rot[G->GetNode((int)pva[0][3]).surface-1].row(1), 0,
						 KB.surface_rot[G->GetNode((int)pva[0][3]).surface-1].row(2), 0,
						 0,0,0,0;

					// Last element of T is zero because last vector element is used for labeling.
					// Point transform to the coordinate system of the surface.
					// p' = [R,R*t]*p
					point_rot  =
							(T * pva[0]) -
							(T * G->GetNode((int)pva[0][3]).centroid);
					point_rot[3] = pva[0][3];

					this->DecideBoundaryCuboidExt(
							point_rot,
							G->GetNode((int)pva[0][3]).box_min,
							G->GetNode((int)pva[0][3]).box_max);
					pva[0][3] = point_rot[3];
				}

				// prevent from going to unknown goal locations during middle of movement
				for(int i=0;i<G->GetNumberOfNodes();i++)
				{
					if (pred_sm.pct_err[i]>0 && last_loc[i]>LOC_INT/10 && last_loc[i]<LOC_INT-(LOC_INT/10))
					{
						if (pred_sm.pct_err[(int)pva[0][3]]==0 && pred_sm.range[(int)pva[0][3]]!=RANGE_EXCEED)
						{
							pva[0][3] = UNCLASSIFIED;
						}
						break;
					}
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::DecideBoundarySphereExt()
{
	return decideBoundarySphere(pva[0], G->GetCentroidList());
}

int ActionPrediction::DecideBoundaryCuboidExt(
	Vector4d &point_,
	Vector3d box_min_,
	Vector3d box_max_)
{
	Vector3d tmp = box_max_-box_min_;// tmp[1] = 0.0;
	box_min_ -= (tmp*0.2);
	box_max_ += (tmp*0.2);
	return decideBoundaryCuboid(point_, box_min_, box_max_);
}

int ActionPrediction::DecideBoundaryClosestExt()
{
	return decideBoundaryClosest(pva[0], G->GetCentroidList());
}

int ActionPrediction::EdgePrediction()
{
	reshapePredictEdge(pred_sm, G->GetNumberOfNodes());

	// 1. Check for motion
	this->DecideMovement(true);

	// 2. Check if the trajectory is within the range of sector map
	this->PredictFromSectorMap();

	// prediction average
	{
		pred_sm_mem.push_back(pred_sm);
		if (pred_sm_mem.size()>3) {pred_sm_mem.erase(pred_sm_mem.begin());}
	}

	// 3. Predict the goal based on the trajectory error from sector map
	this->EvaluateEdgePrediction();

//	if (label1_ap==2)
//	{
//		for(int i=0;i<G->GetNumberOfNodes();i++)
//		{
//			cout << i << " : " << last_loc[i] << "," << pct_err_eval[i] << "," << pred_sm.pct_err[i] << "," << pred_sm.range[i] << "  ::  ";
//		}
//		cout << endl;
//	}

	return EXIT_SUCCESS;
}

int ActionPrediction::NodePrediction()
{
	reshapePredictNode(pred_la, 0);

	// 1. check movement
	this->DecideMovement(false);

	// 2. check surface distance
	if (node_ap.surface > 0)
	{
		pred_la.surface_dist =
				checkSurfaceDistance(pva[0], KB.surface_eq[node_ap.surface-1]);
	}

	// prediction average
	{
		pred_la_mem.push_back(pred_la);
		if (pred_la_mem.size()>3) {pred_la_mem.erase(pred_la_mem.begin());}
	}

	// 3. Evaluate (for now whether it is moving on the surface)
	this->EvaluateNodePrediction();

	return EXIT_SUCCESS;
}

// #### TODO: vel limit
int ActionPrediction::DecideMovement(bool x_)
{
	// edge
	if(x_)
	{
		if (V4d3d(pva[1]).norm()<0.001)
		{ pred_sm.vel = pred_sm.acc = 0; }
		else
		{
			pred_sm.vel = V4d3d(pva[1]).norm();
			pred_sm.acc = V4d3d(pva[2]).norm();
		}
	}
	// node
	else
	{
		if (V4d3d(pva[1]).norm()<0.001)
		{ pred_la.vel = pred_la.acc = 0; }
		else
		{
			pred_la.vel = V4d3d(pva[1]).norm();
			pred_la.acc = V4d3d(pva[2]).norm();
		}
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::PredictFromSectorMap()
{
	reshapeVector(range_in,G->GetNumberOfNodes());

	for(int i=0;i<G->GetNumberOfNodes();i++)
	{
		pred_sm.range[i] 		= RANGE_NULL;
		pred_sm.err[i] 			= 0.0;
		pred_sm.pct_err[i]   	= 0.0;
		pred_sm.err_diff[i] 	= 0.0;
		pred_sm.pct_err_diff[i] = 0.0;
		pred_sm.oscillate[i]	= 0.0;
		pred_sm.window[i]		= 0.0;
		label2_ap 				= i;

		if (label1_ap==i) {continue;}
		if (G->GetEdgeCounter(label1_ap, i, 0)==0) {continue;}

		Vector3d delta_t;
		int loc_idx, sec_idx; loc_idx=sec_idx=-1;

		// LOC SEC INT
		int init_tmp = init[i];
		double tmp_dis =
				this->DecideLocSecInt(
						delta_t, sec_idx, loc_idx, last_loc[i], init_tmp);

//		if(label1_ap==2)
//			cout << "AA : " << loc_idx << endl;

		init[i] = init_tmp;
		if(last_loc[i]==0) { init[i] = 0; }

		vector<double> sm_tmp;
		G->GetEdgeSectorMap(label1_ap, i, 0, sm_tmp);

		// GLA
		bool flag =
				this->DecideGoal(
						i, sm_tmp[loc_idx*SEC_INT + sec_idx],
						delta_t.norm(), tmp_dis);
		if (!flag) {continue;}

		// window
		this->DecideWindow(sm_tmp, loc_idx, i);
//		if(i == 2) cout << loc_idx << " " <<predict_.window[i] << endl;

	}

	// Give priority to range in
	int sum_tmp = accumulate(range_in.begin(), range_in.end(), 0.0, addFunction);

//	if (sum_tmp>0)
//	{
//		for(int i=0;i<G->GetNumberOfNodes();i++)
//		{
//			if (pred_sm.range[i]!=RANGE_IN)
//			{
//				pred_sm.pct_err[i] *= 0.5;
//			}
//		}
//	}

	return EXIT_SUCCESS;
}

double ActionPrediction::DecideLocSecInt(
	Vector3d &delta_t_,
	int &sec_idx_,
	int &loc_idx_,
	int &loc_last_idx_,
	int &init_)
{
	double tmp_dis, offset, last;

	if (init_==0)	{offset = LOC_INT*0.75;}
	else			{offset = LOC_INT*0.50;}

	last = loc_last_idx_;

	tmp_dis  =
			dLIPredict(
					loc_idx_,
					loc_last_idx_,
					(pva[0] - node_ap.centroid),
					G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
					G->GetEdge(label1_ap, label2_ap, 0).loc_len,
					G->GetEdge(label1_ap, label2_ap, 0).tan,
					offset,
					init_);

	if(loc_idx_>0) init_ = 1;

	// going forward
	if(loc_idx_-loc_last_idx_>0)
	{
		for(int i=loc_last_idx_+1;i<loc_idx_+1;i++)
		{
			decideSectorInterval(
					sec_idx_,
					i,
					delta_t_,
					(pva[0] - node_ap.centroid),
					G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
					G->GetEdge(label1_ap, label2_ap, 0).tan,
					G->GetEdge(label1_ap, label2_ap, 0).nor);
		}
	}
	// going backward
	else if(loc_idx_-loc_last_idx_<0)
	{
		for(int i=loc_idx_+1;i<loc_last_idx_+1;i++)
		{
			decideSectorInterval(
					sec_idx_,
					i,
					delta_t_,
					(pva[0] - node_ap.centroid),
					G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
					G->GetEdge(label1_ap, label2_ap, 0).tan,
					G->GetEdge(label1_ap, label2_ap, 0).nor);
		}
	}
	// same loc int
	else
	{
		if (init_==1 && last>0) {loc_idx_ = last;}

		decideSectorInterval(
				sec_idx_,
				loc_idx_,
				delta_t_,
				(pva[0] - node_ap.centroid),
				G->GetEdge(label1_ap, label2_ap, 0).loc_mid,
				G->GetEdge(label1_ap, label2_ap, 0).tan,
				G->GetEdge(label1_ap, label2_ap, 0).nor);
	}

	loc_last_idx_ = loc_idx_;
	return tmp_dis;
}

bool ActionPrediction::DecideGoal(
	int label2_,
	double &sm_i_, //sectormap single
	double delta_t_,
	double loc_error_)
{
	if (loc_error_ > 0.15) //## TODO :NEED TO VERIFY
	{
		pred_sm.range[label2_] = RANGE_EXCEED;
		pred_sm.err[label2_] = delta_t_ - sm_i_;
		pred_sm.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, pred_sm.err[label2_]);
		return false;
	}

	if (delta_t_ <= sm_i_)
	{
		range_in[label2_] = 1;
		pred_sm.range[label2_] = RANGE_IN;
		pred_sm.pct_err[label2_]   = 0.999999;
	}
	else
	{
		pred_sm.range[label2_] = RANGE_OUT;
		pred_sm.err[label2_] = delta_t_ - sm_i_;
		pred_sm.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, pred_sm.err[label2_]);
	}

	return true;
}

int ActionPrediction::DecideWindow(
	vector<double> sm_,
	int loc_idx_,
	int label2_)
{
	double max_val = 0.0;
	for(int s=0;s<SEC_INT;s++)
	{
		max_val = max(sm_[loc_idx_*SEC_INT+s],max_val);
	}

	pred_sm.window[label2_] = max_val;

	if(max_val > 0)
	{
		pred_sm.window[label2_] = min(pdfExp(P_WIN_VAR,0,max_val), 1.0);
	}

	return EXIT_SUCCESS;
}

int ActionPrediction::EvaluateNodePrediction()
{
	vector<double> surface_dist;
	for(int i=0;i<pred_la_mem.size();i++)
	{
		surface_dist.push_back(pred_la_mem[i].surface_dist);
	}

	surface_dist_eval 	= average(surface_dist);
//	surface_dist_eval 	= pred_la_mem.back().surface_dist;
	vel_eval 			= pred_la_mem.back().vel;
	label1_eval 		= label1_ap;

	win_eval.clear();
	pct_err_eval.clear();

	this->UpdateStateNode(G);

	return EXIT_SUCCESS;
}

int ActionPrediction::EvaluateEdgePrediction()
{
	vector<vector<double> > err; err.resize(G->GetNumberOfNodes());
	vector<vector<double> > win; win.resize(G->GetNumberOfNodes());
	reshapeVector(pct_err_eval,G->GetNumberOfNodes());
	reshapeVector(win_eval,G->GetNumberOfNodes());

	for(int i=0;i<pred_sm_mem.size();i++)
	{
		for(int ii=0;ii<G->GetNumberOfNodes();ii++)
		{
			win[ii].push_back(pred_sm_mem[i].window[ii]);
			err[ii].push_back(pred_sm_mem[i].pct_err[ii]);
		}
	}

	double sum_tmp = 0.0;
	double counter = 0.0;
	bool flag = false;
	for(int i=0;i<G->GetNumberOfNodes();i++)
	{
		win_eval[i] 	= average(win[i]);
		pct_err_eval[i] = average(err[i]);

		if (G->GetEdgeCounter(label1_ap,i,0)>0) { counter += 1.0; }

		if (last_loc[i]>LOC_INT/10) { flag = true; }

		sum_tmp += pct_err_eval[i];
	}

	for(int i=0;i<G->GetNumberOfNodes();i++)
	{
		if (flag)
		{
			if (sum_tmp == 0.0) { pct_err_eval[i] = 0.0;}
			else				{ pct_err_eval[i] /= sum_tmp;}
		}
		else
		{
			// this will show zero probability but do we need it ????
//			if (pred_sm_mem.back().range[i]==RANGE_EXCEED)
//			{
//				pct_err_eval[i] = 0.0;
//			}
//			else
//			{
//				if (G->GetEdgeCounter(label1_ap,i,0)>0)
//				{
//					pct_err_eval[i] = 1.0/counter;
//				}
//			}
			if (G->GetEdgeCounter(label1_ap,i,0)>0)
			{ pct_err_eval[i] = 1.0/counter; }
		}
	}

	vel_eval 			= pred_sm_mem.back().vel;
	label1_eval 		= label1_ap;
	surface_dist_eval 	= 0.0;

	this->UpdateStateEdge(G);

	return EXIT_SUCCESS;
}

//int ActionPrediction::RebuildSectorMap(
//	vector<vector<point_d> > pva_avg_,
//	int	label1_,
//	int label2_)
//{
//	if (pva_avg_.size() < 5) { return EXIT_FAILURE; }
//
//	vector<point_d> pts_avg, vel_avg;
//	for(int i=0;i<pva_avg_.size();i++)
//	{
//		pts_avg.push_back(pva_avg_[i][0]);
//		vel_avg.push_back(pva_avg_[i][0]);
//	}
//
//	this->SetLabel1SM(label1_);
//	this->SetLabel2SM(label2_);
//	this->UpdateSectorMap(G, pts_avg);
//
//	//VISUALIZE
//	if(0)
//	{
//		vector<point_d> point_zero; vector<string> label_zero;
//		vector<vector<unsigned char> > color_code; colorCode(color_code);
//		showConnection(G, pts_avg, label_zero, color_code, true);
//	}
//
//	return EXIT_SUCCESS;
//}
