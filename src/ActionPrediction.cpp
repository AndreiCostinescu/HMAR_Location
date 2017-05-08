/*
 * ActionPrediction.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "ActionPrediction.h"

ActionPrediction::ActionPrediction(
	Graph *Graph_,
	bool learn_) : Evaluate(Graph_)
{
	label1_ap = -1;
	label2_ap = -1;
	learn = learn_;
	la_sm_change = true;
	node_ap = {};
	pred_sm = {};
	pred_la = {};
	state = {};
	last_loc.clear();
	init.clear();
	pva_avg.clear();
	pva_avg_mem.clear();
	pred_sm_mem.clear();

	reshapeVector(init, 	G->GetNumberOfNodes());
	reshapeVector(range_in, G->GetNumberOfNodes());
	reshapeVector(last_loc, G->GetNumberOfNodes());
	reshapePredictEdge(pred_sm, G->GetNumberOfNodes());
}

ActionPrediction::~ActionPrediction() { }

void ActionPrediction::PredictInit()
{
	state = G->GetActionState();

	state.grasp 	= RELEASE;
	state.label1 	= label1_ap;
	state.label2 	= label1_ap;
	state.mov 		=  0.0;
	state.pct_err 	= -1;
	state.sur 		= -1;
	state.sur_dist 	= 0.0;

	vector<string> al_tmp = G->GetActionLabel();
	map<string,pair<int,int> > ac_tmp = G->GetActionCategory();
	for(int i=ac_tmp["GEOMETRIC"].first;i<ac_tmp["GEOMETRIC"].second+1;i++)
	{
		state.goal	[al_tmp[i]] = 0.0;
		state.window[al_tmp[i]] = 0.0;
	}

	G->SetActionState(state);
}

void ActionPrediction::PredictExt(
	vector<point_d> &pva_avg_,
	int contact_)
{
	pva_avg = pva_avg_;
	state = G->GetActionState();
	if (contact_==1)
	{
		if (state.grasp==RELEASE)
		{
			state.grasp = GRABBED_CLOSE;
			G->SetActionState(state);
			this->Predict();
		}
		else
		{
			state.grasp = GRABBED;
			G->SetActionState(state);
			this->Predict();
		}
	}
	else
	{
		if (state.grasp==GRABBED)
		{
			state.grasp = RELEASE_CLOSE;
			G->SetActionState(state);
			this->Predict();
		}
		else
		{
			state.grasp 	= RELEASE;
			state.label2 	= state.label1;
			state.mov 		= 0.0;
			// state.pct_err 	= -1; // should be -1 because it is in LA
			// state.sur 		= 0; // should just follow whatever that was determined beforehand
			state.sur_dist 	= 0.0;
			for(int i=G->GetActionCategory()["GEOMETRIC"].first;
					i<G->GetActionCategory()["GEOMETRIC"].second+1;
					i++)
			{
				state.goal[G->GetActionLabel()[i]] = 0.0;
				state.window[G->GetActionLabel()[i]] = 0.0;
			}
			G->SetActionState(state);
		}
	}
	pva_avg_ = pva_avg;
}

int ActionPrediction::Predict()
{
	// 1. Contact trigger
	// 1.1 Check if the object is within a sphere volume of the location areas
	this->ContactTrigger();

	// 2. Prediction during motion
	if (pva_avg[0].l < 0)
	{
		if (!la_sm_change) { pred_la_mem.clear(); }
		la_sm_change = true;
		pva_avg_mem.push_back(pva_avg); // rebuild SM
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
				label1_ap = pva_avg[0].l;
				G->GetNode(label1_ap, node_ap);
			}

			// iii. update action state
			{
				state = G->GetActionState();
				for(int i=0;i<G->GetActionLabel().size();i++)
				{
					if (!strcmp(
							node_ap.name.c_str(),
							G->GetActionLabel()[i].c_str()))
					{ state.label1 = state.label2 = i; }
				}
				for(int i=G->GetActionCategory()["GEOMETRIC"].first;
						i<G->GetActionCategory()["GEOMETRIC"].second+1;
						i++)
				{
					state.goal[G->GetActionLabel()[i]] = 0.0;
					state.window[G->GetActionLabel()[i]] = 0.0;
				}
				state.mov 		= l2Norm(pva_avg[1]);
				state.pct_err 	= -1;
				state.sur 		= node_ap.surface;
				G->SetActionState(state);
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
		this->DecideBoundaryClosestExt(pva_avg[0], G->GetCentroidList());
		state = G->GetActionState();
		state.grasp = GRABBED_CLOSE;
		G->SetActionState(state);
	}
	else
	{
		// give a starting location during change from release to move and vice versa
		if ((G->GetActionState().grasp==GRABBED_CLOSE) ||
			(G->GetActionState().grasp==RELEASE_CLOSE))
		{
			this->DecideBoundaryClosestExt(pva_avg[0], G->GetCentroidList());
		}
		else
		{
			this->DecideBoundarySphereExt(pva_avg[0], G->GetCentroidList());

			if (pva_avg[0].l>=0)
			{
				node_tt node_tmp = {};
				G->GetNode((int)pva_avg[0].l, node_tmp);

//				if (node_tmp.surface>0)
//				{
//					if (!decideSurface(pva_avg[0], G->GetSurfaceEq()[node_tmp.surface-1], 0.2))
//						pva_avg[0].l = UNCLASSIFIED;
//				}

				if (node_tmp.surface > 0)
				{
					// point transform to the coord system of the surface
					point_d point_rot={}, point_rot2={};

					point_rot.x =
							G->GetSurfaceRot()[node_tmp.surface-1][0]*pva_avg[0].x +
							G->GetSurfaceRot()[node_tmp.surface-1][1]*pva_avg[0].y +
							G->GetSurfaceRot()[node_tmp.surface-1][2]*pva_avg[0].z;
					point_rot.y =
							G->GetSurfaceRot()[node_tmp.surface-1][3]*pva_avg[0].x +
							G->GetSurfaceRot()[node_tmp.surface-1][4]*pva_avg[0].y +
							G->GetSurfaceRot()[node_tmp.surface-1][5]*pva_avg[0].z;
					point_rot.z =
							G->GetSurfaceRot()[node_tmp.surface-1][6]*pva_avg[0].x +
							G->GetSurfaceRot()[node_tmp.surface-1][7]*pva_avg[0].y +
							G->GetSurfaceRot()[node_tmp.surface-1][8]*pva_avg[0].z;
					point_rot2.x =
							G->GetSurfaceRot()[node_tmp.surface-1][0]*node_tmp.centroid.x +
							G->GetSurfaceRot()[node_tmp.surface-1][1]*node_tmp.centroid.y +
							G->GetSurfaceRot()[node_tmp.surface-1][2]*node_tmp.centroid.z;
					point_rot2.y =
							G->GetSurfaceRot()[node_tmp.surface-1][3]*node_tmp.centroid.x +
							G->GetSurfaceRot()[node_tmp.surface-1][4]*node_tmp.centroid.y +
							G->GetSurfaceRot()[node_tmp.surface-1][5]*node_tmp.centroid.z;
					point_rot2.z =
							G->GetSurfaceRot()[node_tmp.surface-1][6]*node_tmp.centroid.x +
							G->GetSurfaceRot()[node_tmp.surface-1][7]*node_tmp.centroid.y +
							G->GetSurfaceRot()[node_tmp.surface-1][8]*node_tmp.centroid.z;

					point_rot = minusPoint(point_rot, point_rot2);
					point_rot.l = pva_avg[0].l;

					this->DecideBoundaryCuboidExt(
							point_rot, node_tmp.box_min, node_tmp.box_max);

					pva_avg[0].l = point_rot.l;
				}

				// prevent from going to unknown goal locations during middle of movement
				for(int i=0;i<G->GetNumberOfNodes();i++)
				{
					if (pred_sm.pct_err[i]>0 && last_loc[i]>LOC_INT/10 && last_loc[i]<LOC_INT-(LOC_INT/10))
					{
						if (pred_sm.pct_err[(int)pva_avg[0].l]==0 && pred_sm.range[(int)pva_avg[0].l]!=RANGE_EXCEED)
						{
							pva_avg[0].l = UNCLASSIFIED;
						}
						break;
					}
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::DecideBoundarySphereExt(
	point_d 		&point_,
	vector<point_d> centroids_)
{
	return decideBoundarySphere(point_, centroids_);
}

int ActionPrediction::DecideBoundaryCuboidExt(
	point_d &point_,
	point_d box_min_,
	point_d box_max_)
{
	point_d tmp = minusPoint(box_max_,box_min_); tmp.y=0.0;
	box_min_ = minusPoint(box_min_, multiPoint(tmp,0.1));
	box_max_ =   addPoint(box_max_, multiPoint(tmp,0.1));
	return decideBoundaryCuboid(point_, box_min_, box_max_);
}

int ActionPrediction::DecideBoundaryClosestExt(
	point_d 		&point_,
	vector<point_d> centroids_)
{
	return decideBoundaryClosest(point_, centroids_);
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

//	if (label1_ap==1)
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
				checkSurfaceDistance(
						pva_avg[0],
						G->GetSurfaceEq()[node_ap.surface-1]);
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
		if (l2Norm(pva_avg[1])<0.001)
		{ pred_sm.vel = pred_sm.acc = 0; }
		else
		{
			pred_sm.vel = l2Norm(pva_avg[1]);
			pred_sm.acc = l2Norm(pva_avg[2]);
		}
	}
	// node
	else
	{
		if (l2Norm(pva_avg[1])<0.001)
		{ pred_la.vel = pred_la.acc = 0; }
		else
		{
			pred_la.vel = l2Norm(pva_avg[1]);
			pred_la.acc = l2Norm(pva_avg[2]);
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

		point_d delta_t;
		int loc_idx, sec_idx; loc_idx=sec_idx=-1;

		// LOC SEC INT
		int init_tmp = init[i];
		double tmp_dis =
				this->DecideLocSecInt(
						delta_t, sec_idx, loc_idx, last_loc[i], init_tmp);
		init[i] = init_tmp;
		if(last_loc[i]==0) { init[i] = 0; }

		vector<double> sm_tmp;
		G->GetEdgeSectorMap(label1_ap, i, 0, sm_tmp);

		// GLA
		bool flag =
				this->DecideGoal(
						i, sm_tmp[loc_idx*SEC_INT + sec_idx],
						l2Norm(delta_t), tmp_dis);
		if (!flag) {continue;}

		// window
		this->DecideWindow(sm_tmp, loc_idx, i);
//		if(i == 2) cout << loc_idx << " " <<predict_.window[i] << endl;

	}

	// give priority to range in
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
	point_d &delta_t_,
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
					minusPoint(pva_avg[0], node_ap.centroid),
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
					minusPoint(pva_avg[0], node_ap.centroid),
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
					minusPoint(pva_avg[0], node_ap.centroid),
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
				minusPoint(pva_avg[0], node_ap.centroid),
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

	this->UpdateStateNode();

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

	this->UpdateStateEdge();

	return EXIT_SUCCESS;
}

int ActionPrediction::RebuildSectorMap(
	vector<vector<point_d> > pva_avg_,
	int	label1_,
	int label2_)
{
	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

	if (pva_avg_.size() < 5) { return EXIT_FAILURE; }

	vector<point_d> pts_avg, vel_avg;
	for(int i=0;i<pva_avg_.size();i++)
	{
		pts_avg.push_back(pva_avg_[i][0]);
		vel_avg.push_back(pva_avg_[i][0]);
	}

	this->SetLabel1SM(label1_);
	this->SetLabel2SM(label2_);
	this->UpdateSectorMap(G, pts_avg);
	this->FindWindowConstraint(G);

	//VISUALIZE
	if(0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(G, pts_avg, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}
