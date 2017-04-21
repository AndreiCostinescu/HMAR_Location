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
	learn = learn_;
	predict = {};
	state = {};
	last_loc.clear();
	init.clear();
	pva_avg.clear();
	pva_avg_mem.clear();
	predict_mem.clear();

	reshapeVector(last_loc, G->getNumberOfNodes());
	for(int is=0;is<G->getNumberOfNodes();is++) init.push_back(true);
}

ActionPrediction::~ActionPrediction() { }

void ActionPrediction::PredictExt(
	vector<point_d> pva_avg_,
	int contact_)
{
	pva_avg = pva_avg_;

	if (contact_==1)
	{
		state = G->getState();
		state.grasp = true;
		G->setState(state);
		this->Predict();
	}
	else
	{
		state = G->getState();
		state.grasp = false;
		G->setState(state);
	}
}

void ActionPrediction::PredictInit(
	point_d p_)
{
	decideBoundary_(
			p_, p_, G->getCentroidList(),
			G->getSurfaceFlagList(), G->getSurfaceEq(),
			G->getSurfaceLimit());

	label1_ap = p_.l;

	state = G->getState();
	state.grasp = false;
	state.label1 = p_.l;
	state.label2 = p_.l;
	state.con = -1;
	state.sur = -1;
	state.pct_err = -1;
	state.mov = 0;

	vector<string> al_tmp = G->getActionLabel();
	map<string,pair<int,int> > ac_tmp = G->getActionCategory();
	for(int i=ac_tmp["GEOMETRIC"].first;i<ac_tmp["GEOMETRIC"].second+1;i++)
	{
		state.goal[al_tmp[i]] = 0.0;
		state.window[al_tmp[i]] = -1.0;
	}

	G->setState(state);
}


int ActionPrediction::Predict()
{
	// 1. Contact trigger
	// 1.1 Check if the object is within a sphere volume of the location areas
	this->PredictFromNode();

	// 2. Prediction during motion
	if (pva_avg[0].l < 0)
	{
		pva_avg_mem.push_back(pva_avg);
		this->PredictFromEdge();
	}

	// 3. Prediction within location area
	else
	{
		init.clear();
		for(int is=0;is<G->getNumberOfNodes();is++) { init.push_back(true); }

		// ### TODO not really correct because the clusters are still there
		// just to show how the sectormap changes with time
		if (label1_ap!=pva_avg[0].l && label1_ap>=0)
		{
			predict_mem.clear();
			reshapeVector(last_loc, G->getNumberOfNodes());

			if (learn)
			{
				this->RebuildSectorMap(pva_avg_mem, label1_ap, pva_avg[0].l);
				pva_avg_mem.clear();
			}
		}

		label1_ap = pva_avg[0].l;

		// there is a problem where the object gets out of bound for a while and return again
		vector<map<string, double> > p_tmp;
		G->getPredictionReset(p_tmp);
		G->setPrediction(p_tmp);

		node_tt nt{};
		G->getNode(label1_ap,nt);
		state = G->getState();
		vector<string> al_tmp = G->getActionLabel();
		map<string,pair<int,int> > ac_tmp = G->getActionCategory();
		for(int i=0;i<al_tmp.size();i++)
		{
			if (!strcmp(nt.name.c_str(),al_tmp[i].c_str()))
			{
				state.label1 = i;
				state.label2 = i;
			}
		}
		state.con = nt.contact;
		state.sur = nt.surface;
		state.pct_err = -1;
		state.mov = l2Norm(pva_avg[1]);
		for(int i=ac_tmp["GEOMETRIC"].first;i<ac_tmp["GEOMETRIC"].second+1;i++)
		{
			state.goal[al_tmp[i]] = 0.0;
			state.window[al_tmp[i]] = -1.0;
		}
		G->setState(state);

	}

	// 4. OUTPUT
	// says what i am currently doing
	return EXIT_SUCCESS;
}

int ActionPrediction::PredictFromNode()
{
	if (label1_ap<0)
		return decideBoundaryClosest_(pva_avg[0], G->getCentroidList());
	else
		return decideBoundary_(
				pva_avg[0], pva_avg[0], G->getCentroidList(),
				G->getSurfaceFlagList(), G->getSurfaceEq(),
				G->getSurfaceLimit());
}

int ActionPrediction::PredictFromEdge()
{
	reshapePredict(predict, G->getNumberOfNodes());

	// 2.1. Check for motion
	this->DecideMovement();

	// 2.5. Check if the trajectory is within the range of sector map
	this->PredictFromSectorMap();

	// prediction average
	{
		predict_mem.push_back(predict);
		if (predict_mem.size()>10) {predict_mem.erase(predict_mem.begin());}
	}

	// 2.X. Predict the goal based on the trajectory error from sector map
	this->EvaluatePrediction();

	return EXIT_SUCCESS;
}

int ActionPrediction::DecideMovement()
{
	// #### TODO: vel limit
	if (l2Norm(pva_avg[1])<0.001)
	{
		predict.vel = 0;
		predict.acc = 0;
	}
	else
	{
		predict.vel = l2Norm(pva_avg[1]);
		predict.acc = l2Norm(pva_avg[2]);
	}
	return EXIT_SUCCESS;
}

int ActionPrediction::PredictFromSectorMap()
{

	for(int i=0;i<G->getNumberOfNodes();i++)
	{
		predict.range[i] 		= RANGE_NULL;
		predict.err[i] 			= 0.0;
		predict.pct_err[i]   	= 0.0;
		predict.err_diff[i] 	= 0.0;
		predict.pct_err_diff[i] = 0.0;
		predict.oscillate[i]	= 0.0;
		predict.window[i]		= 0.0;

		if (label1_ap==i) {continue;}
		if (G->getEdgeCounter(label1_ap, i, 0)==0) {continue;}

		vector<point_d> tan_tmp, nor_tmp;
		vector<double> sm_tmp;
		G->getEdgeTan(label1_ap, i, 0, tan_tmp);
		G->getEdgeNor(label1_ap, i, 0, nor_tmp);
		G->getEdgeSectorMap(label1_ap, i, 0, sm_tmp);

		point_d delta_t;
		int loc_idx, sec_idx;
		loc_idx = sec_idx = -1;

		// LOC SEC INT
		bool init_tmp = init[i];
		double tmp_dis =
				this->DecideLocSecInt(
						G->getEdge(label1_ap, i, 0), delta_t, sec_idx, loc_idx,
						last_loc[i], init_tmp);
		init[i] = init_tmp;

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

	return EXIT_SUCCESS;
}

double ActionPrediction::DecideLocSecInt(
	edge_tt edge_,
	point_d &delta_t_,
	int &sec_idx_,
	int &loc_idx_,
	int &loc_last_idx_,
	bool &init_)
{
	double tmp_dis, offset, last;

	if (init_)	{offset = LOC_INT;}
	else		{offset = LOC_INT;}

	last = loc_last_idx_;

	tmp_dis  =
			dLIPredict(
					loc_idx_,
					loc_last_idx_,
					pva_avg[0],
					edge_.loc_start,
					edge_.loc_mid,
					edge_.loc_end,
					edge_.tan,
					offset,
					init_);

	if(loc_idx_>0) init_ = false;

	if(loc_idx_-loc_last_idx_>0)
	{
		for(int i=loc_last_idx_+1;i<loc_idx_+1;i++)
		{
			decideSectorInterval(
					sec_idx_,
					i,
					delta_t_,
					pva_avg[0],
					edge_.loc_mid,
					edge_.tan,
					edge_.nor);
		}
	}
	else if(loc_idx_-loc_last_idx_<0)
	{
		for(int i=loc_idx_+1;i<loc_last_idx_+1;i++)
		{
			decideSectorInterval(
					sec_idx_,
					i,
					delta_t_,
					pva_avg[0],
					edge_.loc_mid,
					edge_.tan,
					edge_.nor);
		}
	}
	else
	{
		if(!init_ && last>0) {loc_idx_ = last;}

		decideSectorInterval(
				sec_idx_,
				loc_idx_,
				delta_t_,
				pva_avg[0],
				edge_.loc_mid,
				edge_.tan,
				edge_.nor);
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
	if (loc_error_ > 2.075) //## TODO :NEED TO VERIFY
	{
		predict.range[label2_] = RANGE_EXCEED;
		return false;
	}

	if (delta_t_ <= sm_i_)
	{
		predict.range[label2_] = RANGE_IN;
		predict.pct_err[label2_]   = 0.999999;
	}
	else
	{
		predict.range[label2_] = RANGE_OUT;
		predict.err[label2_] = delta_t_ - sm_i_;
		predict.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, predict.err[label2_]);
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

	predict.window[label2_] = max_val;

	if(max_val > 0)
	{
		predict.window[label2_] = min(pdfExp(P_WIN_VAR,0,max_val), 1.0);
	}

	return EXIT_SUCCESS;
}

int ActionPrediction::EvaluatePrediction()
{
	vector<vector<double> > err; err.resize(G->getNumberOfNodes());
	vector<vector<double> > win; win.resize(G->getNumberOfNodes());
	reshapeVector(pct_err_eval,G->getNumberOfNodes());
	reshapeVector(win_eval,G->getNumberOfNodes());

	for(int i=0;i<predict_mem.size();i++)
	{
		for(int ii=0;ii<G->getNumberOfNodes();ii++)
		{
			win[ii].push_back(predict_mem[i].window[ii]);
			err[ii].push_back(predict_mem[i].pct_err[ii]);
		}
	}

	for(int i=0;i<G->getNumberOfNodes();i++)
	{
		win_eval[i] = average(win[i]);
		pct_err_eval[i] = average(err[i]);
	}

	vel_eval = predict_mem.back().vel;
	label1_eval = label1_ap;

	this->UpdateEval();

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
