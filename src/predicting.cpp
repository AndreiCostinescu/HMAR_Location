/*
 * predicting.cpp
 *
 *  Created on: Mar 19, 2017
 *      Author: chen
 */

#include "predicting.h"

int predictFromNode(
	Graph *Graph_,
	point_d &point_)
{
	return decideBoundary_(
			point_, point_, Graph_->getCentroidList(),
			Graph_->getSurfaceFlagList(), Graph_->getSurfaceEq(),
			Graph_->getSurfaceLimit());
}

int decideMovement(
	point_d vel_,
	point_d acc_,
	predict_t &predict_)
{
	// #### TODO: vel limit
	if (l2Norm(vel_)<0.001)
	{
		predict_.vel = 0;
		predict_.acc = 0;
	}
	else
	{
		predict_.vel = l2Norm(vel_);
		predict_.acc = l2Norm(acc_);
	}
	return EXIT_SUCCESS;
}

double decideLocSecInt(
	edge_tt edge_,
	point_d point_,
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
					point_,
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
					point_,
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
					point_,
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
				point_,
				edge_.loc_mid,
				edge_.tan,
				edge_.nor);
	}

	loc_last_idx_ = loc_idx_;
	return tmp_dis;
}

bool decideGoal(
	predict_t &predict_,
	int label2_,
	double &sm_i_, //sectormap single
	double delta_t_,
	double loc_error_)
{
	if (loc_error_ > 2.075) //## TODO :NEED TO VERIFY
	{
		predict_.range[label2_] = RANGE_EXCEED;
		return false;
	}

	if (delta_t_ <= sm_i_)
	{
		predict_.range[label2_] = RANGE_IN;
		predict_.pct_err[label2_]   = 0.999999;
	}
	else
	{
		predict_.range[label2_] = RANGE_OUT;
		predict_.err[label2_] = delta_t_ - sm_i_;
		predict_.pct_err[label2_] = pdfExp(P_ERR_VAR, 0.0, predict_.err[label2_]);
	}
	return true;
}

int decideWindow(
	predict_t &predict_,
	vector<double> sm_,
	int loc_idx_,
	int label2_)
{
	double max_val = 0.0;
	for(int s=0;s<SEC_INT;s++)
	{
		max_val = max(sm_[loc_idx_*SEC_INT+s],max_val);
	}

	predict_.window[label2_] = max_val;

	if(max_val > 0)
	{
		predict_.window[label2_] = min(pdfExp(P_WIN_VAR,0,max_val), 1.0);
	}

	return EXIT_SUCCESS;
}

int predictFromSectorMap(
	Graph *Graph_,
	point_d point_,
	predict_t &predict_,
	vector<int> &loc_last_idxs_,
	int label1_,
	vector<bool> &init_)
{
	for(int i=0;i<Graph_->getNumberOfNodes();i++)
	{
		predict_.range[i] 			= RANGE_NULL;
		predict_.err[i] 			= 0.0;
		predict_.pct_err[i]   		= 0.0;
		predict_.err_diff[i] 		= 0.0;
		predict_.pct_err_diff[i]   	= 0.0;
		predict_.oscillate[i]		= 0.0;
		predict_.window[i]			= 0.0;

		if (label1_==i) {continue;}
		if (Graph_->getEdgeCounter(label1_, i, 0)==0) {continue;}

		vector<point_d> tan_tmp, nor_tmp;
		vector<double> sm_tmp;
		Graph_->getEdgeTan(label1_, i, 0, tan_tmp);
		Graph_->getEdgeNor(label1_, i, 0, nor_tmp);
		Graph_->getEdgeSectorMap(label1_, i, 0, sm_tmp);

		point_d delta_t;
		int loc_idx, sec_idx;
		double tmp_dis;

		// [LOC SEC INT]*******************************************************
		loc_idx = -1;
		bool init_tmp = init_[i];
		tmp_dis =
				decideLocSecInt(
						Graph_->getEdge(label1_, i, 0), point_, delta_t,
						sec_idx, loc_idx, loc_last_idxs_[i], init_tmp);

		init_[i] = init_tmp;
		// *******************************************************[LOC SEC INT]

		// [GLA]***************************************************************
		bool flag =
				decideGoal(
						predict_, i, sm_tmp[loc_idx*SEC_INT + sec_idx],
						l2Norm(delta_t), tmp_dis);
		if (!flag) {continue;}
		// ***************************************************************[GLA]

		// [LOC INT CONSTRAINT]************************************************
		decideWindow(predict_, sm_tmp, loc_idx, i);
//		if(i == 2) cout << loc_idx << " " <<predict_.window[i] << endl;
		// ************************************************[LOC INT CONSTRAINT]
	}

	return EXIT_SUCCESS;
}

int evaluatePrediction(
	Graph *Graph_,
	vector<predict_t> predict_mem_,
	int label1_)
{
	map<string,pair<int,int> > ac_tmp = Graph_->getActionCategory();
	vector<string> al_tmp = Graph_->getActionLabel();
	vector<vector<double> > err; err.resize(Graph_->getNumberOfNodes());
	vector<vector<double> > win; win.resize(Graph_->getNumberOfNodes());
	vector<double> err_avg; err_avg.resize(Graph_->getNumberOfNodes());
	vector<double> win_avg; win_avg.resize(Graph_->getNumberOfNodes());

	Evaluate Eval;

	for(int i=0;i<predict_mem_.size();i++)
	{
		for(int ii=0;ii<Graph_->getNumberOfNodes();ii++)
		{
			win[ii].push_back(predict_mem_[i].window[ii]);
			err[ii].push_back(predict_mem_[i].pct_err[ii]);
		}
	}

	for(int i=0;i<Graph_->getNumberOfNodes();i++)
	{
		win_avg[i] = average(win[i]);
		err_avg[i] = average(err[i]);
	}

	Eval.setVelocity(predict_mem_.back().vel);
	Eval.setLabel(label1_);
	Eval.setWindow(win_avg);
	Eval.setPctErr(err_avg);
	Eval.setGraph(Graph_);
	Eval.update();
	Graph_->setState(Eval.getState());

	return EXIT_SUCCESS;
}

int predictFromEdge(
	Graph *Graph_,
	point_d pos_,
	point_d vel_,
	point_d acc_,
	vector<predict_t> &predict_mem_,
	vector<int> &last_loc_,
	int label1_,
	vector<bool> &init_)
{
	predict_t predict;
	reshapePredict(predict, Graph_->getNumberOfNodes());

	// 2.1. Check for motion
	decideMovement(vel_, acc_, predict);

	// 2.5. Check if the trajectory is within the range of sector map
	predictFromSectorMap(Graph_, pos_, predict, last_loc_, label1_, init_);

	// prediction average
	{
		predict_mem_.push_back(predict);
		if (predict_mem_.size()>10) {predict_mem_.erase(predict_mem_.begin());}
	}

	// 2.X. Predict the goal based on the trajectory error from sector map
	evaluatePrediction(Graph_, predict_mem_, label1_);

	return EXIT_SUCCESS;
}

int rebuildSectorMap(
	Graph 						*Graph_,
	vector<vector<point_d> > 	pva_avg_,
	int							label1_,
	int 						label2_)
{
	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

	if (pva_avg_.size() < 5) { return EXIT_FAILURE; }

	vector<point_d> pts_avg, vel_avg;
	for(int i=0;i<pva_avg_.size();i++)
	{
		pts_avg.push_back(pva_avg_[i][0]);
		vel_avg.push_back(pva_avg_[i][0]);
	}

	updateSectorMap(Graph_,
			pts_avg,
			label1_,
			label2_);
	findWindowConstraint(
			Graph_,
			label1_,
			label2_);

	//VISUALIZE
	if(0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, pts_avg, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int predictAction(
	Graph *Graph_,
	vector<point_d> &pva_avg_,
	vector<vector<point_d> > &pva_avg_mem_,
	vector<predict_t> &predict_mem_,
	int &label1_,
	vector<int> &last_loc_,
	vector<bool> &init_,
	bool learn_)
{

	// 1. Contact trigger
	// 1.1 Check if the object is within a sphere volume of the location areas
	predictFromNode(Graph_, pva_avg_[0]);

	// 2. Prediction during motion
	if (pva_avg_[0].l < 0)
	{
		pva_avg_mem_.push_back(pva_avg_);
		predictFromEdge(
				Graph_, pva_avg_[0], pva_avg_[1], pva_avg_[2],
				predict_mem_, last_loc_, label1_, init_);
	}

	// 3. Prediction within location area
	else
	{
		init_.clear();
		for(int is=0;is<Graph_->getNumberOfNodes();is++)
		{
			init_.push_back(true);
		}

		// ### TODO not really correct because the clusters are still there
		// just to show how the sectormap changes with time
		if (label1_!=pva_avg_[0].l && label1_>=0)
		{
			predict_mem_.clear();
			reshapeVector(last_loc_, Graph_->getNumberOfNodes());

			if (learn_)
			{
				rebuildSectorMap(Graph_, pva_avg_mem_, label1_, pva_avg_[0].l);
				pva_avg_mem_.clear();
			}
		}

		label1_ = pva_avg_[0].l;

		// there is a problem where the object gets out of bound for a while and return again
		vector<map<string, double> > p_tmp;
		Graph_->getPredictionReset(p_tmp);
		Graph_->setPrediction(p_tmp);

		node_tt nt{};
		Graph_->getNode(label1_,nt);
		state_t s_tmp = Graph_->getState();
		vector<string> al_tmp = Graph_->getActionLabel();
		map<string,pair<int,int> > ac_tmp = Graph_->getActionCategory();
		for(int i=0;i<al_tmp.size();i++)
		{
			if (!strcmp(nt.name.c_str(),al_tmp[i].c_str()))
			{
				s_tmp.label1 = i;
				s_tmp.label2 = i;
			}
		}
		s_tmp.con = nt.contact;
		s_tmp.sur = nt.surface;
		s_tmp.pct_err = -1;
		s_tmp.mov = l2Norm(pva_avg_[1]);
		for(int i=ac_tmp["GEOMETRIC"].first;i<ac_tmp["GEOMETRIC"].second+1;i++)
		{
			s_tmp.goal[al_tmp[i]] = 0.0;
		}
		Graph_->setState(s_tmp);

	}

	// 4. OUTPUT
	// says what i am currently doing

	return EXIT_SUCCESS;
}
