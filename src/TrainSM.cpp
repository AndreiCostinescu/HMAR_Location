/*
 * TrainSM.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "TrainSM.h"

TrainSM::TrainSM()
{
	label1=label2=label_idx=0;
}

TrainSM::~TrainSM() {
	// TODO Auto-generated destructor stub
}

//stack<clock_t> tictoc_stack;
//void tic() {tictoc_stack.push(clock());}
//void toc()
//{
//	cout << "Time elapsed: "
//       << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
//       << std::endl;
//	tictoc_stack.pop();
//}

void TrainSM::ClearSM()
{
	label1=label2=label_idx=0;
	pts_avg.clear();
	vel_avg.clear();
}

int TrainSM::FitCurve(
	vector<point_d> points_avg_,
	vector<point_d> &points_est_,
	vector<point_d> &coeffs_)
{
	vector<point_d> covs;
	reshapeVector(points_est_,(points_avg_.size())*4);
	polyCurveFitPoint(points_avg_, points_est_, coeffs_, covs, true);

	// Visualize by comparing both original and estimated points
	if(0)
	{
		vector<point_d> P = points_avg_;
		P.insert(P.end(),points_est_.begin(),points_est_.end());
		for(int i=0;i<points_avg_.size();i++) { P[i].l = 1; }
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<string> label_; label_.resize(2);
		vector<string> label_ref; label_ref.resize(2);
		vector<int> loc_idx_zero;
		showData(P, label_, label_ref, loc_idx_zero, color_code, true, false, false);
	}

	printer(18);
	return EXIT_SUCCESS;
}

int TrainSM::DecideSectorIntervalExt(
	edge_tt	edge_,
	point_d point_,
	point_d &delta_t_,
	int &sec_idx_,
	int loc_idx_)
{
	vector<point_d> mid	= edge_.loc_mid;
	vector<point_d> tan	= edge_.tan;
	vector<point_d> nor	= edge_.nor;
	return
			decideSectorInterval(
					sec_idx_, loc_idx_, delta_t_, point_, mid, tan, nor);
}

double TrainSM::DecideLocationIntervalExt(
	edge_tt	edge_,
	point_d point_,
	int &loc_idx_,
	int &loc_last_idx_,
	int loc_offset_,
	bool loc_init_)
{
	vector<point_d> beg	= edge_.loc_start;
	vector<point_d> mid	= edge_.loc_mid;
	vector<point_d> end	= edge_.loc_end;
	vector<point_d> tan	= edge_.tan;
	return
			decideLocationInterval(
					loc_idx_, loc_last_idx_, point_,
					beg, mid, end, tan, loc_offset_);
}

double TrainSM::DecideLocationIntervalExt(
	edge_tt	edge_,
	point_d point_,
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	int loc_offset_,
	bool loc_init_)
{
	vector<point_d> beg	= edge_.loc_start;
	vector<point_d> mid	= edge_.loc_mid;
	vector<point_d> end	= edge_.loc_end;
	vector<point_d> tan	= edge_.tan;
	return decideLocationInterval(
			loc_idxs_, loc_last_idx_, point_,
			beg, mid, end, tan, loc_offset_, loc_init_);
}

double TrainSM::DLIE(
	edge_tt	edge_,
	point_d point_,
	int &loc_idx_,
	int loc_last_idx_,
	int loc_offset_,
	bool loc_init_)
{
	vector<point_d> beg	= edge_.loc_start;
	vector<point_d> mid	= edge_.loc_mid;
	vector<point_d> end	= edge_.loc_end;
	vector<point_d> tan	= edge_.tan;
	return dLI(
			loc_idx_, loc_last_idx_, point_,
			beg, mid, end, tan, loc_offset_, loc_init_);
}

int TrainSM::AdjustSectorMap(
	edge_tt &edge_,
	vector<point_d> &points_,
	int &loc_last_idx_,
	int &loc_curr_idx_,
	double &delta_t_mem_,
	bool &loc_init_,
	int loc_offset_)
{
	point_d delta_t, delta_t_zero;
	int sec_idx = -1;
	int loc_idx = -1;
	int loc_last_idx_mem = loc_last_idx_;
	bool mem = loc_init_;
	// to deal with cases where the beginning is out of the sectormap
	// required for the fitting part only
	// also for the ending

	if (this->DLIE(edge_, points_[1],
			loc_idx, loc_last_idx_, loc_offset_, loc_init_) > 0.001)
	{
		if(mem || loc_last_idx_==LOC_INT-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
	}

	// find the delta_t
	this->DecideSectorIntervalExt(
			edge_, points_[1], delta_t, sec_idx, loc_idx);


	// to fill up the line if loc_int is valid but tangent is not aligned to traj (at a curve)
	double tmpmin = 10.0;
	double tmpdeltatmin = 0.0;
	int min_idx = 0;

	if (loc_idx==loc_last_idx_)
	{
		min_idx			= loc_idx*SEC_INT + sec_idx;
		tmpmin			= l2Norm(minusPoint(points_[1], edge_.loc_mid[loc_idx]));
		tmpdeltatmin	= l2Norm(delta_t);
	}
	else if (loc_idx>loc_last_idx_)
	{
		for(int ll=loc_last_idx_+1;ll<loc_idx+1;ll++)
		{
			this->DecideSectorIntervalExt(
					edge_, points_[1], delta_t_zero, sec_idx, ll);
			int tmp_idx = ll*SEC_INT + sec_idx;
			if (min_(l2Norm(minusPoint(points_[1], edge_.loc_mid[ll])), tmpmin))
			{
				tmpmin			= l2Norm(minusPoint(points_[1], edge_.loc_mid[ll]));
				tmpdeltatmin	= l2Norm(delta_t_zero);
				min_idx 		= tmp_idx;
			}
		}
	}


	if (loc_init_)
	{
		// if point starts before curve
		point_d proj_dir_tmp =
				multiPoint(
						edge_.tan[0],
						dotProduct(
								point2vector(minusPoint(points_[1],edge_.loc_mid[0])),
								point2vector(edge_.tan[0])));
		if (!vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(edge_.tan[0])))
		{
			loc_init_ = true;
			loc_last_idx_ = 0;
			loc_curr_idx_ = 0;
			return EXIT_SUCCESS;
		}
		else
		{
			loc_init_ = false;
			edge_.sector_map[min_idx] = max(edge_.sector_map[min_idx], tmpdeltatmin);
//					if (edge_.sector_map[min_idx]>0.3)  {return EXIT_FAILURE;}
			loc_last_idx_ = min_idx/SEC_INT;
			loc_curr_idx_ = min_idx/SEC_INT;
			return EXIT_SUCCESS;
		}
	}

	edge_.sector_map[min_idx] = max(edge_.sector_map[min_idx], tmpdeltatmin);
//			if (edge_.sector_map[min_idx]>0.3)  {return EXIT_FAILURE;}

	if ((min_idx/SEC_INT) - loc_last_idx_ > 1)
	{
		for(int l=loc_last_idx_+1;l<(min_idx/SEC_INT);l++)
		{
			this->DecideSectorIntervalExt(
					edge_, points_[1], delta_t_zero, sec_idx, l);
			int tmp_idx = l*SEC_INT + sec_idx;
			edge_.sector_map[tmp_idx] = max(edge_.sector_map[tmp_idx], l2Norm(delta_t_zero));
		}
		loc_curr_idx_ = (min_idx/SEC_INT);
	}
	else if ((min_idx/SEC_INT) - loc_last_idx_ == 1)
	{
		loc_curr_idx_ = (min_idx/SEC_INT);
	}


	loc_last_idx_ = loc_curr_idx_;

	return EXIT_SUCCESS;
}

int TrainSM::AdjustCurve(
	edge_tt &edge_,
	vector<point_d> &points_,
	int &loc_last_idx_,
	bool &loc_init_,
	int loc_offset_)
{
	point_d delta_t_zero;
	int sec_idx 			= -1;
	int loc_idx 			= -1;
	int loc_last_idx_mem 	= loc_last_idx_;
	bool mem 				= loc_init_;

	double tmpdlie =
			this->DLIE(
					edge_, points_[1], loc_idx, loc_last_idx_,
					loc_offset_, loc_init_);

	if (tmpdlie > 0.01)
	{
		if(mem || loc_last_idx_==LOC_INT-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_FAILURE;
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_FAILURE;
		}
	}

	// to fill up the line if loc_int is valid but tangent is not aligned to traj (at a curve)
	this->DecideSectorIntervalExt(
			edge_, points_[1], delta_t_zero, sec_idx, loc_idx);
	int tmp_idx = loc_idx*SEC_INT + sec_idx;
	edge_.sector_map[tmp_idx] = max(edge_.sector_map[tmp_idx], l2Norm(delta_t_zero));

	if (loc_idx>10) //offset because we shift back 10 at beginning
	{
		for(int l=loc_idx-2;l<loc_idx;l++)
		{
			this->DecideSectorIntervalExt(
					edge_, points_[1], delta_t_zero, sec_idx, l);
			int tmp_idx = l*SEC_INT + sec_idx;
			edge_.sector_map[tmp_idx] = max(edge_.sector_map[tmp_idx], l2Norm(delta_t_zero));
		}
	}

	loc_last_idx_ = loc_idx;

	return EXIT_SUCCESS;
}

int TrainSM::AdjustCurveExt(
	Graph *Graph_,
	vector<point_d> coeffs_,
	int integral_limit_,
	vector<point_d> pts_)
{
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double N 				= Graph_->getEdgeCounter(label1,label2,0);
	double total_len 		= 0;
	edge_tt edge_tmp 		= Graph_->getEdge(label1, label2, 0);
	edge_tt edge_tmp_mem	= edge_tmp;

	// [CURVE FIT]*************************************************************
	polyCurveLength(total_len, 0, integral_limit_, coeffs_);
	edge_tmp.total_len = total_len;
	for(int i=0;i<DEGREE;i++)
	{
		cx[i] = coeffs_[i].x;
		cy[i] = coeffs_[i].y;
		cz[i] = coeffs_[i].z;
	}
	int mem = 0;
	for(int i=0;i<LOC_INT;i++)
	{
		double lim[3], len[3];
		lim[0] = lim[1] = lim[2] = -1.0;
		len[0] =  (total_len/LOC_INT)* i;
		len[1] = ((total_len/LOC_INT)* i + (total_len/LOC_INT)*0.5);
		len[2] =  (total_len/LOC_INT)*(i+1);
		// ### HACK: resample points along curve and cal length.
		for(int ii=mem;ii<integral_limit_*100+1;ii++)
		{
			double tl = 0.0;
			double t  = (double)ii/100.0;
			polyCurveLength(tl, 0.0, t, coeffs_);
			if      (tl>=len[0] && lim[0]<0.0) {lim[0] = t;}
			else if (tl>=len[1] && lim[1]<0.0) {lim[1] = t;}
			else if (tl>=len[2] && lim[2]<0.0) {lim[2] = t; mem = ii; break;}
		}

		// Checking length values
		if(0)
		{
			double tmplen;
			polyCurveLength(tmplen, 0.0, lim[0], coeffs_);
			printf("length : %.4f %.4f %.4f ", len[0], tmplen, lim[0]);
			polyCurveLength(tmplen, 0.0, lim[1], coeffs_);
			printf("length : %.4f %.4f %.4f ", len[1], tmplen, lim[1]);
			polyCurveLength(tmplen, 0.0, lim[2], coeffs_);
			printf("length : %.4f %.4f %.4f \n", len[2], tmplen, lim[2]);
		}

		if (lim[2]<0) {lim[2] = integral_limit_;}

		point_d p_mid, p_tan, p_nor;
		p_mid.x = gsl_poly_eval (cx, DEGREE, lim[1]);
		p_mid.y = gsl_poly_eval (cy, DEGREE, lim[1]);
		p_mid.z = gsl_poly_eval (cz, DEGREE, lim[1]);

		edge_tmp.loc_mid  [i] = p_mid;
		edge_tmp.loc_start[i] = addPoint(p_mid , multiPoint(p_tan, lim[0]-lim[1]));
		edge_tmp.loc_end  [i] = addPoint(p_mid , multiPoint(p_tan, lim[2]-lim[1]));

		// N : counter
		if (N==0)
		{
			// at location interval 0
			if (i==0)
			{
				cal_tangent_normal(lim[1], p_tan, p_nor, coeffs_, DEGREE, true);
				edge_tmp.nor  	  [i] = multiPoint(p_nor , 1/l2Norm(p_nor));
				edge_tmp.tan  	  [i] = multiPoint(p_tan , 1/l2Norm(p_tan));
			}
			// rotates the normal at location interval 0
			else
			{
				cal_tangent_normal(lim[1], p_tan, p_nor, coeffs_, DEGREE, false);
				edge_tmp.tan  	  [i] = multiPoint(p_tan , 1/l2Norm(p_tan));
				vector<double> tmpRTI =
						transInv(
								rodriguezRot(
										edge_tmp.tan[0],
										edge_tmp.tan[i]));
				edge_tmp.nor[i].x =
						tmpRTI[0]*edge_tmp.nor[0].x +
						tmpRTI[1]*edge_tmp.nor[0].y +
						tmpRTI[2]*edge_tmp.nor[0].z;
				edge_tmp.nor[i].y =
						tmpRTI[3]*edge_tmp.nor[0].x +
						tmpRTI[4]*edge_tmp.nor[0].y +
						tmpRTI[5]*edge_tmp.nor[0].z;
				edge_tmp.nor[i].z =
						tmpRTI[6]*edge_tmp.nor[0].x +
						tmpRTI[7]*edge_tmp.nor[0].y +
						tmpRTI[8]*edge_tmp.nor[0].z;
			}
		}
		else
		{
			cal_tangent_normal(lim[1], p_tan, p_nor, coeffs_, DEGREE, false);
			edge_tmp.nor  	  [i] = multiPoint(p_nor , 1/l2Norm(p_nor));
			edge_tmp.tan  	  [i] = multiPoint(p_tan , 1/l2Norm(p_tan));
		}
	}
	// *************************************************************[CURVE FIT]

	// [ADJUSTMENT]************************************************************
	if (N>0)
	{
		// [AVERAGE]***********************************************************
		for(int l=0;l<LOC_INT;l++)
		{
			edge_tmp.loc_start[l] =
					addPoint(
							multiPoint(edge_tmp_mem.loc_start[l], N/(N+1)),
							multiPoint(edge_tmp.loc_start[l]	, 1/(N+1)));
			edge_tmp.loc_mid[l]	=
					addPoint(
							multiPoint(edge_tmp_mem.loc_mid[l]	, N/(N+1)),
							multiPoint(edge_tmp.loc_mid[l]		, 1/(N+1)));
			edge_tmp.loc_end[l]	=
					addPoint(
							multiPoint(edge_tmp_mem.loc_end[l]	, N/(N+1)),
							multiPoint(edge_tmp.loc_end[l] 	  	, 1/(N+1)));
			edge_tmp.tan[l] =
					addPoint(
							multiPoint(edge_tmp_mem.tan[l] 		, N/(N+1)),
							multiPoint(edge_tmp.tan[l] 	  	  	, 1/(N+1)));
			edge_tmp.tan[l]	=
					multiPoint(edge_tmp.tan[l], 1/l2Norm(edge_tmp.tan[l]));
			if (dotProduct(
					point2vector(edge_tmp_mem.tan[l]),
					point2vector(edge_tmp.tan[l])) > 0.99)
			{
				edge_tmp.nor[l] = edge_tmp_mem.nor[l];
			}
			else
			{
				vector<double> tmpRTI =
						transInv(
								rodriguezRot(
										edge_tmp_mem.tan[l],
										edge_tmp.tan[l]));
				edge_tmp.nor[l].x =
						tmpRTI[0]*edge_tmp_mem.nor[l].x +
						tmpRTI[1]*edge_tmp_mem.nor[l].y +
						tmpRTI[2]*edge_tmp_mem.nor[l].z;
				edge_tmp.nor[l].y =
						tmpRTI[3]*edge_tmp_mem.nor[l].x +
						tmpRTI[4]*edge_tmp_mem.nor[l].y +
						tmpRTI[5]*edge_tmp_mem.nor[l].z;
				edge_tmp.nor[l].z =
						tmpRTI[6]*edge_tmp_mem.nor[l].x +
						tmpRTI[7]*edge_tmp_mem.nor[l].y +
						tmpRTI[8]*edge_tmp_mem.nor[l].z;
			}
		}
		// ***********************************************************[AVERAGE]

		// [SECTOR MAP]********************************************************
		reshapeVector(edge_tmp.sector_map, LOC_INT*SEC_INT);
		vector<double> delta_t_mem; delta_t_mem.resize(3);

		for(int l=0;l<LOC_INT;l++)
		{
			vector<point_d> p_tmp; int res;
			for(int s=0;s<SEC_INT;s++)
			{
				vector<int> ind_loc;
				point_d tmpN, p_old[3];
				// [OLD POINT]*************************************************
				tmpN = rodriguezVec(
								2*M_PI*fmod((s+0.5),(double)SEC_INT)/SEC_INT,
								edge_tmp_mem.tan[l],
								edge_tmp_mem.nor[l]);
				p_old[0] =
						addPoint(
								edge_tmp_mem.loc_start[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				p_old[1] =
						addPoint(
								edge_tmp_mem.loc_mid[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				p_old[2] =
						addPoint(
								edge_tmp_mem.loc_end[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				// *************************************************[OLD POINT]
				for(int ii=1;ii<2;ii++)
				{
					vector<point_d> points_tmp = {p_old[ii],p_old[ii]};
					bool tmp_init = false;
					int last, curr; //##TODO
					if (l<20) 	{ last = 0; 	curr = 0; }
					else 		{ last = l-20; 	curr = l-20; }
					res = this->AdjustCurve(edge_tmp, points_tmp, last, tmp_init, 40);
				}
				p_tmp.push_back(p_old[1]);
			} //s
		} //l
		// ********************************************************[SECTOR MAP]
	}
	// ************************************************************[ADJUSTMENT]

	Graph_->setEdge(label1, label2, 0, edge_tmp);
	printer(19);
	return EXIT_SUCCESS;
}

int TrainSM::FitSectorMap(
	edge_tt &edge_,
	vector<point_d> &points_,
	int &loc_last_idx_,
	int loc_offset_,
	bool &loc_init_)
{
	int sec_idx 			= -1;
	int loc_idx 			= -1;
	int loc_last_idx_mem 	= loc_last_idx_;
	bool mem 				= loc_init_;
	point_d delta_t, delta_t_zero;

	// to deal with cases where the beginning is out of the sectormap
	// required for the fitting part only
	// also for the ending

	double tmp =
			this->DLIE(
					edge_, points_[1], loc_idx, loc_last_idx_, loc_offset_,
					loc_init_);

	if (tmp > 0.01)
	{
		if(mem || loc_last_idx_==LOC_INT-1)
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
		else
		{
			loc_last_idx_ = loc_last_idx_mem;
			return EXIT_SUCCESS; //#TODO should be failure
		}
	}

	// find the delta_t
	this->DecideSectorIntervalExt(
			edge_, points_[1], delta_t, sec_idx, loc_idx);


	// to fill up the line if loc_int is valid but tangent is not aligned to traj (at a curve)
	double min_dist 	= 10.0;
	double delta_t_min	= 0.0;
	int min_idx 		= 0;

//	if (loc_idx==loc_last_idx_)
	{
		min_idx		= loc_idx*SEC_INT + sec_idx;
		min_dist	= l2Norm(minusPoint(points_[1], edge_.loc_mid[loc_idx]));
		delta_t_min	= l2Norm(delta_t);
	}
//	else if (loc_idx>loc_last_idx_)
//	{
//		for(int ll=loc_last_idx_+1;ll<loc_idx+1;ll++)
//		{
//			decideSectorIntervalExt(
//					edge_, points_[1], delta_t_zero, sec_idx, ll);
//			int tmp_idx = ll*SEC_INT + sec_idx;
//			if (min_(l2Norm(minusPoint(points_[1], edge_.loc_mid[ll])),
//					 min_dist))
//			{
//				min_dist	= l2Norm(minusPoint(points_[1], edge_.loc_mid[ll]));
//				delta_t_min	= l2Norm(delta_t_zero);
//				min_idx 	= tmp_idx;
//			}
//		}
//	}
	//else
	// we do not consider going back for now

	if (loc_init_)
	{
		// if point starts before curve
		point_d proj_dir_tmp =
				multiPoint(
						edge_.tan[0],
						dotProduct(
								point2vector(
										minusPoint(
												points_[1],edge_.loc_mid[0])),
								point2vector(edge_.tan[0])));
		if (!vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(edge_.tan[0])))
		{
			loc_init_ = true;
			loc_last_idx_ = 0;
			return EXIT_SUCCESS;
		}
		else
		{
			if (loc_idx>0) loc_init_ = false;
			edge_.sector_map[min_idx] =
					max(edge_.sector_map[min_idx], delta_t_min);
			if (edge_.sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG
			loc_last_idx_ = min_idx/SEC_INT;
			return EXIT_SUCCESS;
		}
	}

	edge_.sector_map[min_idx] = max(edge_.sector_map[min_idx], delta_t_min);
	if (edge_.sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG change too big possible for new edge

	if ((min_idx/SEC_INT) - loc_last_idx_ > 1 && !mem)
	{
		for(int l=loc_last_idx_+1;l<(min_idx/SEC_INT);l++)
		{
			this->DecideSectorIntervalExt(
					edge_, points_[1], delta_t_zero, sec_idx, l);
			int tmp_idx = l*SEC_INT + sec_idx;
			edge_.sector_map[tmp_idx] = max(edge_.sector_map[tmp_idx], l2Norm(delta_t_zero));
			if (edge_.sector_map[min_idx]>0.9)  {return EXIT_FAILURE;} // TO BIGGGGGGGGG
		}
		loc_last_idx_ = (min_idx/SEC_INT);
	}
	else if ((min_idx/SEC_INT) - loc_last_idx_ == 1)
	{
		loc_last_idx_ = (min_idx/SEC_INT);
	}
	//else
	// we do not consider going back for now

	//cout << edge_.sector_map[min_idx] << endl; 0.449658

	return EXIT_SUCCESS;

}

int TrainSM::FitSectorMapInit(
	Graph *Graph_,
	vector<point_d> &points_,
	int loc_offset_)
{
	int loc_last_idx = 0;
	int loc_curr_idx = 0;
	double delta_t_mem = 0.0;
	bool init	= true;
	int offset	= loc_offset_;
	for(int i=1;i<points_.size();i++)
	{
		edge_tt edge_tmp = Graph_->getEdge(label1, label2, 0);
		vector<point_d> points_tmp {points_[i-1], points_[i]};
		if (init)	{offset = LOC_INT;}
		else		{offset = loc_offset_;}
		this->AdjustSectorMap(
				edge_tmp, points_tmp, loc_last_idx, loc_curr_idx,
				delta_t_mem, init, offset);
		Graph_->setEdge(label1, label2, 0, edge_tmp);
	}
	printer(22);
	return EXIT_SUCCESS;
}

int TrainSM::FitSectorMapExt(
	Graph *Graph_,
	vector<point_d> &points_,
	int loc_offset_)
{
	edge_tt edge_tmp = Graph_->getEdge(label1, label2, 0);

	// add the option to check if the curve is too much
	// how to handle with it ??? ####
	if(0)
	{
		vector<point_d> curve_mem;
		vector<double> x, curve;  curve.resize(points_.size());
		for(int i=0;i<edge_tmp.loc_mid.size();i++)
		{
			decideCurvature(edge_tmp.loc_mid[i], curve_mem, curve[i], 3);
			x.push_back(i);
		}
		plotData(x,curve);
	}

	bool flag_init		= true;
	int offset			= loc_offset_;
	int loc_last_idx	= 0;
	int res				= -1;
	for(int i=1;i<points_.size();i++)
	{
		edge_tmp = Graph_->getEdge(label1, label2, 0);

		if (flag_init)	{offset = LOC_INT;}
		else			{offset = loc_offset_;}

		vector<point_d> points_tmp {points_[i-1], points_[i]};

		res =
				this->FitSectorMap(
						edge_tmp, points_tmp, loc_last_idx, offset, flag_init);

		if (res==EXIT_FAILURE) {printer(31); return EXIT_FAILURE;}

		Graph_->setEdge(label1, label2, 0, edge_tmp);
	}

	Graph_->setEdge(label1, label2, 0, edge_tmp);

	printer(20);
	return EXIT_SUCCESS;
}

int TrainSM::UpdateSectorMap(
	Graph *Graph_,
	vector<point_d> points_avg_)
{
	// fitCurve()				: Fit the curve based on pure trajectory points.
	//							: Estimate points that are used to do fitting later.
	// adjustCurveExt()			: Obtain the tan and nor from the estimated points. (only for the first run)
	//							: Adjust the tan and nor from the estimated points.
	// fitSectorMapInit()		: Fit the estimated points to the sector map. (only for the first run)
	// fitSectorMapExt()			: Fit the estimated points to the sector map.
	// findSectorMapConstraint(): Check the constraints of  the sector map.
	// Graph_.setEdgeCounter()	: Increment the counter.

	vector<point_d> points_est, coeffs;
	int res = -1;

	if (Graph_->getEdgeCounter(label1,label2,0) == 0)
	{
		this->FitCurve(points_avg_, points_est, coeffs);
		this->AdjustCurveExt(Graph_, coeffs, points_avg_.size(), points_est);
		this->FitSectorMapInit(Graph_, points_avg_, 10);
		Graph_->setEdgeCounter(label1, label2, 0, 1);
	}
	else if (Graph_->getEdgeCounter(label1,label2,0) < 50)
	{
		this->FitCurve(points_avg_, points_est, coeffs);
		res = this->FitSectorMapExt(Graph_, points_avg_, 40);
		if (res==EXIT_FAILURE) {return EXIT_FAILURE;}
		//VISUALIZE
		if(0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, points_avg_, label_zero, color_code, true);
		}
		this->AdjustCurveExt(Graph_, coeffs, points_avg_.size(), points_avg_);
		Graph_->setEdgeCounter(label1, label2, 0, 1);
	}
	else
	{
		this->FitCurve(points_avg_, points_est, coeffs);
		res = this->FitSectorMapExt(Graph_, points_avg_, 20);
		if (res==EXIT_FAILURE) {return EXIT_FAILURE;}
		Graph_->setEdgeCounter(label1, label2, 0, 1);
	}

	//VISUALIZE
	if(0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, points_est, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int TrainSM::BuildSectorMap(
	Graph 						*Graph_,
	vector<vector<point_d> > 	pva_avg_,
	vector<int> 				contact_)
{
	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

	for(int i=0;i<pva_avg_.size();i++)
	{
		pts_avg.push_back(pva_avg_[i][0]);
		vel_avg.push_back(pva_avg_[i][1]);
	}

	// label1	 : start location
	// label2	 : goal location
	// label_idx : index of data point for label1
	label1 = label2 = label_idx = -1;

	for(int i=0;i<pts_avg.size();i++)
	{
		if (pts_avg[i].l >= 0)
		{
			// Initial location
			if	(label1 < 0) { label1 = pts_avg[i].l; continue; }
			else
			{
				// Check if location has changed
				if (label_idx > 0)
				{
					label2 = pts_avg[i].l;

					if (label2==label1)
					{
						label_idx = -1;
						continue;
					}

					if (i - label_idx > 10) // to prevent only a few points evaluated for curve, might not need this ###
					{
						vector<point_d> pts_avg_tmp(
								pts_avg.begin()+label_idx,
								pts_avg.begin()+i);
						vector<point_d> vel_avg_tmp(
								vel_avg.begin()+label_idx,
								vel_avg.begin()+i);

						this->UpdateSectorMap(Graph_, pts_avg_tmp);
						this->FindWindowConstraint(Graph_);

						//VISUALIZE
						if(0)
						{
							vector<point_d> point_zero; vector<string> label_zero;
							vector<vector<unsigned char> > color_code; colorCode(color_code);
							showConnection(Graph_, pts_avg_tmp, label_zero, color_code, true);
						}

					}

					label1 		= label2;
					label2 		= -1;
					label_idx 	= -1;
				}
			}
		}
		// saves the data number of initial location
		else { if (label1 >=0 && label_idx < 0) { label_idx = i; } }
	}
	return EXIT_SUCCESS;
}
