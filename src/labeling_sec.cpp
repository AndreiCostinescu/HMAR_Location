/*
 * labeling_sec.cpp
 *
 *  Created on: Mar 17, 2017
 *      Author: chen
 */

#include "labeling_sec.h"

// Keeps the original label and removes any clusters that it finds
int findMovementConstraint(
	Graph &Graph_,
	vector<point_d> &points_,
	vector<point_d> vels_,
	int label1_,
	int label2_)
{
	bool flag 	= true;
	point_d point_mem;
	vector<point_d> points_cluster; // ### TODO: can be extended to include multiple clusters
	vector<point_d> points_tmp, points_out, centroids;
	vector<double> vel_tmp;
	vector<int> contact;
	points_tmp = points_;
	clustering(points_tmp);
//	combineNearCluster(points_tmp, centroids, contact); ### TODO: can be extended to include multiple clusters
//	reshapeVector(points_cluster, centroids.size()); ### TODO: can be extended to include multiple clusters
	for(int i=0;i<points_tmp.size();i++)
	{
		if(points_tmp[i].l<0)
		{
			if(flag)
			{
				flag = false;
				point_mem = points_tmp[i];
			}
			if(l2Norm(minusPoint(point_mem,points_[i]))>0.20 && !flag) //TODO : verify 0.10 ratio
			{
				continue;
			}
			points_out.push_back(points_[i]);
			point_mem = points_[i];
		}
		else
		{
			points_cluster.push_back(points_[i]);
			vel_tmp.push_back(l2Norm(vels_[i]));
		}
	}
	points_.clear();
	points_ = points_out;

//	double det = 0.0;
//	vector<vector<double> > tmp;
//	tmp.push_back(
//			point2vector(
//					minusPoint(
//							points_cluster[1],
//							points_cluster[0])));
//	tmp.push_back(
//			point2vector(
//					minusPoint(
//							points_cluster[points_cluster.size()-1],
//							points_cluster[0])));
//	for(int i=2;i<points_cluster.size()-1;i++)
//	{
//		if(vel_tmp[i]>0.005) //TODO : verify vel_lim
//		{
//			tmp.push_back(
//					point2vector(
//							minusPoint(
//									points_cluster[i],
//									points_cluster[0])));
//			det = max(determinant(tmp),det);
//			tmp.pop_back();
//		}
//	}
//	vector<int> constraint;
//	Graph_.getEdgeMovementConstraint(label1_, label2_, 0, constraint);
//	if (det > 0.2) //TODO : verify determinant error
//	{
//		constraint[MOV_CONST_SURF] = 1;
//	}
//	Graph_.setEdgeMovementConstraint(label1_, label2_, 0, constraint);

	return EXIT_SUCCESS;
}

int fitCurve(
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

int decideSectorIntervalExt(
	int &sec_idx_,
	int loc_idx_,
	point_d &delta_t_,
	point_d point_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	vector<point_d> normal_)
{
	return
			decideSectorInterval(
					sec_idx_, loc_idx_, delta_t_,
					point_, mid_, tangent_, normal_);
}

double decideLocationIntervalExt(
	int &loc_idx_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int offset_)
{
	return
			decideLocationInterval(
					loc_idx_, loc_last_idx_, point_,
					beg_, mid_, end_, tangent_, offset_);
}

void decideLocationIntervalExt(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_)
{
	decideLocationInterval(
			loc_idxs_, loc_last_idx_, point_,
			beg_, mid_, end_, tangent_, loc_offset_);
}

int adjustSectorMap(
	vector<double> &sector_map_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	vector<point_d> normal_,
	int loc_offset_,
	bool multiple_locations_)
{
	point_d delta_t;
	int sec_idx = -1;
	if(multiple_locations_)
	{
		vector<int> loc_idxs;
		decideLocationIntervalExt(
				loc_idxs, loc_last_idx_, point_,
				beg_, mid_, end_, tangent_, loc_offset_);
		for(int ll=0;ll<loc_idxs.size();ll++)
		{
			decideSectorIntervalExt(
					sec_idx, loc_idxs[ll], delta_t,
					point_, mid_, tangent_, normal_);
			sector_map_[loc_idxs[ll]*SEC_INT + sec_idx] =
					max(
							sector_map_[loc_idxs[ll]*SEC_INT + sec_idx],
							l2Norm(delta_t));
		}
	}
	else
	{
		int loc_idx = -1;
		decideLocationIntervalExt(
				loc_idx, loc_last_idx_, point_,
				beg_, mid_, end_, tangent_,loc_offset_);
		decideSectorIntervalExt(
				sec_idx, loc_idx, delta_t,
				point_, mid_, tangent_, normal_);
		sector_map_[loc_idx*SEC_INT + sec_idx] =
				max(
						sector_map_[loc_idx*SEC_INT + sec_idx],
						l2Norm(delta_t));
	}
	return EXIT_SUCCESS;
}

int adjustCurve(
	Graph &Graph_,
	vector<point_d> coeffs_,
	int integral_limit_,
	int label1_,
	int label2_)
{
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double N 				= Graph_.getEdgeCounter(label1_,label2_,0);
	double total_len 		= 0;
	edge_tt edge_tmp 		= Graph_.getEdge(label1_, label2_, 0);
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
		// ***********************************************************[AVERAGE]
		// [SECTOR MAP]********************************************************
		vector<double> sector_map_new; sector_map_new.resize(SEC_INT*LOC_INT);
		vector<int> loc_last; loc_last.resize(3);
		for(int l=0;l<LOC_INT;l++)
		{
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
				for(int ii=0;ii<3;ii++)
				{
					adjustSectorMap(
							sector_map_new,
							loc_last[ii],
							p_old[ii],
							edge_tmp.loc_start,
							edge_tmp.loc_mid,
							edge_tmp.loc_end,
							edge_tmp.tan,
							edge_tmp.nor,
							5,
							true);
				}
			} //s
			for(int i=1;i<3;i++)
			{
				if (loc_last[i-1]>loc_last[i]) {loc_last[i] = loc_last[i-1];}
			}
//			cout << l << " " << loc_last[0] << " " << loc_last[1] << " " << loc_last[2] << endl;
		} //l
		// ********************************************************[SECTOR MAP]
		edge_tmp.sector_map.clear();
		edge_tmp.sector_map = sector_map_new;
	}
	// ************************************************************[ADJUSTMENT]
	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printer(19);
	return EXIT_SUCCESS;
}

int fitSectorMapInit(
	Graph &Graph_,
	vector<point_d> &points_,
	int label1_,
	int label2_,
	int loc_offset_,
	bool multiple_locations_)
{
	edge_tt edge_tmp = Graph_.getEdge(label1_, label2_, 0);
	int loc_last_idx = 0;
	vector<double> sector_map_new; sector_map_new.resize(SEC_INT*LOC_INT);
	for(int i=0;i<points_.size();i++)
	{
		adjustSectorMap(
				edge_tmp.sector_map,
				loc_last_idx,
				points_[i],
				edge_tmp.loc_start,
				edge_tmp.loc_mid,
				edge_tmp.loc_end,
				edge_tmp.tan,
				edge_tmp.nor,
				loc_offset_,
				multiple_locations_);
	}
	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printer(22);
	return EXIT_SUCCESS;
}

int fitSectorMap(
	Graph &Graph_,
	vector<point_d> &points_,
	int label1_,
	int label2_,
	int loc_offset_,
	bool multiple_locations_)
{
	edge_tt edge_tmp = Graph_.getEdge(label1_, label2_, 0);
	int loc_last_idx = 0;
	vector<double> sector_map_new; sector_map_new.resize(SEC_INT*LOC_INT);

//	vector<point_d> curve_mem;
//	vector<double> x, curve;  curve.resize(points_.size());
//	for(int i=0;i<edge_tmp.loc_mid.size();i++)
//	{
//		decideCurvature(edge_tmp.loc_mid[i], curve_mem, curve[i], 3);
//		x.push_back(i);
//	}
//	plotData(x,curve);

	int offset, last;
	for(int i=0;i<points_.size();i++)
	{
		offset = loc_offset_;
		last = loc_last_idx;
		adjustSectorMap(
				edge_tmp.sector_map,
				last,
				points_[i],
				edge_tmp.loc_start,
				edge_tmp.loc_mid,
				edge_tmp.loc_end,
				edge_tmp.tan,
				edge_tmp.nor,
				offset,
				multiple_locations_);
		loc_last_idx = last;
	}

	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printer(20);
	return EXIT_SUCCESS;
}

int findSectorMapConstraint(
	Graph &Graph_,
	int label1_,
	int label2_)
{
	edge_tt edge_tmp = Graph_.getEdge(label1_, label2_, 0);
	if (edge_tmp.sector_const.empty())
	{
		edge_tmp.sector_const.resize(LOC_INT*SEC_INT);
	}
	for(int i=0;i<SEC_INT*LOC_INT;i++)
	{
		edge_tmp.sector_const[i] =
				edge_tmp.sector_map[i] > MAX_RANGE ?
						edge_tmp.sector_map[i] : 0;
	}
	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printer(21);
	return EXIT_SUCCESS;
}

int updateSectorMap(
	Graph &Graph_,
	vector<point_d> points_avg_,
	int label1_,
	int label2_)
{
	// fitCurve()				: Fit the curve based on pure trajectory points.
	//							: Estimate points that are used to do fitting later.
	// adjustCurve()			: Obtain the tan and nor from the estimated points. (only for the first run)
	//							: Adjust the tan and nor from the estimated points.
	// fitSectorMapInit()		: Fit the estimated points to the sector map. (only for the first run)
	// fitSectorMap()			: Fit the estimated points to the sector map.
	// findSectorMapConstraint(): Check the constraints of  the sector map.
	// Graph_.setEdgeCounter()	: Increment the counter.

	vector<point_d> points_est, coeffs;

	if (Graph_.getEdgeCounter(label1_,label2_,0) == 0)
	{
		fitCurve(points_avg_, points_est, coeffs);
		adjustCurve(Graph_, coeffs, points_avg_.size(), label1_, label2_);
		fitSectorMapInit(Graph_, points_est, label1_, label2_, 10, true);
		findSectorMapConstraint(Graph_, label1_, label2_);
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);
	}
	else if (Graph_.getEdgeCounter(label1_,label2_,0) < 50)
	{
		fitCurve(points_avg_, points_est, coeffs);
		fitSectorMap(Graph_, points_est, label1_, label2_, 20, true);

		//VISUALIZE
		if(0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, points_avg_, label_zero, color_code, true);
		}

		adjustCurve(Graph_, coeffs, points_avg_.size(), label1_, label2_);
		findSectorMapConstraint(Graph_, label1_, label2_);
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);
	}
	else
	{
		fitCurve(points_avg_, points_est, coeffs);
		fitSectorMap(Graph_, points_est, label1_, label2_, 10, true);
		findSectorMapConstraint(Graph_, label1_, label2_);
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);
	}

	//VISUALIZE
	if(0)
	{
		vector<point_d> point_zero; vector<string> label_zero;
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		showConnection(Graph_, points_avg_, label_zero, color_code, true);
	}

	return EXIT_SUCCESS;
}

int findWindowConstraint(
	Graph &Graph_,
	int label1_,
	int label2_)
{
//	edge_tt edge_tmp = Graph_.getEdge(label1_, label2_, 0);
//	if (Graph_.getEdgeCounter(label1_, label2_, 0)>2)
//	{
//		vector<double> x,y,y_mem;
//		vector<double> ddt_tmp,x_tmp;
//
//		for(int l=0;l<LOC_INT;l++)
//		{
//			double max_val = 0.0;
//			for(int s=0;s<SEC_INT/2;s++)
//			{
//				double tmp =
//						edge_tmp.sector_map[l*SEC_INT+s] +
//						edge_tmp.sector_map[l*SEC_INT+s+SEC_INT/2];
//				max_val = max(tmp,max_val);
//			}
//			if (max_val<0.02) // TODO: the min constraint
//			{
//				vector<int> c;
//				Graph_.getEdgeMovementConstraint(label1_, label2_, 0, c);
//				c[MOV_CONST_CIRC] = 1;
//				Graph_.setEdgeMovementConstraint(label1_, label2_, 0, c);
//			}
//
////			x.push_back(l);
////			y_mem.push_back(max_val);
////			if (y_mem.size()<3)
////				y.push_back(max_val);
////			else
////				y.push_back(movingAverage(max_val, y_mem));
//		}
//
////		for(int n=2;n<y.size();n++)
////		{
////			x_tmp.push_back(n-2);
////			ddt_tmp.push_back(
////					y[n] -
////					y[n-1] -
////					y[n-1] +
////					y[n-2]);
////		}
////		plotData(x,y);
////		plotData(x_tmp,ddt_tmp);
//
//	}

	return EXIT_SUCCESS;
}

int findSectorMapChangeRate(
	Graph &Graph_,
	int label1_,
	int label2_)
{
	edge_tt edge_tmp = Graph_.getEdge(label1_, label2_, 0);
	if (Graph_.getEdgeCounter(label1_, label2_, 0)>2)
	{
		vector<double> diff_tmp; diff_tmp.resize(LOC_INT);
		for(int l=0;l<LOC_INT;l++)
		{
			double a = 0.0;
			for(int s=0;s<SEC_INT/2;s++)
			{
				double tmp =
						edge_tmp.sector_map[l*SEC_INT+s] +
						edge_tmp.sector_map[l*SEC_INT+s+SEC_INT/2];
				a = max(tmp, a);
			}
			diff_tmp[l] = a;
		}
		vector<double> ddt_tmp;
		for(int n=2;n<diff_tmp.size();n++)
		{
			ddt_tmp.push_back(
					diff_tmp[n] -
					diff_tmp[n-1] -
					diff_tmp[n-1] +
					diff_tmp[n-2]);
		}

	}

	return EXIT_SUCCESS;
}

int buildSectorMap(
	Graph 						&Graph_,
	vector<vector<point_d> > 	pva_avg_,
	vector<int> 				contact_)
{
	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

	vector<point_d> pts_avg, vel_avg;
	for(int i=0;i<pva_avg_.size();i++)
	{
		pts_avg.push_back(pva_avg_[i][0]);
		vel_avg.push_back(pva_avg_[i][0]);
	}

	// label1	: start location
	// label2	: goal location
	// label_idx: index of data point for label1
	int label1, label2, label_idx;
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
					if (i - label_idx > 5) // to prevent only a few points evaluated for curve
					{
						vector<point_d> pts_avg_tmp(
								pts_avg.begin()+label_idx,
								pts_avg.begin()+i);
						vector<point_d> vel_avg_tmp(
								vel_avg.begin()+label_idx,
								vel_avg.begin()+i);

						findMovementConstraint(
								Graph_,
								pts_avg_tmp,
								vel_avg_tmp,
								label1,
								label2);
						updateSectorMap(Graph_,
								pts_avg_tmp,
								label1,
								label2);
						findWindowConstraint(
								Graph_,
								label1,
								label2);

//						vector<point_d> point_zero; vector<string> label_zero;
//						vector<vector<unsigned char> > color_code; colorCode(color_code);
////						showConnection(Graph_, points_avg_tmp2, label_zero, color_code, true);
//						Graph G2 = Graph_;
//						// Fit the pure trajectory points to the sector map.
//						fitSectorMap(G2, points_avg_tmp2, label1, label2, false);
//						showConnection(G2, points_avg_tmp2, label_zero, color_code, true);

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
