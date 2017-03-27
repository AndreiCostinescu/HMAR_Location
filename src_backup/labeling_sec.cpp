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
			if(l2Norm(minusPoint(point_mem,points_[i]))>0.10 && !flag) //TODO : verify 0.10 ratio
			{
				continue;
			}
			points_out.push_back(points_[i]);
			point_mem = points_[i];
		}
		else
		{
			points_cluster.push_back(points_[i]);
			vel_tmp .push_back(l2Norm(vels_[i]));
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

	if(0) // Visualize by comparing both original and estimated points
	{
		vector<point_d> P = points_avg_;
		P.insert(P.end(),points_est_.begin(),points_est_.end());
		for(int i=0;i<points_avg_.size();i++)
		{
			P[i].l = 2;
		}
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<string> label_; label_.resize(2);
		vector<string> label_ref; label_ref.resize(2);
		vector<int> loc_idx_zero;
		showData(P, label_, label_ref, loc_idx_zero, color_code, true, false, false);
	}

	printf("# Fitting curve......SUCCESS\n");
	return EXIT_SUCCESS;
}

bool vectorDirectionCheck(
	vector<double> A,
	vector<double> B)
{
	bool out = true;
	for(int i=0;i<A.size();i++)
	{
		if (((A[i] >= 0) && (B[i] < 0)) || ((A[i] < 0) && (B[i] >= 0)))
		{
			out = false;
			break;
		}
	}
	return out;
}

int decideSectorInterval(
	int &sec_idx_,
	int loc_idx_,
	point_d &delta_t_,
	point_d point_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	vector<point_d> normal_)
{
	point_d proj_dir =
			multiPoint(
					tangent_[loc_idx_],
					dotProduct(
							point2vector(minusPoint(point_, mid_[loc_idx_])),
							point2vector(tangent_[loc_idx_])));
	delta_t_ = minusPoint(point_, addPoint(proj_dir, mid_[loc_idx_]));
	double angle_tmp =
			atan2(
					l2Norm(
							crossProduct(
									point2vector(delta_t_),
									point2vector(normal_[loc_idx_]))),
					dotProduct(
							point2vector(delta_t_),
							point2vector(normal_[loc_idx_])));
	if (!vectorDirectionCheck(
			crossProduct(
					point2vector(normal_[loc_idx_]),
					point2vector(multiPoint(delta_t_,1/l2Norm(delta_t_)))),
			point2vector(tangent_[loc_idx_])))
	{angle_tmp *= -1;}
	angle_tmp = fmod((2*M_PI + angle_tmp),(2*M_PI));
	sec_idx_ = ceil(angle_tmp*(SEC_INT/2)/M_PI) - 1 ;
	return EXIT_SUCCESS;
}

double decideLocationInterval(
	int &loc_idx_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int offset_)
{
	double d1,d2,d3,d4,d5,d6,d7;
	point_d proj_dir_tmp;
	int idx1 = (loc_last_idx_-offset_  < 0 ? 0 : loc_last_idx_-offset_);
	int idx2 =
			(offset_ < 0 ?
					LOC_INT :
					(loc_last_idx_+offset_ > LOC_INT ?
							LOC_INT :
							loc_last_idx_+offset_));
	bool mem = false;
	for(int l=idx1;l<idx2;l++)
	{
		proj_dir_tmp =
				multiPoint(
						tangent_[l],
						dotProduct(
								point2vector(minusPoint(point_,mid_[l])),
								point2vector(tangent_[l])));
		d1 = l2Norm(minusPoint(beg_[l],mid_[l]));
		d2 = l2Norm(minusPoint(end_[l],mid_[l]));
		d3 = l2Norm(minusPoint(beg_[l],addPoint(mid_[l],proj_dir_tmp)));
		d4 = l2Norm(minusPoint(end_[l],addPoint(mid_[l],proj_dir_tmp)));
		d5 = l2Norm(minusPoint(beg_[l],end_[l]));
		if (vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			mem = true;
			d7 = d3-d5;
			if (l == 0)						{d6 = d3-d5;}
			if (d4<=d2 && (d3-d5)<0.001)	{loc_idx_ = l; break;} //### small error deviation (deadzone)
		}
		else
		{
			//### helps to mitigate the deadzone
			if (mem) {loc_idx_ = (l-1<0 ? 0:l-1); break;}
			d7 = d4-d5;
			if (l == 0)						{d6 = d4-d5;}
			if (d3<=d1 && (d4-d5)<0.001) 	{loc_idx_ = l; break;}
		}
	}
	// to prevent unknown locations at start and end
	if (loc_idx_<0) {loc_idx_ = loc_last_idx_;} loc_last_idx_ = loc_idx_;
	if (loc_idx_<0) return d6;
	else 			return d7;
}

void decideLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_)
{
	loc_idxs_.clear();
	double d1,d2,d3,d4,d5;
	point_d proj_dir_tmp;
	// Added a buffer to prevent the location prediction from jumping too much
	int idx1 = ( loc_last_idx_ < 0 ? 0 : loc_last_idx_ );
	int idx2 = ( idx1+loc_offset_ > LOC_INT ? LOC_INT : idx1+loc_offset_ );
	for(int l=idx1;l<idx2;l++)
	{
		proj_dir_tmp =
				multiPoint(
						tangent_[l],
						dotProduct(
								point2vector(minusPoint(point_,mid_[l])),
								point2vector(tangent_[l])));
		d1 = l2Norm(minusPoint(beg_[l],mid_[l]));
		d2 = l2Norm(minusPoint(end_[l],mid_[l]));
		d3 = l2Norm(minusPoint(beg_[l],addPoint(mid_[l],proj_dir_tmp)));
		d4 = l2Norm(minusPoint(end_[l],addPoint(mid_[l],proj_dir_tmp)));
		d5 = l2Norm(minusPoint(beg_[l],end_[l]));
		if (vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			if (d4<=d2 && (d3-d5)<0.001) //### TODO small error deviation (deadzone)
			{
				if (loc_idxs_.empty()) 			{loc_idxs_.push_back(l);}
				else if (l-loc_idxs_.back()<2) 	{loc_idxs_.push_back(l);} //### TODO prevent the algo from cutting parts of a curve off
			}
		}
		else
		{
			if (d3<=d1 && (d4-d5)<0.001)
			{
				if (loc_idxs_.empty()) 			{loc_idxs_.push_back(l);}
				else if (l-loc_idxs_.back()<2) 	{loc_idxs_.push_back(l);}
			}
		}
	}
	// to prevent unknown locations at start and end
	if (loc_idxs_.size()<1) 	{loc_idxs_.push_back(loc_last_idx_);}
	if (loc_idxs_.size() == 1) 	{loc_last_idx_ = loc_idxs_[0];}
	else						{loc_last_idx_ = loc_idxs_[1];}
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
		decideLocationInterval(
				loc_idxs, loc_last_idx_, point_,
				beg_, mid_, end_, tangent_, loc_offset_);
		for(int ll=0;ll<loc_idxs.size();ll++)
		{
			decideSectorInterval(
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
		decideLocationInterval(
				loc_idx, loc_last_idx_, point_,
				beg_, mid_, end_, tangent_,loc_offset_);
		decideSectorInterval(
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
		len[0] =  (total_len/LOC_INT)*i;
		len[1] = ((total_len/LOC_INT)*i + (total_len/LOC_INT)*0.5);
		len[2] =  (total_len/LOC_INT)*(i+1);
		// ### HACK: resample points along curve and cal length.
		for(int ii=mem;ii<integral_limit_*100 + 1;ii++)
		{
			double tmplen 	= 0.0;
			double t 		= (double)ii/100.0;
			polyCurveLength(tmplen, 0.0, t, coeffs_);
			if      (tmplen>=len[0] && lim[0]<0.0) {lim[0] = t;}
			else if (tmplen>=len[1] && lim[1]<0.0) {lim[1] = t;}
			else if (tmplen>=len[2] && lim[2]<0.0) {lim[2] = t; mem = ii; break;}
		}

		if(0) // Checking values of length
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

		if (N==0) // counter
		{
			if (i==0) // location 0
			{
				cal_tangent_normal(lim[1], p_tan, p_nor, coeffs_, DEGREE, true);
				edge_tmp.nor  	  [i] = multiPoint(p_nor , 1/l2Norm(p_nor));
				edge_tmp.tan  	  [i] = multiPoint(p_tan , 1/l2Norm(p_tan));
			}
			else // rotates the normal at location 0
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
				point_d tmpN, pb_old, pm_old, pe_old;
				// [OLD POINT]*************************************************
				tmpN = rodriguezVec(
								2*M_PI*fmod((s+0.5),(double)SEC_INT)/SEC_INT,
								edge_tmp_mem.tan[l],
								edge_tmp_mem.nor[l]);
				pb_old =
						addPoint(
								edge_tmp_mem.loc_start[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				pm_old =
						addPoint(
								edge_tmp_mem.loc_mid[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				pe_old =
						addPoint(
								edge_tmp_mem.loc_end[l],
								multiPoint(
										tmpN,
										edge_tmp_mem.sector_map[l*SEC_INT+s]));
				// *************************************************[OLD POINT]
				adjustSectorMap(
						sector_map_new,
						loc_last[0],
						pb_old,
						edge_tmp.loc_start,
						edge_tmp.loc_mid,
						edge_tmp.loc_end,
						edge_tmp.tan,
						edge_tmp.nor,
						2,
						true);
				adjustSectorMap(
						sector_map_new,
						loc_last[1],
						pm_old,
						edge_tmp.loc_start,
						edge_tmp.loc_mid,
						edge_tmp.loc_end,
						edge_tmp.tan,
						edge_tmp.nor,
						2,
						true);
				adjustSectorMap(
						sector_map_new,
						loc_last[2],
						pe_old,
						edge_tmp.loc_start,
						edge_tmp.loc_mid,
						edge_tmp.loc_end,
						edge_tmp.tan,
						edge_tmp.nor,
						2,
						true);
			} //s
		} //l
		// ********************************************************[SECTOR MAP]
		edge_tmp.sector_map.clear();
		edge_tmp.sector_map = sector_map_new;
	}
	// ************************************************************[ADJUSTMENT]
	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printf("# Adjusting sector map......SUCCESS\n");
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
	printf("# Fitting points to sector map......SUCCESS\n");
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
	for(int i=0;i<points_.size();i++)
	{
//		int tmp = loc_offset_;
//		if(i>=10)
//		{
//			double curv =
//					(l2Norm(minusPoint(points_[i-3],points_[i-5])) +
//					 l2Norm(minusPoint(points_[i]  ,points_[i-3]))    ) /
//					 l2Norm(minusPoint(points_[i]  ,points_[i-5]));
//			cout << i << curv << endl;
//			if (curv > 1.001)
//				tmp = 0;
//			else
//				tmp = loc_offset_;
//		}
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
	printf("# Fitting points to sector map......SUCCESS\n");
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
				edge_tmp.sector_map[i]>MAX_RANGE ? edge_tmp.sector_map[i]:0;
	}
	Graph_.setEdge(label1_, label2_, 0, edge_tmp);
	printf("# Checking constraint......SUCCESS\n");
	return EXIT_SUCCESS;
}

int updateSectorMap(
	Graph &Graph_,
	vector<point_d> points_avg_,
	int label1_,
	int label2_)
{
	vector<point_d> points_est, coeffs;

	if (Graph_.getEdgeCounter(label1_,label2_,0) == 0)
	{
		// Fit the curve based on pure trajectory points.
		// Estimate points that are used to do fitting later.
		fitCurve(points_avg_, points_est, coeffs);
		// Obtain the tan and nor from the estimated points.
		adjustCurve(Graph_, coeffs, points_avg_.size(), label1_, label2_);
		// Fit the estimated points (only for the first run) to the sector map.
		fitSectorMapInit(Graph_, points_est, label1_, label2_, 2, true);
		// Check the constraints of  the sector map.
		findSectorMapConstraint(Graph_, label1_, label2_);
		// Increment the counter.
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);

		//VISUALIZE
		if(0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, points_avg_, label_zero, color_code, true);
		}

	}
	else if (Graph_.getEdgeCounter(label1_,label2_,0) < 50)
	{
		// Fit the curve based on pure trajectory points.
		// Estimate points that are used to do fitting later.
		fitCurve(points_avg_, points_est, coeffs);
		// Fit the pure trajectory points to the sector map.
		fitSectorMap(Graph_, points_est, label1_, label2_, 10, true);
		// Adjust the tan and nor from the estimated points.
		adjustCurve(Graph_, coeffs, points_avg_.size(), label1_, label2_);
		// Check the constraints of  the sector map.
		findSectorMapConstraint(Graph_, label1_, label2_);
		// Increment the counter.
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);

		//VISUALIZE
		if(0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, points_avg_, label_zero, color_code, true);
		}

	}
	else
	{
		// Fit the curve based on pure trajectory points.
		// Estimate points that are used to do fitting later.
		fitCurve(points_avg_, points_est, coeffs);
		// Fit the pure trajectory points to the sector map.
		fitSectorMap(Graph_, points_est, label1_, label2_, 10, true);
		// Check the constraints of  the sector map.
		findSectorMapConstraint(Graph_, label1_, label2_);
		// Increment the counter.
		Graph_.setEdgeCounter(label1_, label2_, 0, 1);

		//VISUALIZE
		if(0)
		{
			vector<point_d> point_zero; vector<string> label_zero;
			vector<vector<unsigned char> > color_code; colorCode(color_code);
			showConnection(Graph_, points_avg_, label_zero, color_code, true);
		}

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

	int label1, label2, label_idx;
	label1 = label2 = label_idx = -1;
	for(int i=0;i<pts_avg.size();i++)
	{
		if (pts_avg[i].l >= 0)
		{
			// Initial location
			if (label1 < 0)
			{
				label1 = pts_avg[i].l;
				continue;
			}
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
