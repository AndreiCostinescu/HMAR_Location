/*
 * core.cpp
 *
 *  Created on: Mar 26, 2017
 *      Author: chen
 */

#include "core.h"


bool vectorDirectionCheck(
	vector<double> A,
	vector<double> B)
{
	for(int i=0;i<A.size();i++)
		if (((A[i] >= 0) && (B[i] < 0)) || ((A[i] < 0) && (B[i] >= 0)))
			return false;
	return true;
}

int decideBoundary(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> centroids_)
{
	point2_.l = UNCLASSIFIED;
	double location_contact = 0.0;
	for(int ii=0;ii<centroids_.size();ii++)
	{
		point_d tmp_diff = minusPoint(point2_, centroids_[ii]);
		if (
				max_(
						pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff)),
						location_contact ))
		{
			location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
			if (max_(location_contact, centroids_[ii].l))
			{
//				double tmpvel = l2Norm(minusPoint(point2_,point1_));
//				if (tmpvel > 0.005) continue;
				point2_.l = ii;
			}
		}
	}
	return EXIT_SUCCESS;
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
	vector<int> idx_tmp;
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
				if (idx_tmp.empty()) 			{idx_tmp.push_back(l);}
				else if (l-idx_tmp.back()<2) 	{idx_tmp.push_back(l);} //### TODO prevent the algo from cutting parts of a curve off
			}
		}
		else
		{
			if (d3<=d1 && (d4-d5)<0.001)
			{
				if (idx_tmp.empty()) 			{idx_tmp.push_back(l);}
				else if (l-idx_tmp.back()<2) 	{idx_tmp.push_back(l);}
			}
		}
	}

	loc_idxs_.clear();
	// to prevent unknown locations at start and end
	if (idx_tmp.size()<1)
	{
		loc_idxs_.push_back(loc_last_idx_);
	}
	else
	{
		for(int i=loc_last_idx_;i<idx_tmp.back()+1;i++)
		{
			loc_idxs_.push_back(i);
		}
		loc_idxs_.size() > 1 ? loc_last_idx_ = loc_idxs_.back() : loc_last_idx_ = loc_idxs_[0];
	}
//	if (loc_idxs_.size() == 1) 	{loc_last_idx_ = loc_idxs_[0];}
//	else						{(loc_idxs_[1] - loc_idxs_[0]>1) ? loc_last_idx_ = loc_idxs_[0] : loc_last_idx_ = loc_idxs_[1];}
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
	int idx1 = ( loc_last_idx_ < 0 ? 0 : loc_last_idx_ );
//	int idx1 = (loc_last_idx_-offset_  < 0 ? 0 : loc_last_idx_-offset_);
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
//			if (mem) {loc_idx_ = (l-1<0 ? 0:l-1); break;}
			d7 = d4-d5;
			if (l == 0)						{d6 = d4-d5;}
			if (d3<=d1 && (d4-d5)<0.001) 	{loc_idx_ = l; break;}
		}
	}
	// to prevent unknown locations at start and end
	if (loc_idx_<0) {loc_idx_		= loc_last_idx_	;}
	else			{loc_last_idx_	= loc_idx_		;}
	if (loc_idx_<0) return d6;
	else 			return d7;
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

int decideCurvature(
	point_d point_,
	vector<point_d> &curve_mem_,
	double &curve_,
	int num_points_)
{
	int s = curve_mem_.size()-1;
	if (s+1<num_points_)
	{
		curve_mem_.push_back(point_);
		curve_ = 0.0;
	}
	else
	{
		curve_mem_.erase(curve_mem_.begin());
		curve_mem_.push_back(point_);
		curve_ =
				(l2Norm(minusPoint(curve_mem_[0], 	curve_mem_[s/2])) +
				 l2Norm(minusPoint(curve_mem_[s/2],	curve_mem_[s]))) /
				 l2Norm(minusPoint(curve_mem_[0],	curve_mem_[s]));
		curve_ -= 1.0;
	}
	return EXIT_SUCCESS;

//	curve_ = 0.0;
//	int s = curve_mem_.size()-1;
//	if (s+1<num_points_)
//	{
//		curve_mem_.push_back(point_);
//	}
//	else
//	{
//		curve_mem_.erase(curve_mem_.begin());
//		curve_mem_.push_back(point_);
//		for(int i=1;i<s+1;i++)
//		{
//			curve_ += l2Norm(minusPoint(curve_mem_[i], curve_mem_[i-1]));
//		}
//		curve_ /= l2Norm(minusPoint(curve_mem_[s], curve_mem_[0]));
//		curve_ -= 1.0;
//	}
//	return EXIT_SUCCESS;
}
