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

//int decideBoundary(
//	point_d 		&point1_,
//	point_d 		&point2_,
//	vector<point_d> centroids_)
//{
//	point2_.l = UNCLASSIFIED;
//	for(int ii=0;ii<centroids_.size();ii++)
//	{
//		double location_contact = 0.0;
//		point_d tmp_diff = minusPoint(point2_, centroids_[ii]);
//		if (
//				max_(
//						pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff)),
//						location_contact ))
//		{
//			location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
//			if (max_(location_contact, centroids_[ii].l))
//			{
////				double tmpvel = l2Norm(minusPoint(point2_,point1_));
////				if (tmpvel > 0.005) continue;
//				point2_.l = ii;
//			}
//		}
//	}
//	return EXIT_SUCCESS;
//}

//int decideBoundary_(
//	point_d 		&point1_,
//	point_d 		&point2_,
//	vector<point_d> centroids_,
//	vector<int>		surfaces_flag_,
//	vector<point_d> surfaces_,
//	vector<double>	surfaces_limit)
//{
//	point2_.l = UNCLASSIFIED;
//
//	for(int ii=0;ii<centroids_.size();ii++)
//	{
//		point_d tmp_diff = minusPoint(point2_, centroids_[ii]);
//		double location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
//		double boundary = sqrt(-log(centroids_[ii].l)*BOUNDARY_VAR*2);
//		if (max_(location_contact, centroids_[ii].l)) //|| pdfExp(0.005,0.0,(l2Norm(tmp_diff)-boundary))>0.75)
//		{
//			point2_.l = ii;
//
//			if ( surfaces_flag_[point2_.l] > 0)
//			{
//				if (!decideSurface(
//						point2_,
//						surfaces_[surfaces_flag_[point2_.l]-1],
//						surfaces_limit[surfaces_flag_[point2_.l]-1]))
//				{
//					point2_.l = UNCLASSIFIED;
//				}
//			}
//		}
//	}
//	return EXIT_SUCCESS;
//}

int checkBoundarySphere(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> centroids_)
{
	point2_.l = UNCLASSIFIED;
	for(int ii=0;ii<centroids_.size();ii++)
	{
		point_d tmp_diff = minusPoint(point2_, centroids_[ii]);
		double location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
		double boundary = sqrt(-log(centroids_[ii].l)*BOUNDARY_VAR*2);
		if (max_(location_contact, centroids_[ii].l)) //|| pdfExp(0.005,0.0,(l2Norm(tmp_diff)-boundary))>0.75)
		{
			point2_.l = ii;
		}
	}
	return EXIT_SUCCESS;
}

int checkBoundaryCuboid(
	point_d &point_,
	point_d box_min_,
	point_d box_max_)
{
	if(	point_.x < box_max_.x &&
		point_.y < box_max_.y &&
		point_.z < box_max_.z &&
		point_.x > box_min_.x &&
		point_.y > box_min_.y &&
		point_.z > box_min_.z )
//	if( point_.y < box_max_.y )
	{
		return EXIT_SUCCESS;
	}
	else
	{
		point_.l = UNCLASSIFIED;
		return EXIT_FAILURE;
	}
}

//int decideBoundaryPredict(
//	point_d 		&point1_,
//	point_d 		&point2_,
//	vector<point_d> centroids_,
//	vector<int>		surfaces_flag_,
//	vector<point_d> surfaces_,
//	vector<double>	surfaces_limit)
//{
//	point2_.l = UNCLASSIFIED;
//	for(int ii=0;ii<centroids_.size();ii++)
//	{
//		point_d tmp_diff = minusPoint(point2_, centroids_[ii]);
//		double location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
//		double boundary = sqrt(-log(centroids_[ii].l)*BOUNDARY_VAR*2);
//		if (max_(location_contact, centroids_[ii].l))// || pdfExp(0.005,0.0,(l2Norm(tmp_diff)-boundary))>0.8)
//		{
//			point2_.l = ii;
//			if (surfaces_flag_[point2_.l]>0)
//			{
//				if (!decideSurface(
//						point2_,
//						surfaces_[surfaces_flag_[point2_.l]-1],
//						surfaces_limit[surfaces_flag_[point2_.l]-1]))
//				{
//					point2_.l = UNCLASSIFIED;
//				}
//			}
//		}
//	}
//	return EXIT_SUCCESS;
//}

int decideBoundarySphere(
	point_d 		&point_,
	vector<point_d> centroids_)
{
	point_.l = UNCLASSIFIED;
	for(int ii=0;ii<centroids_.size();ii++)
	{
		point_d tmp_diff = minusPoint(point_, centroids_[ii]);
		double location_contact = pdfExp(BOUNDARY_VAR, 0.0, l2Norm(tmp_diff));
		double boundary = sqrt(-log(centroids_[ii].l)*BOUNDARY_VAR*2);
		if (max_(location_contact, centroids_[ii].l))// || pdfExp(0.005,0.0,(l2Norm(tmp_diff)-boundary))>0.8)
		{
			point_.l = ii;
		}
	}
	return EXIT_SUCCESS;
}

int decideBoundaryCuboid(
	point_d &point_,
	point_d box_min_,
	point_d box_max_)
{
	return checkBoundaryCuboid(point_, box_min_, box_max_);
}

int decideBoundaryClosest(
	point_d 				&point2_,
	vector<point_d> 		centroids_)
{
	point2_.l = UNCLASSIFIED;
	vector<double> tmp;
	for(int ii=0;ii<centroids_.size();ii++)
	{
		tmp.push_back(
				pdfExp(
						BOUNDARY_VAR,
						0.0,
						l2Norm(minusPoint(point2_, centroids_[ii]))));
	}
	point2_.l =
			distance(
					tmp.begin(),
					max_element(tmp.begin(), tmp.end()));
	return EXIT_SUCCESS;
}

double checkSurfaceDistance(
	point_d centroids_,
	point_d	surfaces_)
{
	return 	centroids_.x *  surfaces_.x +
			centroids_.y *  surfaces_.y +
			centroids_.z *  surfaces_.z -
							surfaces_.l;
}

bool decideSurface(
	point_d centroids_,
	point_d	surfaces_,
	double 	limit_)
{
	if(fabs(checkSurfaceDistance(centroids_,surfaces_))<limit_)
	{ return true; }
	else
	{ return false; }
}

double dLI(
	int &loc_idx_,
	int loc_last_idx_,
	point_d point_,
	vector<double> len_,
	vector<point_d> mid_,
	vector<point_d> tangent_,
	int loc_offset_,
	bool loc_init_)
{
	int idx_tmp;
	double d1,d2,d3,d4,d5,d6,d6min,d6min2; d1=d2=d3=d4=d5=d6=0.0; d6min = 10;
	point_d proj_dir_tmp, beg, mid, end;

	// Added an offset buffer to prevent the location prediction from jumping too much
	int idx1 = ( loc_last_idx_ < 0 ? 0 : loc_last_idx_ );
	int idx2 = ( idx1+loc_offset_ > LOC_INT ? LOC_INT : idx1+loc_offset_ );
	for(int l=idx1;l<idx2;l++)
	{
		mid = multiPoint(mid_[l],mid_[l].l);
		beg = minusPoint(mid, multiPoint(tangent_[l],len_[l]/2));
		end =   addPoint(mid, multiPoint(tangent_[l],len_[l]/2));
		proj_dir_tmp =
				multiPoint(
						tangent_[l],
						dotProduct(
								point2vector(minusPoint(point_, mid)),
								point2vector(tangent_[l])));
		d1 = l2Norm(minusPoint(beg, mid));
		d2 = l2Norm(minusPoint(end, mid));
		d3 = l2Norm(minusPoint(beg, addPoint(mid,proj_dir_tmp)));
		d4 = l2Norm(minusPoint(end, addPoint(mid,proj_dir_tmp)));
		d5 = l2Norm(minusPoint(beg, end));
		if (vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			if (l == idx1)	{d6min2 = d3-d5; d6 = d6min2;}
			if (d4<=d2 && (d3-d5) < 0.01) //### TODO small error deviation (deadzone)
			{
				d6 = d3-d5;
			}
		}
		else
		{
			if (l == idx1)	{d6min2 = d4-d5; d6 = d6min2;}
			if (d3<=d1 && (d4-d5) < 0.01)
			{
				d6 = d4-d5;
			}
		}

		// ignore the first loc_int so that we don't get "stuck" at curves
		if(l==idx1) continue;

		if (min_(d6,d6min))
		{
			d6min = d6;
			idx_tmp = l;
		}
		else
		{
			// breaks only when the location is larger than the current one
			if(d6min < 0.0001 && l>idx1) // inside
			{
				break;
			}
		}
	}

	// to prevent unknown locations at start and end
	if (d6min > 0.001 && loc_init_)
	{
		loc_idx_ = loc_last_idx_;
	}
	else
	{
		loc_idx_ = idx_tmp;
	}
	return d6min;
}

double dLIPredict(
	int &loc_idx_,
	int loc_last_idx_,
	point_d point_,
	vector<point_d> mid_,
	vector<double> len_,
	vector<point_d> tangent_,
	int loc_offset_,
	int loc_init_) //0 for true
{
	int idx_tmp=loc_last_idx_;
	double d1,d2,d3,d4,d5,d6,d6min,d6min2,min_dist;
	d1=d2=d3=d4=d5=d6=0.0; d6min = min_dist = 10;
	point_d proj_dir_tmp, beg, mid, end;

	// Added an offset buffer to prevent the location prediction from jumping too much
	int idx1 = ( loc_last_idx_-(loc_offset_/2) < 0 ? 0 : loc_last_idx_-(loc_offset_/2) );
	int idx2 = ( idx1+loc_offset_ > LOC_INT ? LOC_INT : idx1+loc_offset_ );
	for(int l=idx1;l<idx2;l++)
	{
		mid = multiPoint(mid_[l],mid_[l].l);
		beg = minusPoint(mid, multiPoint(tangent_[l],len_[l]/2));
		end =   addPoint(mid, multiPoint(tangent_[l],len_[l]/2));
		proj_dir_tmp =
				multiPoint(
						tangent_[l],
						dotProduct(
								point2vector(minusPoint(point_, mid)),
								point2vector(tangent_[l])));
		d1 = l2Norm(minusPoint(beg, mid));
		d2 = l2Norm(minusPoint(end, mid));
		d3 = l2Norm(minusPoint(beg, addPoint(mid,proj_dir_tmp)));
		d4 = l2Norm(minusPoint(end, addPoint(mid,proj_dir_tmp)));
		d5 = l2Norm(minusPoint(beg, end));
		if (vectorDirectionCheck(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			// ignore the first loc_int so that we don't get "stuck" at curves
			if (l == idx1)	{d6min2 = d3-d5; d6 = d6min2; continue;}
			if (d4<=d2 && (d3-d5) < 0.005) //### TODO small error deviation (deadzone)
			{
				d6 = d3-d5;
				if (min_(l2Norm(minusPoint(point_, mid_[l])), min_dist))
				{
					min_dist	= l2Norm(minusPoint(point_, mid_[l]));
					idx_tmp 	= l;
				}
			}
		}
		else
		{
			// ignore the first loc_int so that we don't get "stuck" at curves
			if (l == idx1)	{d6min2 = d4-d5; d6 = d6min2; continue;}
			if (d3<=d1 && (d4-d5) < 0.005)
			{
				d6 = d4-d5;
				if (min_(l2Norm(minusPoint(point_, mid_[l])), min_dist))
				{
					min_dist	= l2Norm(minusPoint(point_, mid_[l]));
					idx_tmp 	= l;
				}
			}
		}
	}

	//	// to prevent unknown locations at start and end
//	if (d6min > 0.001)
//	{
//		loc_idx_ = loc_last_idx_;
//	}
//	else
//	{
//		loc_idx_ = idx_tmp;
//	}

	if(loc_init_==0)
	{
		min_dist = 10;
		int tmp = 0;
		for(int i=idx1;i<idx2;i++)
		{
			if (min_(l2Norm(minusPoint(point_, mid_[i])), min_dist))
			{
				min_dist = l2Norm(minusPoint(point_, mid_[i]));
				tmp = i;
			}
		}

		if (tmp<idx_tmp)
			loc_idx_ = tmp;
		else
			loc_idx_ = idx_tmp;
	}
	else
	{
		loc_idx_ = idx_tmp;
	}

	return d6;
}

double decideLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	point_d point_,
	vector<point_d> beg_,
	vector<point_d> mid_,
	vector<point_d> end_,
	vector<point_d> tangent_,
	int loc_offset_,
	bool loc_init_)
{
	int idx_tmp;
	double d1,d2,d3,d4,d5,d6,d6min; d1=d2=d3=d4=d5=d6=0.0; d6min = 10;
	point_d proj_dir_tmp;
	bool flag = true;

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
			if (l == idx1)	{d6 = d3-d5;}
			if (d4<=d2 && (d3-d5) < 0.01) //### TODO small error deviation (deadzone)
			{
				d6 = d3-d5;
			}
		}
		else
		{
			if (l == idx1)	{d6 = d4-d5;}
			if (d3<=d1 && (d4-d5) < 0.01)
			{
				d6 = d4-d5;
			}
		}
		if (min_(fabs(d6),d6min))
		{
			d6min = fabs(d6);
			idx_tmp = l;
		}
	}

	loc_idxs_.clear();
	// to prevent unknown locations at start and end
	if (d6min > 0.005)
	{
		loc_idxs_.push_back(loc_last_idx_);
		loc_last_idx_ = loc_idxs_.back();
	}
	else
	{
		if (loc_init_ || idx_tmp==loc_last_idx_)
		{
			loc_idxs_.push_back(idx_tmp);
			loc_last_idx_ = loc_idxs_.back();
		}
		else
		{
			if (idx_tmp>loc_last_idx_)
			{
				for(int i=loc_last_idx_+1;i<idx_tmp+1;i++)
				{
					loc_idxs_.push_back(i);
				}
				loc_last_idx_ = idx_tmp;
			}
			else
			{
				for(int i=idx_tmp+1;i<loc_last_idx_+1;i++)
				{
					loc_idxs_.push_back(i);
				}
				loc_last_idx_ = idx_tmp;
			}
		}
	}
	return d6min;
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
	if (loc_idx_<0) {loc_idx_		= loc_last_idx_	; return d6;}
	else			{loc_last_idx_	= loc_idx_		; return d7;}

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
	//projection of point onto the tangent at mid
	point_d proj_dir =
			multiPoint(
					tangent_[loc_idx_],
					dotProduct(
							point2vector(
									minusPoint(
											point_,
											multiPoint(
													mid_[loc_idx_],
													mid_[loc_idx_].l))),
							point2vector(tangent_[loc_idx_])));
	delta_t_ =
			minusPoint(
					point_,
					addPoint(
							proj_dir,
							multiPoint(mid_[loc_idx_], mid_[loc_idx_].l)));
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
	{
		angle_tmp *= -1;
	}
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
	// HACK TODO
	if (curve_>0.4) {curve_ = 0.4;}
	return EXIT_SUCCESS;
}

int decideRateOfChangeOfDeltaT(
	point_d delta_t_,
	vector<point_d> &delta_t_mem_,
	double &dd_delta_t_,
	int num_points_)
{
	int s = delta_t_mem_.size()-1;
	if (s+1<num_points_)
	{
		delta_t_mem_.push_back(delta_t_);
		dd_delta_t_ = 0.0;
	}
	else
	{
		delta_t_mem_.erase(delta_t_mem_.begin());
		delta_t_mem_.push_back(delta_t_);
		dd_delta_t_ =
				l2Norm(
					minusPoint(
							minusPoint(
									delta_t_mem_[s  ],
									delta_t_mem_[s/2]),
							minusPoint(
									delta_t_mem_[s/2],
									delta_t_mem_[0  ])));
	}
	return EXIT_SUCCESS;
}

int folderSelect1(
	const struct dirent *entry)
{
	if (entry->d_name[2] == '_')	{return 1;}
	else							{return 0;}
}

int folderSelect2(
	const struct dirent *entry)
{
	size_t found_extension = string(entry->d_name).find(".");
	if ((int)found_extension == -1) {return 1;}
	else							{return 0;}
}

int fileSelect(
	const struct dirent *entry)
{
	size_t found_extension = string(entry->d_name).find(".txt");
	if ((int)found_extension == -1) {return 0;}
	else							{return 1;}
}
