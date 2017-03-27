/*
 * labeling.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#include "labeling.h"

//#define DELETE

// ============================================================================
// Mov, Loc Labels
// ============================================================================

void labelMovement(
	Graph &Graph_)
{
	vector<string> label; label.clear();
	label.push_back("MOVE");
	label.push_back("SLIDE");
	Graph_.updateMovLabel(label);
}

void labelSector(
	Graph &Graph_,
	vector<vector<point_t> > pos_vel_acc_avg_,
	vector<int> contact_,
	vector<int> file_eof_,
	vector<vector<unsigned char> > color_code_)
{
	// Graph_.getEdgeList() = [#loc*#loc -> #edges -> #loc*#sec]

//	generateSectorCurve(Graph_, pos_vel_acc_avg_, contact_, file_eof_);
	printf("Generating sectors......Complete\n");

	vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
	for(int i=0;i<pos_vel_acc_avg_.size();i++) point_zero.push_back(pos_vel_acc_avg_[i][0]);
	showConnection(point_zero, label_zero, Graph_, color_code_, true);
	printf("Viewing sector......Complete\n");

	fillLocationData(Graph_);
}

bool replaceLabel(
	vector<string> &label_)
{
	bool replace = false;

	printf("Saved labels...\n");
	for(int i=0;i<label_.size();i++)
	{
		printf("Label %d : %s\n", i, label_[i].c_str());
	}

	printf("Replace labels? [Y/N]\n");
	while(0)
	{
		string mystr2; getline (cin, mystr2);
		if(!strcmp(mystr2.c_str(),"Y"))
		{
			for(int i=0;i<label_.size();i++)
			{
				printf("Enter new label for MLabel %d : ",i);
				string mystr;
				getline (cin, mystr);
				if(!strcmp(mystr.c_str(),""))
				{
					printf("[WARNING] : Label has not been overwritten.\n");
				}
				else
				{
					printf("[WARNING] : Label has been overwritten. New label : %s\n", mystr.c_str());
					label_[i] = mystr;
					replace = true;
				}
			}
			printf("Replace labels? [Y/N]\n");
			string mystr3; getline (cin, mystr3);
			if(!strcmp(mystr3.c_str(),"Y"))
			{
				continue;
			}
			if(!strcmp(mystr3.c_str(),"N"))
			{
				printf("Labels have been changed.\n");
				break;
			}
		}
		if(!strcmp(mystr2.c_str(),"N"))
		{
			cout << "[WARNING] : Label has not been overwritten." << endl;
			break;
		}
	}
	return replace;
}

// ============================================================================
// Sector Curve
// ============================================================================

void cal_inverse_intergal(
	point_t t_,
	point_t &p_tan_,
	point_t &p_nor_,
	vector<point_t> coeff,
	int dim) //degree+1
{
	vector<point_t> coeff_diff(dim-1);
	for(int i=0;i<dim-1;i++)
	{
		coeff_diff[i].x = i * coeff[i+1].x;
		coeff_diff[i].y = i * coeff[i+1].y;
		coeff_diff[i].z = i * coeff[i+1].z;
	}
	vector<point_t> coeff_sqr(2*(dim-1)-1);
	int c=0;
	for(int i=0;i<dim-1;i++)
	{
		for(int ii=0;ii<dim-1;ii++)
		{
			coeff_sqr[c+ii].x = coeff[i].x * coeff[ii].x;
			coeff_sqr[c+ii].y = coeff[i].y * coeff[ii].y;
			coeff_sqr[c+ii].z = coeff[i].z * coeff[ii].z;
		}
		c++;
	}
	//...
}

bool checkDirection(
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

double determineLocationInterval(
	int &loc_idx_,
	int &loc_last_idx_,
	int loc_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_)
{
	double d1,d2,d3,d4,d5,d6,d7,d66,d77; d6 = d7 = d66 = d77 = 0.0;
	point_t proj_dir_tmp;
	int idx = (loc_last_idx_<0?0:loc_last_idx_);
//	int idx = 0;
	bool mem = false;
	for(int l=idx;l<loc_int_;l++)
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
		if (checkDirection(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			mem = true;
			if (l == 0)	d6 = d3-d5;
			d7 = d3-d5;
			d66 = d6; d77 = d7;
			if (d4<=d2 && (d3-d5)<0.001)
			{
				loc_idx_ = l; d6 = 0.0; break;
			} //### small error deviation (deadzone)
		}
		else
		{
			if (mem) //### helps to mitigate the deadzone
			{
				if (l-1 <= 0)	d6 = d66;
				d7 = d77;
				loc_idx_ = (l-1<0 ? 0:l-1);
				d6 = 0.0;
				break;
			}
			if (l == 0)	d6 = d4-d5;
			d7 = d4-d5;
			if (d3<=d1 && (d4-d5)<0.001)
			{
				loc_idx_ = l; d6 = 0.0; break;
			}
		}
	}
	// to prevent unknown locations at start and end
	if (loc_idx_<0) loc_idx_ = loc_last_idx_; loc_last_idx_ = loc_idx_;
	if (loc_idx_<0) return d6;
	else 			return d7;
}

void determineLocationInterval(
	vector<int> &loc_idxs_,
	int &loc_last_idx_,
	int loc_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_)
{
	loc_idxs_.clear();
	double d1,d2,d3,d4,d5;
	point_t proj_dir_tmp;
	// Added a buffer to prevent the location prediction from jumping too much
	int idx = (loc_last_idx_<0?0:loc_last_idx_);
	for(int l=idx;l<(idx+(loc_int_/4)>loc_int_?loc_int_:idx+(loc_int_/4));l++)
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
		if (checkDirection(
				point2vector(proj_dir_tmp),
				point2vector(tangent_[l])))
		{
			if (d4<=d2 && (d3-d5)<0.001) {loc_idxs_.push_back(l);} //### small error deviation (deadzone)
		}
		else
		{
			if (d3<=d1 && (d4-d5)<0.001) {loc_idxs_.push_back(l);}
		}
	}
	// to prevent unknown locations at start and end
	if (loc_idxs_.size()<1) loc_idxs_.push_back(loc_last_idx_); loc_last_idx_ = loc_idxs_.back();
}

void determineSectorInterval(
	int &sec_idx_,
	int loc_idx_,
	int sec_int_,
	point_t &delta_t_,
	point_t point_,
	vector<point_t> mid_,
	vector<point_t> tangent_,
	vector<point_t> normal_)
{
	point_t proj_dir =
			multiPoint(
					tangent_[loc_idx_],
					dotProduct(
							point2vector(
									minusPoint(
											point_,
											mid_[loc_idx_])),
							point2vector(tangent_[loc_idx_])));
	delta_t_ =
			minusPoint(
					point_,
					addPoint(proj_dir, mid_[loc_idx_]));
	double angle_tmp =
			atan2(
					l2Norm(
							crossProduct(
									point2vector(delta_t_),
									point2vector(normal_[loc_idx_]))),
					dotProduct(
							point2vector(delta_t_),
							point2vector(normal_[loc_idx_])));
	if (!checkDirection(
			crossProduct(
					point2vector(normal_[loc_idx_]),
					point2vector(multiPoint(delta_t_,1/l2Norm(delta_t_)))),
			point2vector(tangent_[loc_idx_])))
	{angle_tmp *= -1;}
	angle_tmp = fmod((2*M_PI + angle_tmp),(2*M_PI));
	sec_idx_ = ceil(angle_tmp*(sec_int_/2)/M_PI) - 1 ;
}

void determineSectorMap(
	vector<double> &sector_map_,
	int &loc_last_idx_,
	int loc_int_,
	int sec_int_,
	point_t point_,
	vector<point_t> beg_,
	vector<point_t> mid_,
	vector<point_t> end_,
	vector<point_t> tangent_,
	vector<point_t> normal_,
	bool adjust)
{
	point_t delta_t;
	int sec_idx = -1;
	if(adjust)
	{
		vector<int> loc_idxs;
		determineLocationInterval(
				loc_idxs, loc_last_idx_, loc_int_,
				point_, beg_, mid_, end_, tangent_);
		for(int ll=0;ll<loc_idxs.size();ll++)
		{
			determineSectorInterval(
					sec_idx, loc_idxs[ll], sec_int_, delta_t,
					point_, mid_, tangent_, normal_);
			int ind_ls  = loc_idxs[ll]*sec_int_ + sec_idx;
				if (loc_idxs[ll] < loc_int_ && sec_idx < sec_int_)
					sector_map_[ind_ls] =
							max(sector_map_[ind_ls], l2Norm(delta_t));
		}
	}
	else
	{
		int loc_idx = -1;
		determineLocationInterval(
				loc_idx, loc_last_idx_, loc_int_,
				point_, beg_, mid_, end_, tangent_);
		determineSectorInterval(
				sec_idx, loc_idx, sec_int_, delta_t,
				point_, mid_, tangent_, normal_);
		int ind_ls  = loc_idx*sec_int_ + sec_idx;
		if (loc_idx < loc_int_ && sec_idx < sec_int_)
			sector_map_[ind_ls] =
					max(sector_map_[ind_ls], l2Norm(delta_t));
	}
}

void fitSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg_,
	vector<point_t> &points_est,
	vector<point_t> &coeffs_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_)
{
	vector<data_t> data_old = Graph_.getEdgeList()[edge_xy_][0].data;
	vector<data_t> edge_data; edge_data.clear();
	vector<point_t> points_tmp, covs;
	data_t motion_data;
	for(int i=point1_idx_;i<point2_idx_;i++)
	{
		points_tmp.push_back(pva_avg_[i][0]);
		motion_data.pos = pva_avg_[i][0];
		motion_data.vel = pva_avg_[i][1];
		motion_data.acc = pva_avg_[i][2];
		edge_data.push_back(motion_data);
	}
	if (!data_old.empty())
	{
		if(edge_data.size()==0)
			cout << "[WARNING] : Data to extend edge is empty." << endl;
		else if(edge_data.size()==1)
			data_old.push_back(edge_data[0]);
		else
			data_old.insert(
					data_old.end(), edge_data.begin(), edge_data.end());
		Graph_.updateEdgeData(data_old, label1_, label2_, 0);
	}
	reshapeVector(points_est,(points_tmp.size())*1);
	polyCurveFitPoint(points_tmp, points_est, coeffs_, covs, true);
	vector<point_t> P = points_tmp;
	P.insert(P.end(),points_est.begin(),points_est.end());
	for(int i=0;i<points_tmp.size();i++)
	{
		P[i].cluster_id = 2;
	}

//	vector<vector<unsigned char> > color_code; colorCode(color_code);
//	vector<string> label_(2);
//	vector<int> loc_idx_zero;
//	showData(P, label_, loc_idx_zero, color_code, true, false, false);

}

void checkSectorCurve(
	Graph &Graph_,
	vector<point_t> &points_est,
	int edge_xy_,
	int label1_,
	int label2_)
{
	int sec_int 				= Graph_.getSectorPara().sec_int;
	int loc_int 				= Graph_.getSectorPara().loc_int;
	int loc_last_idx			= 0;
	double total_len 			= Graph_.getEdgeList()[edge_xy_][0].total_len;
	vector<double>  sector_map	= Graph_.getEdgeList()[edge_xy_][0].sector_map;
	vector<point_t> tan 		= Graph_.getEdgeList()[edge_xy_][0].tan;
	vector<point_t> nor 		= Graph_.getEdgeList()[edge_xy_][0].nor;
	vector<point_t> loc_beg		= Graph_.getEdgeList()[edge_xy_][0].loc_start;
	vector<point_t> loc_mid		= Graph_.getEdgeList()[edge_xy_][0].loc_mid;
	vector<point_t> loc_end		= Graph_.getEdgeList()[edge_xy_][0].loc_end;
	point_t delta_t;

	for(int ii=0;ii<points_est.size();ii++)
	{
		determineSectorMap(
				sector_map, loc_last_idx, loc_int, sec_int,
				points_est[ii], loc_beg, loc_mid, loc_end, tan, nor,
				true);
	}
	Graph_.updateEdgeSector(sector_map, label1_, label2_, 0);
}

void checkSectorCurveConstraint(
	Graph &Graph_,
	double max_range_,
	int edge_xy_,
	int label1_,
	int label2_)
{
	int sec_int = Graph_.getSectorPara().sec_int;
	int loc_int = Graph_.getSectorPara().loc_int;
	vector<double> sector_map =
			Graph_.getEdgeList()[edge_xy_][0].sector_map;
	vector<double> sector_const	=
			Graph_.getEdgeList()[edge_xy_][0].sector_const;
	for(int i=0;i<sec_int*loc_int;i++)
		sector_const[i] =
				sector_map[i] > max_range_ ? sector_map[i] : 0;
	Graph_.updateEdgeConst(sector_const, label1_, label2_, 0);
}

void adjustSectorCurve(
	Graph &Graph_,
	vector<point_t> &points_est,
	vector<point_t> coeffs_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_)
{
	int loc_int = Graph_.getSectorPara().loc_int;
	int sec_int = Graph_.getSectorPara().sec_int;
	double N 	= Graph_.getCounter(edge_xy_,0);
	double cx[DEGREE], cy[DEGREE], cz[DEGREE];
	double total_len 			= Graph_.getEdgeList()[edge_xy_][0].total_len;
	vector<point_t> loc_beg 	= Graph_.getEdgeList()[edge_xy_][0].loc_start;
	vector<point_t> loc_mid		= Graph_.getEdgeList()[edge_xy_][0].loc_mid;
	vector<point_t> loc_end		= Graph_.getEdgeList()[edge_xy_][0].loc_end;
	vector<point_t> tan 	= Graph_.getEdgeList()[edge_xy_][0].tan;
	vector<point_t> nor 		= Graph_.getEdgeList()[edge_xy_][0].nor;
	vector<double>  sector_map  = Graph_.getEdgeList()[edge_xy_][0].sector_map;
	vector<point_t> loc_beg_mem = loc_beg;
	vector<point_t> loc_mid_mem	= loc_mid;
	vector<point_t> loc_end_mem = loc_end;
	vector<point_t> tangent_mem = tan;
	vector<point_t> normal_mem  = nor;

	// [CURVE FIT]*************************************************************
	polyCurveLength(total_len, 0, point2_idx_-point1_idx_-1, coeffs_);
	for(int i=0;i<DEGREE;i++)
	{cx[i] = coeffs_[i].x; cy[i] = coeffs_[i].y; cz[i] = coeffs_[i].z;}
	for(int i=0;i<loc_int;i++)
	{
		double t_beg 	= -1;
		double t_mid  	= -1;
		double t_end  	= -1;
		double len_inc1 =  total_len*(i)  /loc_int;
		double len_inc2 = (total_len*(i)  /loc_int) +
						  (0.5*total_len  /loc_int);
		double len_inc3 =  total_len*(i+1)/loc_int;
		// ### Shortcut: resample points along curve and cal length.
		for(int ii=0;ii<(point2_idx_-point1_idx_-1)*100 + 1;ii++)
		{
			double tmplen 	= 0.0;
			double t 		= (double)ii/100;
			polyCurveLength(tmplen, 0, t, coeffs_);
			if      (tmplen>=len_inc1 && t_beg<0)
			{t_beg = t;}
			else if (tmplen>=len_inc2 && t_mid<0)
			{t_mid  = t;}
			else if (tmplen>=len_inc3 && t_end<0)
			{t_end  = t; break;}
		}
		if (t_end<0) { t_end = point2_idx_-point1_idx_-1; }
		point_t p_mid, p_tan, p_nor;
		p_mid.x = gsl_poly_eval (cx, DEGREE, t_mid);
		p_mid.y = gsl_poly_eval (cy, DEGREE, t_mid);
		p_mid.z = gsl_poly_eval (cz, DEGREE, t_mid);
		if (N>0)
		{
			cal_tangent_normal(t_mid, p_tan, p_nor, coeffs_, DEGREE, false);
		}
		else
		{
			cal_tangent_normal(t_mid, p_tan, p_nor, coeffs_, DEGREE, true);
		}
		loc_mid	[i] = p_mid;
		loc_beg [i] = addPoint( p_mid , multiPoint(p_tan, t_beg-t_mid) );
		loc_end [i] = addPoint( p_mid , multiPoint(p_tan, t_end-t_mid) );
		nor   	[i] = multiPoint( p_nor , 1/l2Norm(p_nor) );
		tan  	[i] = multiPoint( p_tan , 1/l2Norm(p_tan) );
	}
	// *************************************************************[CURVE FIT]

	// [ADJUSTMENT]************************************************************
	if (N>0)
	{
		// [AVERAGE]***********************************************************
		for(int l=0;l<loc_int;l++)
		{
			loc_beg[l] = addPoint( multiPoint( loc_beg_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_beg[l] 	  , 1/(N+1) ) );
			loc_mid[l] = addPoint( multiPoint( loc_mid_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_mid[l] 	  , 1/(N+1) ) );
			loc_end[l] = addPoint( multiPoint( loc_end_mem[l] , N/(N+1) ) ,
								   multiPoint( loc_end[l] 	  , 1/(N+1) ) );
			tan[l] = addPoint( multiPoint( tangent_mem[l] , N/(N+1) ) ,
								   multiPoint( tan[l] 	  , 1/(N+1) ) );
			tan[l] = multiPoint( tan[l], 1/l2Norm(tan[l]) );
			vector<double> tmpRTI =
					transInv(rodriguezRot(tangent_mem[l],tan[l]));
			nor[l].x = tmpRTI[0]*normal_mem[l].x +
						  tmpRTI[1]*normal_mem[l].y +
						  tmpRTI[2]*normal_mem[l].z;
			nor[l].y = tmpRTI[3]*normal_mem[l].x +
						  tmpRTI[4]*normal_mem[l].y +
						  tmpRTI[5]*normal_mem[l].z;
			nor[l].z = tmpRTI[6]*normal_mem[l].x +
						  tmpRTI[7]*normal_mem[l].y +
						  tmpRTI[8]*normal_mem[l].z;
		}
		// ***********************************************************[AVERAGE]
		// [SECTOR MAP]********************************************************
		vector<double> sector_map_new(sec_int*loc_int);
		vector<double> sector_map_new2(sec_int*loc_int);
		int ind_ls = 0;
		vector<int> loc_last(3);

		for(int l=0;l<loc_int;l++)
		{
			for(int s=0;s<sec_int;s++)
			{
				vector<int> ind_loc;
				int ind_sec = -1;
				point_t delta_t, tmpN, pb_old, pm_old, pe_old;
				// [OLD POINT]*************************************************
				tmpN = rodriguezVec(
								2*M_PI*fmod((s+0.5),(double)sec_int)/sec_int,
								tangent_mem[l],
								normal_mem[l]);
				pb_old = addPoint(
								loc_beg_mem[l],
								multiPoint(
										tmpN,
										sector_map[l*sec_int+s]));
				pm_old = addPoint(
								loc_mid_mem[l],
								multiPoint(
										tmpN,
										sector_map[l*sec_int+s]));
				pe_old = addPoint(
								loc_end_mem[l],
								multiPoint(
										tmpN,
										sector_map[l*sec_int+s]));
				// *************************************************[OLD POINT]

				determineSectorMap(sector_map_new, loc_last[0], loc_int, sec_int,
						pb_old, loc_beg, loc_mid, loc_end, tan, nor, true);
				determineSectorMap(sector_map_new, loc_last[1], loc_int, sec_int,
						pm_old, loc_beg, loc_mid, loc_end, tan, nor, true);
				determineSectorMap(sector_map_new, loc_last[2], loc_int, sec_int,
						pe_old, loc_beg, loc_mid, loc_end, tan, nor, true);
			} //s
		} //l
		// ********************************************************[SECTOR MAP]
		Graph_.updateEdgeSector(sector_map_new, label1_, label2_, 0);
	}
	// ************************************************************[ADJUSTMENT]

	Graph_.updateEdgeLocStartMidEnd(loc_beg, loc_mid, loc_end, label1_, label2_, 0);
	Graph_.updateEdgeLocDist(total_len, label1_, label2_, 0);
	Graph_.updateEdgeNormal(nor, label1_, label2_, 0);
	Graph_.updateEdgeTangent(tan, label1_, label2_, 0);
}

void updateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg_,
	int edge_xy_,
	int point1_idx_,
	int point2_idx_,
	int label1_,
	int label2_,
	double max_range_)
{
	vector<vector<unsigned char> > color_code; colorCode(color_code);

	vector<point_t> points_est, coeffs;

	if (Graph_.getCounter(edge_xy_,0) == 0)
	{

		fitSectorCurve(
				Graph_, pva_avg_, points_est, coeffs, edge_xy_,
				point1_idx_, point2_idx_, label1_, label2_);

		adjustSectorCurve(
				Graph_, points_est, coeffs, edge_xy_,
				point1_idx_, point2_idx_, label1_, label2_);

		checkSectorCurve(
				Graph_, points_est, edge_xy_, label1_, label2_);

		checkSectorCurveConstraint(
				Graph_, max_range_, edge_xy_, label1_, label2_);

		Graph_.incrementCounter(edge_xy_,0);
	}
	else if (Graph_.getCounter(edge_xy_,0) < 50)
	{
		fitSectorCurve(
				Graph_, pva_avg_, points_est, coeffs, edge_xy_,
				point1_idx_, point2_idx_, label1_, label2_);

		checkSectorCurve(
				Graph_, points_est, edge_xy_, label1_, label2_);

//		vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
//		for(int i=point1_idx_;i<point2_idx_;i++) point_zero.push_back(pva_avg_[i][0]);
//		showConnection(points_est, label_zero, Graph_, color_code, true);
//		printf("Viewing sector......Complete\n");

		adjustSectorCurve(
				Graph_, points_est, coeffs, edge_xy_,
				point1_idx_, point2_idx_, label1_, label2_);

		checkSectorCurveConstraint(
				Graph_, max_range_, edge_xy_, label1_, label2_);

		Graph_.incrementCounter(edge_xy_,0);
	}
	else
	{
		fitSectorCurve(
				Graph_, pva_avg_, points_est, coeffs, edge_xy_,
				point1_idx_, point2_idx_, label1_, label2_);

		checkSectorCurve(
				Graph_, points_est, edge_xy_, label1_, label2_);

		checkSectorCurveConstraint(
				Graph_, max_range_, edge_xy_, label1_, label2_);
	}

//	vector<point_t> point_zero; vector<string> label_zero; bool flag = false;
//	for(int i=point1_idx_;i<point2_idx_;i++) point_zero.push_back(pva_avg_[i][0]);
//	showConnection(points_est, label_zero, Graph_, color_code, true);
//	printf("Viewing sector......Complete\n");
}

void generateSectorCurve(
	Graph &Graph_,
	vector<vector<point_t> > pva_avg_,
	vector<int> contact_,
	vector<int> file_eof_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<point_t> locations(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].location;}

	int tmp_id1 = 0, tmp_id2 = -1, tmp_id3 = -1, file_num = 0, edge_xy = 0;
	bool flag_next 	= false;

	for(int p_idx=0;p_idx<pva_avg_.size();p_idx++)
	{
		if(p_idx==file_eof_[file_num]-1)
		{
			vector<vector<string> > data_;
			vector<vector<point_t> > pva_avg_tmp(p_idx-tmp_id1);

			for(int p_idx2=tmp_id1;p_idx2<p_idx;p_idx2++)
			{
				pva_avg_tmp[p_idx2-tmp_id1] = pva_avg_[p_idx2];
			}

//			labelLocation_(Graph_, pva_avg_tmp, contact_, data_, DBSCAN_EPS, DBSCAN_MIN);

			//						updateSectorCurve(
			//								Graph_, pva_avg, edge_xy,
			//								tmp_id3, i, tmp_id1, tmp_id2, max_range_);

			file_num++;
			tmp_id1 = p_idx+1; // from
		}
//		if (pva_avg[i][0].cluster_id >= 0)
//		{
//			// Initial location
//			if (tmp_id1 < 0)
//			{
//				tmp_id1 = pva_avg[i][0].cluster_id;
//				continue;
//			}
//			else
//			{
//				// Check if location has changed
//				if (flag_next)
//				{
//					tmp_id2 = pva_avg[i][0].cluster_id;
//					edge_xy = tmp_id1*locations.size() + tmp_id2;
//					if (i - tmp_id3 > 5) // to prevent only a few points evaluated for curve
//						updateSectorCurve(
//								Graph_, pva_avg, edge_xy,
//								tmp_id3, i, tmp_id1, tmp_id2, max_range_);
//					flag_next = false;
//					tmp_id1 = tmp_id2;
//					tmp_id2 = -1;
//					tmp_id3 = -1;
//				}
//			}
//		}
//		else
//		{
//			// saves the data number of initial location
//			if (tmp_id1 >=0 && tmp_id3 < 0)
//			{
//				tmp_id3 = i;
//				flag_next = true;
//			}
//		}
	}
}

void fillLocationData(
	Graph &Graph_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_loc = nodes.size();
	vector<point_t> locations(num_loc);
	for(int i=0;i<num_loc;i++) {locations[i] = nodes[i].location;}

	int sec_int = Graph_.getSectorPara().sec_int;
	int loc_int	= Graph_.getSectorPara().loc_int;
	int c = 0;
	point_t point; point.x=point.z=0.0; point.y=1.0;

	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	for(int i=0;i<Sqr(num_loc);i++)
	{
		if (i==c)
		{
			c+=(num_loc+1);
			continue;
		}
		for(int ii=0;ii<edges[i].size();ii++)
		{
			vector<point_t> tan = edges[i][ii].tan;
			if (tan[0].x!=0 &&
				tan[0].y!=0 &&
				tan[0].z!=0) break;
			vector<point_t> nor 	= edges[i][ii].nor;
			vector<point_t> loc_beg	= edges[i][ii].loc_start;
			vector<point_t> loc_mid	= edges[i][ii].loc_mid;
			vector<point_t> loc_end	= edges[i][ii].loc_end;
			point_t dis_t = minusPoint(locations[i%num_loc],
									   locations[i/num_loc]);
			point_t tan_t = multiPoint(dis_t,1/l2Norm(dis_t));
			point_t nor_t =
					vector2point(
							crossProduct(
									point2vector(tan_t),
									point2vector(point)));
			point_t beg_t =
					addPoint(
							locations[i/num_loc],
							multiPoint(tan_t, 0.10107676525)); // 0.10107676525 is reversed calculated from 0.60 boundary.
			point_t end_t =
					minusPoint(
							locations[i%num_loc],
							multiPoint(tan_t, 0.10107676525));
			double len_t = l2Norm(minusPoint(end_t,beg_t));
			for(int iii=0;iii<loc_int;iii++)
			{
				tan[iii] = tan_t;
				nor [iii] = nor_t;
				loc_beg[iii] = addPoint(beg_t,multiPoint(tan_t,iii*len_t/loc_int));
				loc_end[iii] = addPoint(beg_t,multiPoint(tan_t,(iii+1)*len_t/loc_int));
				loc_mid[iii] = multiPoint(addPoint(loc_beg[iii],loc_end[iii]),0.5);
			}
			Graph_.updateEdgeLocStartMidEnd(
					loc_beg, loc_mid, loc_end, i/num_loc, i%num_loc, ii);
			Graph_.updateEdgeTangent(tan,  i/num_loc, i%num_loc, ii);
			Graph_.updateEdgeNormal (nor,   i/num_loc, i%num_loc, ii);
		}
	}
}
