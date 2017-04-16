/*
 * labeling.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#include "labeling_loc.h"

#define DELETE

bool flaggg = false;

int decideBoundaryExt(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> centroids_)
{
	return decideBoundary(point1_, point2_, centroids_);
}

int contactBoundary(
	vector<point_d> 		&points_,
	vector<point_d> 		&centroids_,
	vector<vector<double> > surfaces_eq_,
	vector<int> 			&surfaces_flag_,
	vector<double> 			&surfaces_limit_,
	bool 					learn_)
{
	for (int i=0;i<points_.size();i++)
	{
		if (learn_)
		{
			if (points_[i].l < 0.0) { continue; }

			if (surfaces_flag_[points_[i].l] > 0)
			{
				if ( !decideSurface(
							centroids_[points_[i].l],
							surfaces_eq_[surfaces_flag_[points_[i].l]-1],
							surfaces_limit_[surfaces_flag_[points_[i].l]-1]))
				{
					points_[i].l = UNCLASSIFIED;
					continue;
				}
			}

			centroids_[points_[i].l].l =
					min(
							pdfExp(
									BOUNDARY_VAR,
									0.0,
									l2Norm(
											minusPoint(
													points_[i],
													centroids_[points_[i].l]))),
							centroids_[points_[i].l].l);
			centroids_[points_[i].l].l =
					max(
							0.50,
							centroids_[points_[i].l].l);
			centroids_[points_[i].l].l =
					min(
							0.98,
							centroids_[points_[i].l].l);
		}
		else
		{
			if (i > 0)
			{
				decideBoundary_(points_[i-1], points_[i], centroids_, surfaces_flag_, surfaces_eq_, surfaces_limit_);
//				decideBoundaryExt(points_[i-1], points_[i], centroids_);
			}
			else
			{
				decideBoundary_(points_[i], points_[i], centroids_, surfaces_flag_, surfaces_eq_, surfaces_limit_);
//				decideBoundaryExt(points_[i],   points_[i], centroids_);
			}

			if (points_[i].l < 0.0) { continue; }

			if (surfaces_flag_[points_[i].l] > 0)
			{
				if ( !decideSurface(
							centroids_[points_[i].l],
							surfaces_eq_[surfaces_flag_[points_[i].l]-1],
							surfaces_limit_[surfaces_flag_[points_[i].l]-1]))
				{
					points_[i].l = UNCLASSIFIED;
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

int contactCheck(
	vector<point_d> &points_,
	vector<point_d> &centroids_,
	vector<int> centroids_flag_,
	vector<point_d> surfaces_,
	vector<vector<double> > surfaces_eq_,
	vector<int> &surface_flag_,
	vector<double> &surfaces_limit_)
{
	for(int i=0;i<points_.size();i++)
	{
		if (points_[i].l < 0) continue;

//		if (centroids_flag_[points_[i].l] == 0) continue;

		for(int ii=0;ii<surfaces_.size();ii++)
		{
			// surface already known
			if (surface_flag_[points_[i].l] > 0) continue;

//			if (points_[i].l == 1)
//			{
//			cout << "SURFACE : " << fabs(points_[i].x * surfaces_eq_[ii][0] +
//					points_[i].y * surfaces_eq_[ii][1] +
//					points_[i].z * surfaces_eq_[ii][2] -
//					surfaces_eq_[ii][3]) <<endl;
//			cout << "DISTANCE : " << l2Norm(minusPoint(surfaces_[ii],centroids_[points_[i].l])) << endl;
//			}

			if (decideSurface(points_[i],surfaces_eq_[ii],surfaces_limit_[ii]) &&
					l2Norm(minusPoint(surfaces_[ii],centroids_[points_[i].l]))<0.2)
			{
				surface_flag_[points_[i].l] = ii+1;
			}
		}
	}

	for(int i=0;i<points_.size();i++)
	{
		if (points_[i].l < 0) continue;

//		if (centroids_flag_[points_[i].l] == 0) continue;

		// no surface
		if (surface_flag_[points_[i].l]==0) continue;

		if (!decideSurface(points_[i], surfaces_eq_[surface_flag_[points_[i].l]-1], surfaces_limit_[surface_flag_[points_[i].l]-1]))
		{
			points_[i].l = UNCLASSIFIED;
		}
	}

	return EXIT_SUCCESS;
}

int clusteringExt(
	vector<point_d> 		&points_,
	vector<point_d> 		&centroids_,
	vector<int> 			&centroids_flag_,
	vector<int> 			contact_,
	vector<point_d> 		surfaces_,
	vector<vector<double> > surfaces_eq_,
	vector<int> 			&surfaces_flag_,
	vector<double> 			&surfaces_limit_)
{
	vector<vector<unsigned char> > color_code; colorCode(color_code);
	vector<int> loc_idx_zero;

	clustering(points_, DBSCAN_EPS, DBSCAN_MIN);

	combineNearCluster(points_, centroids_, centroids_flag_, contact_);
	centroids_flag_[0] = 0; // hack TODO
	printer(14);

//	vector<string> goal_action, m; goal_action.resize(10);
//	showData(
//			points_, goal_action, m,
//			loc_idx_zero, color_code, true, false, false);

	reshapeVector(surfaces_flag_, centroids_flag_.size());

	// Visualize
	if (flaggg)
	{
		cout << centroids_.size()<<endl;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	contactCheck(
			points_, centroids_, centroids_flag_, surfaces_, surfaces_eq_,
			surfaces_flag_, surfaces_limit_);
	printer(40);

	// Visualize
	if (flaggg)
	{
		cout << centroids_.size()<<endl;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	contactBoundary(points_, centroids_, surfaces_eq_, surfaces_flag_, surfaces_limit_, true);

	// Visualize
	if (flaggg)
	{
		cout << centroids_.size()<<endl;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	contactBoundary(points_, centroids_, surfaces_eq_, surfaces_flag_, surfaces_limit_, false);

	// Visualize
	if (flaggg)
	{
		cout << centroids_.size()<<endl;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	return EXIT_SUCCESS;
}

int buildLocationArea(
	Graph 						*Graph_,
	vector<vector<point_d> > 	&pos_vel_acc_,
	vector<int> 				contact_,
	bool 						flag_)
{
//	flaggg = flag_;
	// [VARIABLES]*************************************************************
	bool 			flag = false;
	vector<point_d> points_avg;
	vector<point_d> locations;
	vector<string>  goal_action;
	vector<int> 	loc_idx_zero;

	vector<int> 	locations_flag;
	vector<int> 	surfaces_flag;
	vector<double> 	surfaces_limit = Graph_->getSurfaceLimit();

	vector<vector<unsigned char> > color_code; colorCode(color_code);
	// *************************************************************[VARIABLES]

	// Gathering points
	for(int i=0;i<pos_vel_acc_.size();i++)
	{
		points_avg.push_back(pos_vel_acc_[i][0]);
	}

	// Graph is empty
	if (Graph_->getNumberOfNodes()==0)
	{
		clusteringExt(
				points_avg, locations, locations_flag, contact_,
				Graph_->getSurface(), Graph_->getSurfaceEq(), surfaces_flag,
				surfaces_limit);

		reshapeVector(goal_action, locations.size());
		printer(15);
		showData(
				points_avg, goal_action, Graph_->getActionLabel(),
				loc_idx_zero, color_code, true, true, false);
		for(int i=0;i<locations.size();i++)
		{
			node_tt node_tmp;
			node_tmp.name 		= goal_action[i];
			node_tmp.index 		= i;
			node_tmp.surface 	= surfaces_flag[i];
			node_tmp.contact 	= locations_flag[i];
			node_tmp.centroid 	= locations[i];
			Graph_->setNode(node_tmp);
			Graph_->addEmptyEdgeForNewNode(node_tmp.index);
		}
	}
	else
	{
		// Graph is not empty
		for(int i=0;i<Graph_->getNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(i, node_tmp);
			goal_action.push_back(node_tmp.name);
			locations.push_back(node_tmp.centroid);
		}

		vector<point_d> locations_; // new ones
		clusteringExt(
				points_avg, locations_, locations_flag, contact_,
				Graph_->getSurface(), Graph_->getSurfaceEq(), surfaces_flag,
				surfaces_limit);
		reshapeVector(goal_action, locations_.size());
		printer(15);

//		cout << surfaces_flag[0] << " ";
//		cout << surfaces_flag[1] << " ";
//		cout << surfaces_flag[2] << " ";
//		cout << surfaces_flag[3] << " ";
//		cout << surfaces_flag[4] << " ";
//		cout << goal_action.size() << " ";
//		cout << Graph_->getActionLabel().size() << endl;

		// when new label is present
		if (flag_)
		{
			showData(
					points_avg, goal_action, Graph_->getActionLabel(),
					loc_idx_zero, color_code, true, true, false);
		}

		// add/adjust locations
		for(int i=0;i<locations_.size();i++)
		{
			for(int ii=0;ii<locations.size();ii++)
			{
				// adjust old LA
				if (
						l2Norm(
								minusPoint(
										locations[ii],
										locations_[i])) < CLUSTER_LIMIT*1.5)
				{
					node_tt node_tmp = {};
					Graph_->getNode(ii, node_tmp);
					double tmp1 = locations_[i].l;
					tmp1 = sqrt(-log(tmp1)*BOUNDARY_VAR*2);
					point_d loc_tmp =
							minusPoint(
									node_tmp.centroid,
									locations_[i]);
					tmp1 += l2Norm(loc_tmp);
					locations[ii].l =
							pdfExp(
									BOUNDARY_VAR,
									0.0,
									tmp1);
//					point_d loc_tmp =
//							multiPoint(
//									addPoint(
//											node_tmp.centroid,
//											locations_[i]),
//									0.5);
//					tmp1 += l2Norm(minusPoint(locations_[i],loc_tmp));
//					tmp2 += l2Norm(minusPoint(node_tmp.centroid,loc_tmp));
//					locations[ii] = loc_tmp;
//					locations[ii].l =
//							pdfExp(
//									BOUNDARY_VAR,
//									0.0,
//									max(tmp1*0.75,tmp2*0.75));
//					cout << endl;
//					cout << locations_[ii].l << endl;
//					cout << tmp2 << endl;
//					cout << locations[ii].l << endl;
//					locations[ii].l =
//							max(
//									0.5,
//									pdfExp(
//											BOUNDARY_VAR,
//											0.0,
//											max(tmp1,tmp2)));
					node_tmp.centroid = locations[ii];
					Graph_->setNode(node_tmp);
					// modify label according to list
					for(int iii=0;iii<points_avg.size();iii++)
					{
						if (points_avg[iii].l == i)
						{
							points_avg[iii].l = ii;
						}
					}
					break;
				}
				// add new LA
				if (ii==locations.size()-1 && flag_)
				{
					flag = true;
					node_tt node_tmp 	= {};
					node_tmp.name 		= goal_action[i];
					node_tmp.index 		= Graph_->getNodeList().size();
					node_tmp.surface 	= surfaces_flag[i];
					node_tmp.contact 	= locations_flag[i];
					node_tmp.centroid 	= locations_[i];
					Graph_->setNode(node_tmp);
					Graph_->addEmptyEdgeForNewNode(node_tmp.index);
					// modify label according to list
					for(int iii=0;iii<points_avg.size();iii++)
					{
						if (points_avg[iii].l == i)
						{
							points_avg[iii].l = node_tmp.index;
						}
					}
					break;
				}
			}
		}
	}

	// putting in the labels
	for(int i=0;i<pos_vel_acc_.size();i++)
	{
		pos_vel_acc_[i][0].l = points_avg[i].l;
	}
	printer(16);
	printer(17);

	// Visualize
	if (0)
	{
		goal_action.clear();
		for(int i=0;i<Graph_->getNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->getNode(i, node_tmp);
			goal_action.push_back(node_tmp.name);
		}
		showData(
				points_avg, goal_action, Graph_->getActionLabel(),
				loc_idx_zero, color_code, true, false, false);
	}
	return EXIT_SUCCESS;
}
