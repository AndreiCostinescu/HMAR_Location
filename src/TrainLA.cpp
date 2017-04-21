/*
 * TrainLA.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "TrainLA.h"

TrainLA::TrainLA()
{
	colorCode(color_code);
}

TrainLA::~TrainLA() {
	// TODO Auto-generated destructor stub
}

void TrainLA::ClearLA()
{
	contact.clear();
	locations_flag.clear();
	surfaces_flag.clear();
	loc_idx_zero.clear(); // loc_idx_zero unused at all
	points_avg.clear();
	locations.clear();
	goal_action.clear();
	surfaces.clear();
	surfaces_eq.clear();
	surfaces_limit.clear();
}

int TrainLA::DecideBoundaryExt(
	point_d 		&point1_,
	point_d 		&point2_,
	vector<point_d> centroids_)
{
	return decideBoundary(point1_, point2_, centroids_);
}

int TrainLA::ContactBoundary(
	vector<point_d> &centroids_,
	bool learn_)
{
	for (int i=0;i<points_avg.size();i++)
	{
		if (learn_)
		{
			if (points_avg[i].l < 0.0) { continue; }

			if (surfaces_flag[points_avg[i].l] > 0)
			{
				if ( !decideSurface(
							centroids_[points_avg[i].l],
							surfaces_eq[surfaces_flag[points_avg[i].l]-1],
							surfaces_limit[surfaces_flag[points_avg[i].l]-1]))
				{
					points_avg[i].l = UNCLASSIFIED;
					continue;
				}
			}

			centroids_[points_avg[i].l].l =
					min(pdfExp(
								BOUNDARY_VAR,
								0.0,
								l2Norm(
										minusPoint(
												points_avg[i],
												centroids_[points_avg[i].l]))),
						centroids_[points_avg[i].l].l);

			centroids_[points_avg[i].l].l =
					max(0.50, centroids_[points_avg[i].l].l);
			centroids_[points_avg[i].l].l =
					min(0.98, centroids_[points_avg[i].l].l);
		}
		else
		{
			if (i > 0)
			{
				decideBoundary_(
						points_avg[i-1], points_avg[i], centroids_,
						surfaces_flag, surfaces_eq, surfaces_limit);
			}
			else
			{
				decideBoundary_(
						points_avg[i], points_avg[i], centroids_,
						surfaces_flag, surfaces_eq, surfaces_limit);
			}

			if (points_avg[i].l < 0.0) { continue; }

			if (surfaces_flag[points_avg[i].l] > 0)
			{
				if ( !decideSurface(
						centroids_[points_avg[i].l],
							surfaces_eq[surfaces_flag[points_avg[i].l]-1],
							surfaces_limit[surfaces_flag[points_avg[i].l]-1]))
				{
					points_avg[i].l = UNCLASSIFIED;
				}
			}
		}
	}
	return EXIT_SUCCESS;
}

int TrainLA::ContactCheck()
{

	for(int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i].l < 0) continue;

		for(int ii=0;ii<surfaces.size();ii++)
		{
			// surface already known
			if (surfaces_flag[points_avg[i].l] > 0) continue;

			if (decideSurface(
					points_avg[i], surfaces_eq[ii], surfaces_limit[ii]) &&
				l2Norm(
					minusPoint(surfaces[ii],locations[points_avg[i].l])) < 0.2)
			{
				surfaces_flag[points_avg[i].l] = ii+1;
			}
		}
	}

	for(int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i].l < 0) continue;

		// no surface
		if (surfaces_flag[points_avg[i].l]==0) continue;

		if (!decideSurface(
				points_avg[i], surfaces_eq[surfaces_flag[points_avg[i].l]-1],
				surfaces_limit[surfaces_flag[points_avg[i].l]-1]))
		{
			points_avg[i].l = UNCLASSIFIED;
		}
	}

	return EXIT_SUCCESS;
}

int TrainLA::ClusteringExt(
		vector<point_d> &centroids_)
{
	vector<vector<unsigned char> > color_code; colorCode(color_code);
	vector<int> loc_idx_zero;

	this->Clustering(points_avg, DBSCAN_EPS, DBSCAN_MIN);

//	vector<string>goal_action, al;goal_action.resize(10);
//	showData(
//			points_avg, goal_action, al,
//			loc_idx_zero, color_code, true, false, false);

	this->CombineNearCluster(points_avg, centroids_, locations_flag, contact);

	locations_flag[0] = 0; // hack TODO
	printer(14);

	reshapeVector(surfaces_flag, locations_flag.size());

	this->ContactCheck();
	printer(40);

	this->ContactBoundary(centroids_,true);
	this->ContactBoundary(centroids_,false);

//	// Visualize
//	if (flaggg)
//	{
//		cout << centroids_.size()<<endl;
//		vector<string>goal_action, al;goal_action.resize(10);
//		showData(
//				points_, goal_action, al,
//				loc_idx_zero, color_code, true, false, false);
//	}

	return EXIT_SUCCESS;
}

int TrainLA::BuildLocationArea(
	Graph 						*Graph_,
	vector<vector<point_d> > 	&pos_vel_acc_,
	vector<int> 				contact_,
	bool 						flag_)
{
	// Gathering points
	for(int i=0;i<pos_vel_acc_.size();i++)
	{ points_avg.push_back(pos_vel_acc_[i][0]); }

	contact			= contact_;
	surfaces 		= Graph_->getSurface();
	surfaces_eq		= Graph_->getSurfaceEq();
	surfaces_limit	= Graph_->getSurfaceLimit();

	// Graph is empty
	if (Graph_->getNumberOfNodes()==0)
	{
		this->ClusteringExt(locations);

		reshapeVector(goal_action, locations.size());
		printer(15);

//		showData(
//				points_avg, goal_action, Graph_->getActionLabel(),
//				loc_idx_zero, color_code, true, true, false);
		goal_action[0] = "SHELF";
		goal_action[1] = "DISPENSER";
		goal_action[2] = "FACE";
		goal_action[3] = "TABLE2";
		goal_action[4] = "WASH";
//		goal_action[0] = "SHELF";
//		goal_action[1] = "FACE";
//		goal_action[2] = "TABLE2";
//		goal_action[3] = "THROW";
//		goal_action[0] = "SHELF";
//		goal_action[1] = "TABLE2";
//		goal_action[2] = "WASH";
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
		this->ClusteringExt(locations_);
		reshapeVector(goal_action, locations_.size());
		printer(15);

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

					// not stable the values
//					double tmp1 = locations_[i].l;
//					double tmp2 = locations[i].l;
//					tmp1 = sqrt(-log(tmp1)*BOUNDARY_VAR*2);
//					tmp2 = sqrt(-log(tmp2)*BOUNDARY_VAR*2);
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
//									max(tmp1,tmp2));

					double tmp1 = locations_[i].l;
					tmp1 = sqrt(-log(tmp1)*BOUNDARY_VAR*2);
					point_d loc_tmp =
							minusPoint(
									node_tmp.centroid,
									locations_[i]);
					tmp1 += l2Norm(loc_tmp);
					locations[ii].l = pdfExp(BOUNDARY_VAR, 0.0, tmp1);

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
