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
	surfaces_mid.clear();
	surfaces_min.clear();
	surfaces_max.clear();
	surfaces_eq.clear();
	surfaces_limit.clear();
}

int TrainLA::LearnBoundary(
	vector<point_d> &centroids_)
{
	for (int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i].l < 0.0) { continue; }

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

		if (surfaces_flag[points_avg[i].l] > 0)
		{
			// point transform to the coord system of the surface
			point_d point_rot={}, point_rot2={};
			point_rot.x =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][0]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][1]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][2]*points_avg[i].z;
			point_rot.y =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][3]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][4]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][5]*points_avg[i].z;
			point_rot.z =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][6]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][7]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][8]*points_avg[i].z;
			point_rot2.x =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][0]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][1]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][2]*centroids_[points_avg[i].l].z;
			point_rot2.y =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][3]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][4]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][5]*centroids_[points_avg[i].l].z;
			point_rot2.z =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][6]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][7]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][8]*centroids_[points_avg[i].l].z;

			point_rot = minusPoint(point_rot, point_rot2);

			if(	point_rot.y > surfaces_max[points_avg[i].l].y )
				surfaces_max[surfaces_flag[points_avg[i].l]-1].y = point_rot.y;

			if(	point_rot.y < surfaces_min[points_avg[i].l].y )
				surfaces_min[surfaces_flag[points_avg[i].l]-1].y = point_rot.y;
		}
	}
	return EXIT_SUCCESS;
}

int TrainLA::ContactBoundary(
	vector<point_d> &centroids_)
{
	for (int i=0;i<points_avg.size();i++)
	{
		if (i > 0)
		{ checkBoundarySphere(points_avg[i-1], points_avg[i], centroids_); }
		else
		{ checkBoundarySphere(points_avg[i]  , points_avg[i], centroids_); }

		if (points_avg[i].l < 0.0) { continue; }

		if (surfaces_flag[points_avg[i].l] > 0)
		{
			point_d point_rot={}, point_rot2={};
			point_rot.x =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][0]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][1]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][2]*points_avg[i].z;
			point_rot.y =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][3]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][4]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][5]*points_avg[i].z;
			point_rot.z =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][6]*points_avg[i].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][7]*points_avg[i].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][8]*points_avg[i].z;
			point_rot2.x =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][0]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][1]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][2]*centroids_[points_avg[i].l].z;
			point_rot2.y =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][3]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][4]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][5]*centroids_[points_avg[i].l].z;
			point_rot2.z =
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][6]*centroids_[points_avg[i].l].x +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][7]*centroids_[points_avg[i].l].y +
					surfaces_rot[surfaces_flag[points_avg[i].l]-1][8]*centroids_[points_avg[i].l].z;

			point_rot = minusPoint(point_rot, point_rot2);

			checkBoundaryCuboid(
					point_rot,
					surfaces_min[surfaces_flag[points_avg[i].l]-1],
					surfaces_max[surfaces_flag[points_avg[i].l]-1]);
		}

//		if (points_avg[i].l < 0.0) { continue; }
//
//		if (surfaces_flag[points_avg[i].l] > 0)
//		{
//			if ( !decideSurface(
//					centroids_[points_avg[i].l],
//						surfaces_eq[surfaces_flag[points_avg[i].l]-1],
//						surfaces_limit[surfaces_flag[points_avg[i].l]-1]))
//			{
//				points_avg[i].l = UNCLASSIFIED;
//			}
//		}
	}
	return EXIT_SUCCESS;
}

int TrainLA::SurfaceContactCheck(
	vector<point_d> centroids_)
{

	// label the surface involved
	for(int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i].l < 0) continue;

		for(int ii=0;ii<surfaces_mid.size();ii++)
		{
			// surface already known
			if (surfaces_flag[points_avg[i].l] > 0) continue;

			if (decideSurface(
					points_avg[i], surfaces_eq[ii], surfaces_limit[ii]) &&
				l2Norm(
					minusPoint(surfaces_mid[ii],centroids_[points_avg[i].l])) < 0.2) // HACK: to prevent arbitrary surface detection
			{
				surfaces_flag[points_avg[i].l] = ii+1;
				break;
			}
		}
	}

	// uodate the data
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
	// 1. Clustering of data
	this->Clustering(points_avg, DBSCAN_EPS, DBSCAN_MIN);

	// Visualize
	if (0)
	{
		vector<int> loc_idx_zero;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_avg, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	// 2. Combine nearby clusters
	this->CombineNearCluster(points_avg, centroids_, locations_flag, contact);

	locations_flag[0] = 0; // hack TODO
	printer(14);

	reshapeVector(surfaces_flag, locations_flag.size());

	// 3. Possible surface contact check.
	//    Surface plane is known beforehand.
	//    Assign flag to node representing the surfaces.
	this->SurfaceContactCheck(centroids_);
	printer(40);

	// 4. Learn boundary values
	this->LearnBoundary(centroids_);

	// 5. Update contact check for the data
	this->ContactBoundary(centroids_);

	// Visualize
	if (0)
	{
		vector<int> loc_idx_zero;
		vector<string>goal_action, al;goal_action.resize(10);
		showData(
				points_avg, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}

	return EXIT_SUCCESS;
}

int TrainLA::BuildLocationArea(
	Graph 						*Graph_,
	vector<vector<point_d> > 	&pos_vel_acc_,
	vector<int> 				contact_,
	bool 						flag_)
{
	this->ClearLA();

	// Gathering points
	for(int i=0;i<pos_vel_acc_.size();i++)
	{ points_avg.push_back(pos_vel_acc_[i][0]); }

	contact			= contact_;
	surfaces_mid 	= Graph_->GetSurfaceMid();
	surfaces_min 	= Graph_->GetSurfaceMin();
	surfaces_max 	= Graph_->GetSurfaceMax();
	surfaces_eq		= Graph_->GetSurfaceEq();
	surfaces_limit	= Graph_->GetSurfaceLimit();
	surfaces_rot	= Graph_->GetSurfaceRot();

	// Graph is empty
	if (Graph_->GetNumberOfNodes()==0)
	{
		this->ClusteringExt(locations);

		reshapeVector(goal_action, locations.size());
		printer(15);

		if(!strcmp(Graph_->GetObject().c_str(),"CUP"))
		{
			goal_action[0] = "SHELF";
			goal_action[1] = "DISPENSER";
			goal_action[2] = "FACE";
			goal_action[3] = "TABLE2";
			goal_action[4] = "SINK";
		}
		else if(!strcmp(Graph_->GetObject().c_str(),"ORG"))
		{
			goal_action[0] = "SHELF";
			goal_action[1] = "FACE";
			goal_action[2] = "TABLE2";
			goal_action[3] = "BIN";
		}
		else if(!strcmp(Graph_->GetObject().c_str(),"KNF"))
		{
			goal_action[0] = "SHELF";
			goal_action[1] = "TABLE2";
			goal_action[2] = "SINK";
		}
		else if(!strcmp(Graph_->GetObject().c_str(),"SPG"))
		{
			goal_action[0] = "SHELF";
			goal_action[1] = "TABLE2";
			goal_action[2] = "SINK";
		}
		else
			showData(
					points_avg, goal_action, Graph_->GetActionLabel(),
					loc_idx_zero, color_code, true, true, false);

		for(int i=0;i<locations.size();i++)
		{
			node_tt node_tmp;
			node_tmp.name 		= goal_action[i];
			node_tmp.index 		= i;
			node_tmp.surface 	= surfaces_flag[i];
			node_tmp.contact 	= locations_flag[i];
			node_tmp.centroid 	= locations[i];
			if(surfaces_flag[i]>0)
			{
				node_tmp.box_max 	= surfaces_max[surfaces_flag[i]-1];
				node_tmp.box_min 	= surfaces_min[surfaces_flag[i]-1];
			}
			else
			{
				node_tmp.box_max 	= {};
				node_tmp.box_min 	= {};
			}
			Graph_->SetNode(node_tmp);
			Graph_->addEmptyEdgeForNewNode(node_tmp.index);
		}
	}
	else
	{
		// Graph is not empty
		for(int i=0;i<Graph_->GetNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->GetNode(i, node_tmp);
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
			if(Graph_->GetNumberOfNodes()==3)
			{
				goal_action[0] = "SHELF";
				goal_action[1] = "TABLE2";
				goal_action[2] = "BIN";
			}
			if(Graph_->GetNumberOfNodes()==4)
			{
				goal_action[0] = "SHELF";
				goal_action[1] = "TABLE1";
				goal_action[2] = "SINK";
			}
			if(Graph_->GetNumberOfNodes()==5)
			{
				goal_action[0] = "SHELF";
				goal_action[1] = "DISPENSER";
				goal_action[2] = "SINK";
			}
//			showData(
//					points_avg, goal_action, Graph_->GetActionLabel(),
//					loc_idx_zero, color_code, true, true, false);
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
					Graph_->GetNode(ii, node_tmp);

//					// not stable the values
//					double tmp1 = locations_[i].l; //new
//					double tmp2 = locations[i].l;  //old
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
					Graph_->SetNode(node_tmp);

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
					node_tmp.index 		= Graph_->GetNodeList().size();
					node_tmp.surface 	= surfaces_flag[i];
					if(surfaces_flag[i]>0)
					{
						node_tmp.box_max 	= surfaces_max[surfaces_flag[i]-1];
						node_tmp.box_min 	= surfaces_min[surfaces_flag[i]-1];
					}
					else
					{
						node_tmp.box_max 	= {};
						node_tmp.box_min 	= {};
					}
					node_tmp.contact 	= locations_flag[i];
					node_tmp.centroid 	= locations_[i];
					Graph_->SetNode(node_tmp);
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
		for(int i=0;i<Graph_->GetNumberOfNodes();i++)
		{
			node_tt node_tmp = {};
			Graph_->GetNode(i, node_tmp);
			goal_action.push_back(node_tmp.name);
		}
		showData(
				points_avg, goal_action, Graph_->GetActionLabel(),
				loc_idx_zero, color_code, true, false, false);
	}

	return EXIT_SUCCESS;
}
