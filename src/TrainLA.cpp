/*
 * TrainLA.cpp
 *
 *  Created on: Apr 20, 2017
 *      Author: chen
 */

#include "TrainLA.h"

TrainLA::TrainLA() : VTK(NULL), loc_int(-1), sec_int(-1), f_win(-1)
{
}

TrainLA::~TrainLA() {}

void TrainLA::ClearLA()
{
	contact_flag.clear();
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
	surfaces_rot.clear();
}

int TrainLA::InitLA(
		int loc_int_,
		int sec_int_,
		int f_win_)
{
	loc_int = loc_int_;
	sec_int = sec_int_;
	f_win 	= f_win_;
	return EXIT_SUCCESS;
}

int TrainLA::DecideBoundaryCuboidExt(
	Vector4d &point_,
	Vector3d cuboid_min_,
	Vector3d cuboid_max_)
{
	Vector3d tmp = cuboid_max_-cuboid_min_;
	cuboid_min_ -= (tmp*0.2);
	cuboid_max_ += (tmp*0.2);
	return decideBoundaryCuboid(point_, cuboid_min_, cuboid_max_);
}

int TrainLA::LearnBoundary(
	vector<Vector4d> &centroids_)
{
	for (int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i][3] < 0.0) { continue; }

		centroids_[(int)points_avg[i][3]][3] =
				min(pdfExp(
							BOUNDARY_VAR,
							0.0,
							V4d3d(
									points_avg[i] -
									centroids_[(int)points_avg[i][3]]
								 ).norm()),
					(double)centroids_[(int)points_avg[i][3]][3]);

		centroids_[(int)points_avg[i][3]][3] =
				max(0.50, (double)centroids_[points_avg[i][3]][3]);
		centroids_[(int)points_avg[i][3]][3] =
				min(0.98, (double)centroids_[points_avg[i][3]][3]);

		if (surfaces_flag[(int)points_avg[i][3]] > 0)
		{
			// TODO : is it correct?
			centroids_[(int)points_avg[i][3]] =
					V3d4d(surfaces_mid[surfaces_flag[(int)points_avg[i][3]]-1]);
			centroids_[(int)points_avg[i][3]][3] = 0.6;

			{
				Vector4d point_rot, point_rot2;
				Matrix4d T;
				T << surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(0), 0,
					 surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(1), 0,
					 surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(2), 0,
					 0,0,0,0;

				// Last element of T is zero because last vector element is used for labeling.
				// Point transform to the coordinate system of the surface.
				// p' = [R,R*t]*p
				point_rot  =
						(T * points_avg[i]) -
						(T * centroids_[points_avg[i][3]]);

				if(	point_rot[1] > surfaces_max[surfaces_flag[(int)points_avg[i][3]]-1][1])
					surfaces_max[surfaces_flag[points_avg[i][3]]-1][1] = point_rot[1];
				if(	point_rot[1] < surfaces_min[surfaces_flag[(int)points_avg[i][3]]-1][1])
					surfaces_min[surfaces_flag[points_avg[i][3]]-1][1] = point_rot[1];
			}
		}
	}
	return EXIT_SUCCESS;
}

int TrainLA::ContactBoundary(
	vector<Vector4d> &centroids_)
{
	for (int i=0;i<points_avg.size();i++)
	{

//		if (points_avg[i][3] < 0.0) { continue; }

		checkBoundarySphere(points_avg[i], centroids_);

		if (points_avg[i][3]>=0)
		{
			if (surfaces_flag[(int)points_avg[i][3]] > 0)
			{
				Vector4d point_rot, point_rot2;
				Matrix4d T;
				T << surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(0), 0,
					 surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(1), 0,
					 surfaces_rot[surfaces_flag[(int)points_avg[i][3]]-1].row(2), 0,
					 0,0,0,0;

				// Last element of T is zero because last vector element is used for labeling.
				// Point transform to the coordinate system of the surface.
				// p' = [R,R*t]*p
				point_rot  =
						(T * points_avg[i]) -
						(T * centroids_[points_avg[i][3]]);
				point_rot[3] = points_avg[i][3];

				this->DecideBoundaryCuboidExt(
						point_rot,
						surfaces_min[surfaces_flag[(int)points_avg[i][3]]-1],
						surfaces_max[surfaces_flag[(int)points_avg[i][3]]-1]);
				points_avg[i][3] = point_rot[3];
			}
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
	vector<Vector4d> &centroids_)
{
	// label the surface involved
	for(int i=0;i<points_avg.size();i++)
	{
		if (points_avg[i][3] < 0) continue;

		for(int ii=0;ii<surfaces_mid.size();ii++)
		{
			// surface already known
			if (surfaces_flag[(int)points_avg[i][3]] > 0) continue;

			if (decideSurface(
					points_avg[i], surfaces_eq[ii], surfaces_limit[ii]) &&
				(surfaces_mid[ii] -
					 V4d3d(centroids_[(int)points_avg[i][3]])).norm()
					< 0.2) // HACK: to prevent arbitrary surface detection
			{
				surfaces_flag[(int)points_avg[i][3]] = ii+1;
				break;
			}
		}
	}
	return EXIT_SUCCESS;
}

int TrainLA::ClusteringExt(
		vector<Vector4d> &centroids_)
{
	// 1. Clustering of data
	// 2. Combine nearby clusters
	this->Clustering(
			points_avg, centroids_, locations_flag, contact_flag,
			DBSCAN_EPS, DBSCAN_MIN);
	printer(13);

	// Visualize
	if (0)
	{
		VTK = new VTKExtra(loc_int,sec_int);
		vector<int> loc_idx_zero;
		vector<string>goal_action, al;goal_action.resize(10);
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowData(
				points_avg, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
		delete VTK;
	}

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
		VTK = new VTKExtra(loc_int,sec_int);
		vector<int> loc_idx_zero;
		vector<string>goal_action, al;goal_action.resize(10);
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowData(
				points_avg, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
		delete VTK;
	}

	return EXIT_SUCCESS;
}

int TrainLA::BuildLocationArea(
		CGraph *Graph_,
		CKB *kb_,
		vector<vector<Vector4d> > &pos_vel_acc_,
		vector<int> *contact_flag_,
		bool flag_)
{
	this->ClearLA();

	// Gathering points
	for(int i=0;i<pos_vel_acc_.size();i++)
	{ points_avg.push_back((pos_vel_acc_)[i][0]); }

	contact_flag	= *contact_flag_;
	surfaces_mid 	= kb_->surface_mid;
	surfaces_min 	= kb_->surface_min;
	surfaces_max 	= kb_->surface_max;
	surfaces_eq		= kb_->surface_eq;
	surfaces_limit	= kb_->surface_lim;
	surfaces_rot	= kb_->surface_rot;

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
		{
			VTK = new VTKExtra(loc_int,sec_int);
			vector<vector<unsigned char> > color_code;
			VTK->ColorCode(color_code);
			VTK->ShowData(
					points_avg, goal_action, kb_->al,
					loc_idx_zero, color_code, true, true, false);
			delete VTK;
		}

		for(int i=0;i<locations.size();i++)
		{
			CGraph::node_t node_tmp;
			node_tmp.name 		= goal_action[i];
			node_tmp.index 		= i;
			node_tmp.surface_flag 	= surfaces_flag[i];
			node_tmp.contact 	= locations_flag[i];
			node_tmp.centroid 	= locations[i];
			if(surfaces_flag[i]>0)
			{
				node_tmp.cuboid_max 	= surfaces_max[surfaces_flag[i]-1];
				node_tmp.cuboid_min 	= surfaces_min[surfaces_flag[i]-1];
			}
			else
			{
				node_tmp.cuboid_max 	= Vector3d::Zero();
				node_tmp.cuboid_min 	= Vector3d::Zero();
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
			goal_action.push_back(Graph_->GetNode(i).name);
			locations.push_back(Graph_->GetNode(i).centroid);
		}

		vector<Vector4d> locations_; // new ones
		this->ClusteringExt(locations_);
		reshapeVector(goal_action, locations_.size());
		printer(15);

		// when new label is present
		if (flag_)
		{
			if(!strcmp(Graph_->GetObject().c_str(),"CUP"))
			{
				goal_action[0] = "SHELF";
				goal_action[1] = "DISPENSER";
				goal_action[2] = "TABLE2";
				goal_action[3] = "SINK";
			}
			else if(!strcmp(Graph_->GetObject().c_str(),"ORG"))
			{
				goal_action[0] = "SHELF";
				goal_action[1] = "TABLE2";
				goal_action[2] = "BIN";
			}
			else
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
				if (V4d3d(locations[ii] - locations_[i]).norm()
						< CLUSTER_LIMIT*1.5)
				{
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

					CGraph::node_t node = Graph_->GetNode(ii);
					double tmp1 = locations_[i][3];
					tmp1 = sqrt(-log(tmp1)*BOUNDARY_VAR*2);
					Vector4d loc_tmp = node.centroid - locations_[i];
					tmp1 += V4d3d(loc_tmp).norm();
					locations[ii][3] = pdfExp(BOUNDARY_VAR, 0.0, tmp1);
					node.centroid = locations[ii];
					Graph_->SetNode(node);

					// modify label according to list
					for(int iii=0;iii<points_avg.size();iii++)
					{
						if (points_avg[iii][3] == i)
						{
							points_avg[iii][3] = ii;
						}
					}
					break;
				}
				// add new LA
				if (ii==locations.size()-1 && flag_)
				{

					CGraph::node_t node_tmp 	= {};
					node_tmp.name 		= goal_action[i];
					node_tmp.index 		= Graph_->GetNodeList().size();
					node_tmp.surface_flag 	= surfaces_flag[i];
					if(surfaces_flag[i]>0)
					{
						node_tmp.cuboid_max 	= surfaces_max[surfaces_flag[i]-1];
						node_tmp.cuboid_min 	= surfaces_min[surfaces_flag[i]-1];
					}
					else
					{
						node_tmp.cuboid_max 	= Vector3d::Zero();
						node_tmp.cuboid_min 	= Vector3d::Zero();
					}
					node_tmp.contact 	= locations_flag[i];
					node_tmp.centroid 	= locations_[i];
					Graph_->SetNode(node_tmp);
					Graph_->addEmptyEdgeForNewNode(node_tmp.index);
					// modify label according to list
					for(int iii=0;iii<points_avg.size();iii++)
					{
						if (points_avg[iii][3] == i)
						{
							points_avg[iii][3] = node_tmp.index;
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
		(pos_vel_acc_)[i][0][3] = points_avg[i][3];
	}
	printer(16);
	printer(17);

	// Visualize
	if (0)
	{
		goal_action.clear();
		for(int i=0;i<Graph_->GetNumberOfNodes();i++)
		{
			goal_action.push_back(Graph_->GetNode(i).name);
		}

		VTK = new VTKExtra(loc_int,sec_int);
		vector<vector<unsigned char> > color_code;
		VTK->ColorCode(color_code);
		VTK->ShowData(
				points_avg, goal_action, kb_->al,
				loc_idx_zero, color_code, true, false, false);
		delete VTK;
	}

	delete VTK;

	return EXIT_SUCCESS;
}
