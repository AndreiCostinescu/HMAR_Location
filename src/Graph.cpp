/*
 * Graph.cpp
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#include "Graph.h"


Graph::Graph(
	string scene_,
	string object_)
{
	scene 	= scene_;
	object 	= object_;
}

void Graph::initFilter()
{
	map<string,double> mask;
	for(int i=0;i<action_label.size();i++)
	{
		filter_init[action_label[i]] 		= 0.0;
		prediction_init[action_label[i]] 	= 0.0;
	}
}

void Graph::setInitFilter(
	vector<int> val_)
{
	if (val_.size()!=filter_init.size())
	{
		cerr << "[WARNING] : DIM(FILTER MASK) != DIM(ACTION LABEL)" << endl;
	}
	else
	{
		for(int i=0;i<val_.size();i++)
		{
			filter_init[action_label[i]] = val_[i];
		}
	}
}

void Graph::expandFilter(
	int x_)
{
	while (filter.size() < x_)
	{
		filter.push_back(filter_init);
		prediction_reset.push_back(prediction_init);
	}
}

void Graph::updateFilter(
	int x_)
{
	filter[x_][node_list[x_].name] = 1.0;
}

//void Graph::setFilter(
//	string name_,
//	vector<int> val_)
//{
//	bool check = false;
//	for (
//			map<string, map<string, int> >::iterator it = filter.begin();
//			it != filter.end();
//			it++)
//	{
//		if (!strcmp(it->first.c_str(),name_.c_str()))
//		{
//			check = true;
//			if (val_.size()!=action_label.size())
//			{
//				cerr << "[WARNING] : DIM(FILTER MASK) != DIM(ACTION LABEL)" << endl;
//			}
//			else
//			{
//				for(int ii=0;ii<val_.size();ii++)
//				{
//					it->second[action_label[ii]] = val_[ii];
//				}
//			}
//		}
//	}
//	if (!check)
//	{
//		cout << "[WARNING] : THE FILTER MASK WAS NOT INITIALIZED." << endl;
//	}
//}
//=============================================================================
// NODES
//=============================================================================

vector<point_d> Graph::getCentroidList()
{
	vector<point_d> tmp;
	tmp.resize(node_list.size());
	for(int i=0;i<node_list.size();i++)
	{
		tmp[i] = node_list[i].centroid;
	}
	return tmp;
}

//=============================================================================
// EDGES
//=============================================================================


void Graph::addEmptyEdgeForNewNode(
	int idx_)
{
	if(edge_list.size()<idx_+1)
	{
		edge_list.resize(idx_+1);
		for(int i=0;i<edge_list.size();i++)
		{
			if(edge_list[i].size()<=getNumberOfNodes())
			{
				edge_list[i].resize(getNumberOfNodes());
				for(int ii=0;ii<edge_list[i].size();ii++)
				{
					edge_list[i][ii].push_back({});
					edge_list[i][ii][0].loc_start.resize(LOC_INT);
					edge_list[i][ii][0].loc_mid.resize(LOC_INT);
					edge_list[i][ii][0].loc_end.resize(LOC_INT);
					edge_list[i][ii][0].tan.resize(LOC_INT);
					edge_list[i][ii][0].nor.resize(LOC_INT);
					edge_list[i][ii][0].sector_map.resize(LOC_INT*SEC_INT);
					edge_list[i][ii][0].sector_const.resize(LOC_INT*SEC_INT);
					edge_list[i][ii][0].mov_const.resize(2);
				}
			}
		}
	}
}













//------------------------------------------------------------------------------------------------------------------------------



void Graph::addSurface(
	vector<vector<double> > surface_)
{
	surface = surface_;
}








void Graph::updateMovLabel(
	vector<string> movLabel_)
{
	movLabel.clear();
	movLabel = movLabel_;
}

void Graph::updateSectorPara(
	sector_para_t sector_para_)
{
	sector_para = {};
	sector_para = sector_para_;
}

//=============================================================================
// NODES
//=============================================================================
void Graph::extendNode(
	unsigned int 	node_num_,
	vector<data_t> 	data_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data used to extend node is empty." << endl;
	else if(data_.size()==1)
		nodes[node_num_].data.push_back(data_[0]);
	else
		nodes[node_num_].data.insert(nodes[node_num_].data.end(),
				                     data_.begin(), data_.end());
}

bool Graph::checkNode(
	unsigned int n1_)
{
	bool check_flag = false;
	if(nodes.size() >= n1_+1)
		check_flag = true;
	return check_flag;
}

vector<string> Graph::getNodeName()
{
	vector<string> names;
	names.resize(nodes.size());
	for(int i=0;i<nodes.size();i++)
		names[i] = nodes[i].name;
	return names;
}

void Graph::updateNodeName(vector<string> names_)
{
	if(names_.size()!=nodes.size())
		printf("[WARNING] : Number of names is not the same as the number of nodes in graph...");
	else
		for(int i=0;i<nodes.size();i++)
			nodes[i].name = names_[i];
}

void Graph::updateNodeLocation(vector<point_d> loc_)
{
	if(loc_.size()!=nodes.size())
		printf("[WARNING] : Number of locations is not the same as the number of nodes in graph...");
	else
		for(int i=0;i<nodes.size();i++)
			nodes[i].centroid = loc_[i];
}

//=============================================================================
// EDGES
//=============================================================================

void Graph::addEdge(
	vector<data_t> data_,
	vector<double> sector_map_,
	unsigned int   n1_,
	unsigned int   n2_,
	unsigned int   edge_num_)
{

	edge = {};

	edge.idx1		= n1_;
	edge.idx2		= n2_;
	edge.data		= data_;
	edge.sector_map	= sector_map_;

	if (edges[n1_*nodes.size()+n2_].size()>edge_num_)
	{
		edge.sector_const =
				edges[n1_*nodes.size()+n2_][edge_num_].sector_const;
		edges[n1_*nodes.size()+n2_][edge_num_] = edge;
	}
	else
	{
		edges[n1_*nodes.size()+n2_].resize(edge_num_);
		edge.sector_const = sector_zero;
		edges[n1_*nodes.size()+n2_][edge_num_] = edge;
	}
}


void Graph::updateEdgeData(
	vector<data_t> 	 data_,
	unsigned int 	 n1_,
	unsigned int 	 n2_,
	unsigned int 	 edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].data = data_;
}

void Graph::updateEdgeConst(
	vector<double> 	sector_map_,
	unsigned int  	n1_,
	unsigned int  	n2_,
	unsigned int 	edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].sector_const = sector_map_;
}

void Graph::updateEdgeSector(
	vector<double>	sector_map_,
	unsigned int  	n1_,
	unsigned int  	n2_,
	unsigned int 	edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].sector_map = sector_map_;
}

void Graph::updateEdgeNormal(
	vector<point_d> 	normal_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].nor = normal_;
}

void Graph::updateEdgeTangent(
	vector<point_d> 	tangent_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].tan = tangent_;
}

void Graph::updateEdgeLocStartMidEnd(
	vector<point_d> 	start_,
	vector<point_d> 	mid_,
	vector<point_d> 	end_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].loc_start = start_;
	edges[n1_*nodes.size()+n2_][edge_num_].loc_mid = mid_;
	edges[n1_*nodes.size()+n2_][edge_num_].loc_end = end_;
}

void Graph::updateEdgeLocDist(
	double				total_,
	unsigned int  		n1_,
	unsigned int  		n2_,
	unsigned int 		edge_num_)
{
	edges[n1_*nodes.size()+n2_][edge_num_].total_len = total_;
}

void Graph::extendEdge(
	vector<data_t> data_,
	unsigned int n1_,
	unsigned int n2_,
	unsigned int edge_num_)
{
	if(data_.size()==0)
		cout << "[WARNING] : Data to extend edge is empty." << endl;
	else if(data_.size()==1)
		edges[n1_*nodes.size()+n2_][edge_num_].data.push_back(data_[0]);
	else
		edges[n1_*nodes.size()+n2_][edge_num_].data.insert(
				edges[n1_*nodes.size()+n2_][edge_num_].data.end(),
				data_.begin(), data_.end());
}

bool Graph::checkEdge(
	unsigned int n1_,
	unsigned int n2_)
{
	bool check_flag = true;
	if (edges[n1_*nodes.size()+n2_].empty())
	{
		check_flag = false;
	}
	return check_flag;
}

vector<vector<double> > Graph::getEdgeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output;
	vector<vector<edge_tt> > list = getEdgeList();
	vector<data_t> data;
	// list of nodes with edges
	for(int i=0; i<list.size();i++)
	{
		// list of edges for each node
		for(int ii=0; ii<list[i].size();ii++)
		{
			data = list[i][ii].data;
			// list of data in each edge
			for(int iii=0; iii<data.size();iii++)
			{
				vector<double> tmp_out;
				tmp_out.push_back(i);
				if(pos_)
				{
					point_d tmp_pos = data[iii].pos;
					tmp_out.push_back(tmp_pos.x);
					tmp_out.push_back(tmp_pos.y);
					tmp_out.push_back(tmp_pos.z);
				}
				if(vel_)
				{
					point_d tmp_vel = data[iii].vel;
					tmp_out.push_back(tmp_vel.x);
					tmp_out.push_back(tmp_vel.y);
					tmp_out.push_back(tmp_vel.z);
				}
				if(acc_)
				{
					point_d tmp_acc = data[iii].acc;
					tmp_out.push_back(tmp_acc.x);
					tmp_out.push_back(tmp_acc.y);
					tmp_out.push_back(tmp_acc.z);
				}
				output.push_back(tmp_out);
			}
		}
	}
	return output;
}

vector<vector<double> > Graph::getNodeDataLabel(
	bool pos_,
	bool vel_,
	bool acc_)
{
	vector<vector<double> > output; // length * XYZId
	vector<node_tt> list = getNodeList();
	vector<data_t> data;
	// list of nodes
	for(int i=0; i<list.size();i++)
	{
		//printf("Category %02d %02d\n", i, list[i].category);
		data = list[i].data;
		// list of node data
		for(int ii=0; ii<data.size();ii++)
		{
			vector<double> tmp_out;
			if(pos_)
			{
				point_d tmp_pos = data[ii].pos;
				tmp_out.push_back(tmp_pos.x);
				tmp_out.push_back(tmp_pos.y);
				tmp_out.push_back(tmp_pos.z);
				tmp_out.push_back(i);
			}
			if(vel_)
			{
				point_d tmp_vel = data[ii].vel;
				tmp_out.push_back(tmp_vel.x);
				tmp_out.push_back(tmp_vel.y);
				tmp_out.push_back(tmp_vel.z);
				tmp_out.push_back(i);
			}
			if(acc_)
			{
				point_d tmp_acc = data[ii].acc;
				tmp_out.push_back(tmp_acc.x);
				tmp_out.push_back(tmp_acc.y);
				tmp_out.push_back(tmp_acc.z);
				tmp_out.push_back(i);
			}
			output.push_back(tmp_out);
		}
	}
	return output;
}


