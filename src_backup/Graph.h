/*
 * Graph.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef GRAPH_H_
#define GRAPH_H_

#include "dataDeclaration.h"

class Graph
{

public:

	Graph(
		string scene_,
		string object_);

	string getScene()
	{
		return scene;
	}

	string getObject()
	{
		return object;
	}

	void getActionCategory(
		map<string,pair<int,int> > &x_)
	{
		x_ = action_cat;
	}

	void setActionCategory(
		map<string,pair<int,int> > x_)
	{
		action_cat = x_;
	}

	void getActionLabel(
			vector<string> &x_)
	{
		x_ = action_label;
	}

	void setActionLabel(
		vector<string> x_)
	{
		action_label = x_;
	}

	void initFilter();

	void setInitFilter(
		vector<int> val_);

	void getInitFilter(
		map<string, double> &x_)
	{
		x_ = filter_init;
	}

	void expandFilter(
		int x_);

	void updateFilter(
		int x_);

	void getFilter(
		vector<map<string, double> > &x_)
	{
		x_ = filter;
	}

	void setFilter(
		vector<map<string, double> > x_)
	{
		filter = x_;
	}

	void getPrediction(
		vector<map<string, double> > &x_)
	{
		x_ = prediction;
	}

	void setPrediction(
		vector<map<string, double> > x_)
	{
		prediction = x_;
	}

//	void setFilter(
//		string name_,
//		vector<int> val_);

	void addSurface(
		vector<vector<double> > surface_);

//=============================================================================
// NODES
//=============================================================================

	node_tt getNode(
		int  idx_)
	{
		return node_list[idx_];
	}

	void setNode(
		node_tt	node_)
	{
		if (node_list.size() < node_.index+1)
		{
			node_list.resize(node_.index+1);
		}
		node_list[node_.index] = node_;
	}

	int getNumberOfNodes()
	{
		return node_list.size();
	}

	vector<node_tt> getNodeList()
	{
		return node_list;
	}

	vector<point_d> getCentroidList();

//=============================================================================
// EDGES
//=============================================================================

	void addEmptyEdgeForNewNode(
		int	idx_);

	vector<vector<vector<edge_tt> > > getListOfEdges()
	{
		return edge_list;
	}

	edge_tt getEdge(
		int n1_,
		int n2_,
		int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_];
	}

	void setEdge(
		int 	n1_,
		int 	n2_,
		int 	edge_num_,
		edge_tt edge_)
	{
		edge_list[n1_][n2_][edge_num_] = edge_;
	}

	int getEdgeCounter(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_)
	{
		return edge_list[n1_][n2_][edge_num_].counter;
	}

	void setEdgeCounter(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		unsigned int x_)
	{
		edge_list[n1_][n2_][edge_num_].counter += x_;
	}

	void getEdgeMovementConstraint(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<int> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].mov_const;
	}

	void setEdgeMovementConstraint(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<int> x_)
	{
		edge_list[n1_][n2_][edge_num_].mov_const = x_;
	}

	void getEdgeLocMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].loc_mem;
	}

	void setEdgeLocMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].loc_mem = x_;
	}

	void getEdgeSecMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].sec_mem;
	}

	void setEdgeSecMem(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sec_mem = x_;
	}

	void getEdgeSectorMap(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].sector_map;
	}

	void setEdgeSectorMap(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<double> x_)
	{
		edge_list[n1_][n2_][edge_num_].sector_map = x_;
	}

	void getEdgeTan(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].tan;
	}

	void setEdgeTan(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> x_)
	{
		edge_list[n1_][n2_][edge_num_].tan = x_;
	}

	void getEdgeNor(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> &x_)
	{
		x_ = edge_list[n1_][n2_][edge_num_].nor;
	}

	void setEdgeNor(
		unsigned int n1_,
		unsigned int n2_,
		unsigned int edge_num_,
		vector<point_d> x_)
	{
		edge_list[n1_][n2_][edge_num_].nor = x_;
	}















	void updateMovLabel(
		vector<string> movLabel_);

	void extendNode(
		unsigned int 	node_num_,
		vector<data_t> 	data_);

	bool checkNode(
		unsigned int node_index_);

	void updateNodeName(
		vector<string> names_);

	void updateNodeLocation(
		vector<point_d> loc_);


	void addEdge(
		vector<data_t>	data_,
		vector<double>	sector_map_,
		unsigned int	n1_,
		unsigned int	n2_,
		unsigned int	edge_num_);

	void updateEdgeData(
		vector<data_t>	data_,
		unsigned int 	n1_,
		unsigned int 	n2_,
		unsigned int 	edge_num_);

	void updateEdgeConst(
		vector<double> 	sector_map_,
		unsigned int  	n1_,
		unsigned int  	n2_,
		unsigned int 	edge_num_);

	void updateEdgeSector(
		vector<double>	sector_map_,
		unsigned int  	n1_,
		unsigned int  	n2_,
		unsigned int 	edge_num_);

	void updateEdgeNormal(
		vector<point_d>  normal_,
		unsigned int  	 n1_,
		unsigned int  	 n2_,
		unsigned int 	 edge_num_);

	void updateEdgeTangent(
		vector<point_d> 	tangent_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void updateEdgeLocStartMidEnd(
		vector<point_d> 	start_,
		vector<point_d> 	mid_,
		vector<point_d> 	end_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void updateEdgeLocDist(
		double 				total_,
		unsigned int  		n1_,
		unsigned int  		n2_,
		unsigned int 		edge_num_);

	void extendEdge(
		vector<data_t> 	data_,
		unsigned int 	node_index1_,
		unsigned int 	node_index2_,
		unsigned int 	edge_num_);

	bool checkEdge(
		unsigned int node_index1_,
		unsigned int node_index2_);

	void updateSectorPara(
		sector_para_t sector_para_);

	void incrementCounter(
		int edge_,
		int edge_num_)
	{counter[edge_][edge_num_]++;}



	vector<string> getNodeName();

	vector<double> getInitSector()
	{return sector_zero;}

	vector<double> getInitSectorConst()
	{return sector_zero;}

	sector_para_t getSectorPara()
	{return sector_para;}

	vector<vector<edge_tt> > getEdgeList()
	{return edges;}

	vector<vector<double> > getNodeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);

	vector<vector<double> > getEdgeDataLabel(
		bool pos_=false,
		bool vel_=false,
		bool acc_=false);

	vector<vector<double> > getSurface()
	{return surface;}



	vector<string> getMovLabel()
	{return movLabel;}

	int getCounter(
		int edge_,
		int edge_num_)
	{return counter[edge_][edge_num_];}

	virtual ~Graph(){}

private:
	string 								scene;
	string 								object;
	vector<node_tt> 					node_list;
	vector<vector<vector<edge_tt> > > 	edge_list;
	map<string, double>					filter_init;
	vector<map<string, double> >		filter;
	vector<map<string, double> >		prediction;
	map<string,pair<int,int> >			action_cat;
	vector<string>						action_label; //fixed given beforehand




	vector<vector<int> >		counter;

	vector<string>				movLabel;
	vector<double> 				sector_zero;
	sector_para_t 				sector_para;
	vector<point_d>				tangent_zero;
	vector<point_d>				normal_zero;
	vector<point_d>				loc_start_zero;
	vector<point_d>				loc_mid_zero;
	vector<point_d>				loc_end_zero;

	node_tt 					node;
	edge_tt 					edge;
	vector<node_tt> 			nodes;
	vector<vector<edge_tt> > 	edges;
	vector<vector<double> > 	surface;


};


#endif /* GRAPH_H_ */
