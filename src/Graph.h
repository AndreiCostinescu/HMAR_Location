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

	virtual ~Graph(){}

	string getScene()
	{
		return scene;
	}

	string getTarget()
	{
		return target;
	}

	kb_t getKB()
	{
		kb_t kb_tmp = {};
		kb_tmp.surface = surface;
		kb_tmp.surface_eq = surface_eq;
		kb_tmp.al = action_label;
		kb_tmp.ac = action_cat;
		kb_tmp.ol = object_label;
		return kb_tmp;
	}

	void setKB(
		kb_t x_);

	vector<double> getSurfaceLimit()
	{
		return surface_lim;
	}

	void setSurfaceLimit(
		vector<double> x_)
	{
		surface_lim = x_;
	}

	vector<vector<double> > getSurfaceEq()
	{
		return surface_eq;
	}

	void setSurfaceEq(
		vector<vector<double> > x_)
	{
		surface_eq = x_;
	}

	vector<point_d> getSurface()
	{
		return surface;
	}

	void setSurface(
		vector<point_d> x_)
	{
		surface = x_;
	}

	map<string,map<string,string> > getObjectLabel()
	{
		return object_label;
	}

	void setObjectLabel(map<string,map<string,string> > object_label_)
	{
		object_label = object_label_;
	}

	map<string,pair<int,int> > getActionCategory()
	{
		return action_cat;
	}

	void setActionCategory(
		map<string,pair<int,int> > x_)
	{
		action_cat = x_;
	}

	vector<string> getActionLabel()
	{
		return action_label;
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

	void getPredictionReset(
		vector<map<string, double> > &x_)
	{
		x_ = prediction_reset;
	}

	void getPredictionInit(
		map<string, double> &x_)
	{
		x_ = prediction_init;
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

	void addSurface(
		vector<vector<double> > surface_);

	state_t getState()
	{
		return state;
	}

	void setState(
		state_t x_)
	{
		state = x_;
	}


//=============================================================================
// NODES
//=============================================================================

	int getNode(
		int  idx_,
		node_tt &x_)
	{
		if (idx_<node_list.size())
		{
			x_ = node_list[idx_];
			return EXIT_SUCCESS;
		}
		else
		{
			return EXIT_FAILURE;
		}
	}

	int setNode(
		node_tt	node_)
	{
		if (node_list.size() < node_.index+1)
		{
			node_list.resize(node_.index+1);
		}
		node_list[node_.index] = node_;
		return EXIT_SUCCESS;
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

	vector<int> getSurfaceFlagList();

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

private:
	string 								scene;
	string 								target; // what kind of action set is being evaluated
	vector<node_tt> 					node_list;
	vector<vector<vector<edge_tt> > > 	edge_list;
	map<string, double>					filter_init;
	map<string, double>					prediction_init;
	vector<map<string, double> >		filter;
	vector<map<string, double> >		prediction;
	vector<map<string, double> >		prediction_reset;
	map<string,pair<int,int> >			action_cat;
	vector<string>						action_label; //fixed given beforehand

	map<string,map<string,string> > 	object_label;

	vector<point_d> 					surface;
	vector<vector<double> > 			surface_eq;
	vector<double> 						surface_lim;

	kb_t 								kb;
	state_t 							state;

};


#endif /* GRAPH_H_ */
