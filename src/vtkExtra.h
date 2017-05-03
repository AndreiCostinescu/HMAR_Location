/*
 * vtkExtra.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#ifndef VTKEXTRA_H_
#define VTKEXTRA_H_

#include "dataDeclaration.h"
#include "Graph.h"
#include "algo.h"
#include "misc.h"

//=============================================================================

void colorCode(
	vector<vector<unsigned char> > &container_);

void showData(
	vector<point_d> points_,
	vector<string> &labels_,
	vector<string> labels_ref_,
	vector<int> &loc_idx_,
	vector<vector<unsigned char> > color_,
	bool cluster_,
	bool labeling_,
	bool deleting_);

void showConnectionOnly(
	Graph Graph_,
	vector<vector<unsigned char> > color_);

void showConnection(
	Graph *Graph_,
	vector<point_d> points_,
	vector<string> &labels_,
	vector<vector<unsigned char> > color_,
	bool show_points);

void showConnectionTest(
	Graph *Graph_,
	vector<point_d> points_,
	vector<string> &labels_,
	vector<vector<unsigned char> > color_,
	bool show_points);

void plotData(
	vector<double> x,
	vector<double> y);

void plotData(
	vector<double> x,
	vector<double> y,
	vector<double> x2,
	vector<double> y2);

void plotDatas(
	vector<string> title,
	vector<double> x,
	vector<vector<vector<double> > > y);

void plotDatasGeo(
	vector<string> title,
	vector<double> x,
	vector<vector<vector<double> > > y);

vtkSmartPointer<vtkPolyDataMapper> dataPoints(
	vector<point_d> points_,
	int num_locations_,
	vector<vector<unsigned char> > color_,
	bool cluster_);

#endif /* VTKEXTRA_H_ */
