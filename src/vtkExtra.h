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

//=============================================================================

void colorCode(vector<unsigned char*> &container);

void showData(
	vector<point_t> p,
	vector<string> &label,
	vector<unsigned char*> color_,
	bool cluster,
	bool labeling);

void showConnectionOnly(
	Graph Graph_,
	vector<unsigned char*> color_);

void showConnection(
	vector<point_t> p,
	vector<string> &label,
	Graph Graph_,
	vector<unsigned char*> color_);

void plotData(
	vector<double> x,
	vector<double> y);

#endif /* VTKEXTRA_H_ */
