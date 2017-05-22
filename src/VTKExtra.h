/*******************************************************************************
 * VTKExtra.h
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 *      Detail: For viewing purposes.
 ******************************************************************************/

#ifndef VTKEXTRA_H_
#define VTKEXTRA_H_

#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <vector>

#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkSphereSource.h>
#include <vtkPointPicker.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkLineSource.h>
#include <vtkDoubleArray.h>
#include <vtkChartXY.h>
#include <vtkTable.h>
#include <vtkPlot.h>
#include <vtkFloatArray.h>
#include <vtkContextView.h>
#include <vtkContextScene.h>
#include <vtkPen.h>
#include <vtkSmartPointer.h>
#include <vtkTubeFilter.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolygon.h>
#include <vtkRect.h>
#include <vtkAxis.h>
#include <vtkProperty.h>
#include <vtkPointData.h>
#include <vtkGL2PSExporter.h>

#include "CGraph.h"

using namespace std;

#define CLICK_EMPTY		0
#define CLICK_LABEL		1
#define CLICK_DELETE 	2

#define WIN_HEIGHT		800
#define WIN_WIDTH 		1280
#define FONT_SIZE 		20

#define VTKCRED "\x1B[31m"
#define VTKCGRN "\x1B[32m"
#define VTKCYEL "\x1B[33m"
#define VTKCBLU "\x1B[34m"
#define VTKCMAG "\x1B[35m"
#define VTKCCYN "\x1B[36m"
#define VTKCWHT "\x1B[37m"
#define VTKCNOR "\x1B[0m"

#define Sqr(x) ((x)*(x))

class VTKExtra
{

private:

int LOC_INT;
int SEC_INT;

static inline Vector3d V4d3d(Vector4d A)
{
	Vector3d B;
	B(0)=A(0);
	B(1)=A(1);
	B(2)=A(2);
	return B;
}

void ArrayTovector(
		unsigned char *A,
		int size,
		vector<unsigned char> &B);

void vectorToArray(
		vector<unsigned char> A,
		unsigned char *B);

Vector3d RodriguesVec(
	AngleAxisd aa_,
	Vector3d vec_);

public:

VTKExtra(
		int loc_int_,
		int sec_int_);
virtual ~VTKExtra();

void ColorCode(vector<vector<unsigned char> > &container_);

void ShowData(
		vector<Vector4d> points_,
		vector<string> &labels_,
		vector<string> labels_ref_,
		vector<int> &loc_idx_,
		vector<vector<unsigned char> > color_,
		bool cluster_,
		bool labeling_,
		bool deleting_);

void ShowConnectionOnly(
		CGraph Graph_,
		vector<vector<unsigned char> > color_);

void ShowConnection(
		CGraph *Graph_,
		vector<Vector4d> points_,
		vector<string> &labels_,
		vector<vector<unsigned char> > color_,
		bool show_points);

void ShowConnectionTest(
		CGraph *Graph_,
		vector<Vector4d> points_,
		vector<string> &labels_,
		vector<vector<unsigned char> > color_,
		bool show_points);

void PlotData(
		vector<double> x,
		vector<double> y);

void PlotData(
		vector<double> x,
		vector<double> y,
		vector<double> x2,
		vector<double> y2);

void PlotDatas(
		vector<string> title,
		vector<double> x,
		vector<vector<vector<double> > > y);

void PlotDatasGeo(
		vector<string> title,
		vector<double> x,
		vector<vector<vector<double> > > y);

vtkSmartPointer<vtkPolyDataMapper> DataPoints(
		vector<Vector4d> points_,
		int num_locations_,
		vector<vector<unsigned char> > color_,
		bool cluster_);
};

#endif /* VTKEXTRA_H_ */
