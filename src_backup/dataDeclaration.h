/*
 * dataDeclaration.h
 *
 *  Created on: Jan 11, 2017
 *      Author: chen
 */

#ifndef DATADECLARATION_H_
#define DATADECLARATION_H_

//#define PC

#ifdef PC
	// For backward compatibility with new VTK generic data arrays
	#define InsertNextTypedTuple InsertNextTupleValue
#endif

#include <iostream>
#include <pthread.h>
#include <signal.h>
#include <list>
#include <algorithm>
#include <math.h>
#include <numeric>
#include <vector>
#include <stack>
#include <map>
#include <semaphore.h>
#include <fstream>
#include <sstream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <iterator>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkCellData.h>
#include <vtkCellArray.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkNamedColors.h>
#include <vtkLookupTable.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPointSource.h>
#include <vtkPointData.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPropPicker.h>
#include <vtkSphereSource.h>
#include <vtkPointPicker.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkTubeFilter.h>
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
#include <vtkParametricFunctionSource.h>
#include <vtkTupleInterpolator.h>
#include <vtkTubeFilter.h>
#include <vtkParametricSpline.h>
#include <vtkDoubleArray.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkCleanPolyData.h>
#include <vtkAppendPolyData.h>
#include <vtkAssembly.h>
#include <vtkPropAssembly.h>
#include <vtkRegularPolygonSource.h>
#include <vtkPolygon.h>
#include <vtkCurvatures.h>
#include <vtkSmoothPolyDataFilter.h>
#include <vtkLight.h>

#include <gsl/gsl_integration.h>
#include <gsl/gsl_bspline.h>
#include <gsl/gsl_multifit.h>
#include <gsl/gsl_poly.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_statistics.h>
#include <gsl/gsl_linalg.h>

using namespace std;

#define CRED "\x1B[31m"
#define CGRN "\x1B[32m"
#define CYEL "\x1B[33m"
#define CBLU "\x1B[34m"
#define CMAG "\x1B[35m"
#define CCYN "\x1B[36m"
#define CWHT "\x1B[37m"
#define CNOR "\x1B[0m"

#define Sqr(x) ((x)*(x))
#define Malloc(type,n) (type *)malloc((n)*sizeof(type))
#define Calloc(type,n) (type *)calloc( n, sizeof(type))
#define v vector

//0 : all
//1 : motion
//2 : location
//3 : label only
#define VERBOSE 3

#define CLUSTER_LIMIT 0.1

#define FILTER_WIN 15

#define BOUNDARY_VAR 0.01

#define CONTACT_TRIGGER_RATIO 0.65

#define DBSCAN_EPS 0.025
#define DBSCAN_MIN 10
#define MAX_RANGE 0.05
#define	LOC_INT 100
#define	SEC_INT 72

// number of fit coefficients
// nbreak = ncoeffs + 2 - k = ncoeffs - 2 since k = 4 (for cubic b-spline)
#define NCOEFFS	15
#define NBREAK 	(NCOEFFS - 2)
#define DEGREE 10 //k+1

#define RANGE_EXCEED	3
#define RANGE_OUT		2
#define	RANGE_IN		1
#define RANGE_NULL		0

// constraints during movements
#define MOV_CONST_CIRC 	2
#define MOV_CONST_SURF 	1
#define MOV_CONST_STOP 	0

#define WIN_HEIGHT		800
#define WIN_WIDTH 		1200
#define FONT_SIZE 		10

#define CLICK_EMPTY		0
#define CLICK_LABEL		1
#define CLICK_DELETE 	2

//******************** TAKEN FROM .....
#define UNCLASSIFIED 	-1
#define NOISE 			-2

#define CORE_POINT 		 1
#define NOT_CORE_POINT 	 0

#define SUCCESS 		 0
#define FAILURE 		-3

//typedef struct point_s point_t;
//struct point_s {
//    double x, y, z;
//    int cluster_id;
//};

typedef struct node_s node_t;
struct node_s {
    unsigned int index;
    node_t *next;
};

typedef struct epsilon_neighbours_s epsilon_neighbours_t;
struct epsilon_neighbours_s {
    unsigned int num_members;
    node_t *head, *tail;
};

//********************

typedef struct point_sd point_d;
struct point_sd
{
    double x, y, z, l;
};

typedef struct sector_s sector_t;
struct sector_s
{
	double max;
	double min;
};

typedef struct sector_para_s sector_para_t;
struct sector_para_s
{
	vector<point_d> dir;
	vector<point_d> dir_n;
	vector<double>  dist;
	int loc_int;
	int sec_int;
};

typedef struct data_s data_t;
struct data_s
{
	point_d pos, vel, acc;
};

typedef struct node_ss node_tt;
struct node_ss
{
	string 			name;
	int 			index;
	int 			category; //moving???
	point_d 		centroid;
	int				surface;
	double 			surface_boundary;
	vector<data_t> 	data;
};

typedef struct edge_ss edge_tt;
struct edge_ss
{
	string 			name;
	unsigned int 	idx1;
	unsigned int 	idx2;
	vector<data_t> 	data;
	vector<double> 	sector_map; // locations int * sectors int
	vector<double> 	sector_const;
	vector<point_d> tan; // locations int
	vector<point_d> nor; // locations int
	vector<point_d> loc_start; // locations int
	vector<point_d> loc_mid; // locations int
	vector<point_d> loc_end; // locations int
	double 			total_len;
	int 			counter;
	vector<int> 	mov_const; // 0/1 activation of the mov_const labels
	vector<double> 	loc_mem; // to calculate d2(loc)
	vector<double> 	sec_mem; // to calculate d2(sec)
	vector<double> 	err_mem; // to calculate d2(err)
};

typedef struct label_s label_t;
struct label_s
{
	int 		mov;
	vector<int> loc;
	vector<int> sur; // surface
};

typedef struct pred_s pred_t;
struct pred_s
{
	vector<int> 	pred; // in or outside
	vector<double> 	pred_in; // prob shared between multiple predictions of inside
	vector<double> 	pred_in_last; // prob shared between multiple predictions of inside
	vector<double> 	pred_err; // prediction error = diff from the sectormap
};

typedef struct predict_s predict_t;
struct predict_s
{
	double		 	curvature; // curvature value : 1 for line
	double 			velocity; // velocity limit 0/1
	vector<double>	range; // in or outside
	vector<double> 	err; // prediction error = diff from the sectormap
	vector<double> 	pct_err; // prob shared between multiple predictions of inside
	vector<double>	err_diff; // change in the error compared to original
	vector<double>	pct_err_diff; // change in the error compared to original
	vector<double>	oscillate; // repetitive movement
	vector<double>	window; // knot in trajectory
};

typedef struct msg_s msg_t;
struct msg_s
{
	int 	msg; // msg option
	int 	idx;
	int 	loc_idx;
	int 	num_loc; // location
	int 	num_sur; // surface
	label_t label;
	pred_t 	pred;
};

typedef struct limit_s limit_t;
struct limit_s
{
	double vel;
	double sur_dis;
	double sur_ang;
};

#endif /* DATADECLARATION_H_ */
