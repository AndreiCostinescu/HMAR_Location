/*
 * vtkExtra.cpp
 *
 *  Created on: Feb 16, 2017
 *      Author: chen
 */

#include "vtkExtra.h"

// ============================================================================
// MOUSE
// ============================================================================

class customMouseInteractorStyle : public vtkInteractorStyleTrackballCamera
{
	public:
		static customMouseInteractorStyle* New();
		vtkTypeMacro(
				customMouseInteractorStyle, vtkInteractorStyleTrackballCamera);

		void setLeftButton(int option_)	{left_click_ = option_;}

		void setColors(vector<vector<unsigned char> > color) {color_ = color;}

		void setLabels(vector<string> label_) {label = label_;}

		void setRefLabels(vector<string> label_) {label_ref = label_;}

		void setLocations(vector<int> x) {location = x;}

		vector<string> getLabels() {return label;}

		vector<int> getLocations() {return location;}

		void setNumberOfLabels(int x) {num_locations = x;}

		virtual void OnLeftButtonDown();

//    virtual void OnMiddleButtonDown()
//    {
//      std::cout << "Pressed middle mouse button." << std::endl;
//      // Forward events
//      vtkInteractorStyleTrackballCamera::OnMiddleButtonDown();
//    }
//
//    virtual void OnRightButtonDown()
//    {
//      std::cout << "Pressed right mouse button." << std::endl;
//      // Forward events
//      vtkInteractorStyleTrackballCamera::OnRightButtonDown();
//    }

	private:
		int 							num_locations;
		int								left_click_;
		vector<int> 					location;
		vector<string> 					label;
		vector<string> 					label_ref;
		vector<vector<unsigned char> > 	color_;

		void writeText(
			const char* text,
			double *rgb,
			int x,
			int y);
};

void customMouseInteractorStyle::OnLeftButtonDown()
{
	switch(left_click_)
	{
		case CLICK_LABEL:
		{
			int* clickPos = this->GetInteractor()->GetEventPosition();

			// Pick from this location.
			vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
			picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

			double rgb[3];
			if (picker->GetActor()!=0)
			{
				double* pos = picker->GetPickPosition();
				picker->GetActor()
					  ->GetMapper()->GetInput()
					  ->GetPointData()->GetScalars()
					  ->GetTuple(picker->GetPointId(),rgb);
			}

			for(int i = 0;i<num_locations;i++)
			{
				if (rgb[0]==color_[i+1][0] &&
					rgb[1]==color_[i+1][1] &&
					rgb[2]==color_[i+1][2])
				{
					for(int ii=0;ii<5;ii++)
					{
						printf(">>>>> %d %s\n", ii, label_ref[ii].c_str());
					}
					printf(">>>>> Select label from list by entering the number : ");
					string mystr; getline (cin, mystr);
					if (atoi(mystr.c_str()) < 0 || atoi(mystr.c_str()) >= 5)
					{
						printf(CYEL ">>>>> [WARNING] : Please choose only from the listed numbers.\n" CNOR);
						printf(">>>>> Pick a location with color.");
						break;
					}
					else
					{
						printf(">>>>> %s selected.\n", label_ref[atoi(mystr.c_str())].c_str());
					}

					if (!label[i].empty())
					{
						printf(CYEL ">>>>> [WARNING] : Label has been given. Do you wish to overwrite? [Y/N]\n" CNOR);
						printf(">>>>> ");
						while(1)
						{
							string mystr2; getline (cin, mystr2);
							if(!strcmp(mystr2.c_str(),"Y"))
							{
								printf(CYEL ">>>>> [WARNING] : Label has been overwritten. New label : %s\n" CNOR, label_ref[atoi(mystr.c_str())].c_str());
								label[i] = label_ref[atoi(mystr.c_str())];
								writeText(label_ref[atoi(mystr.c_str())].c_str(), rgb, 10, 470-10*(i+1));
								break;
							}
							if(!strcmp(mystr2.c_str(),"N"))
							{
								printf(CYEL ">>>>> [WARNING] : Label has not been overwritten.\n" CNOR);
								break;
							}
						}
					}
					else
					{
						label[i] = label_ref[atoi(mystr.c_str())];
						writeText(label_ref[atoi(mystr.c_str())].c_str(), rgb, 10, 470-10*(i+1));
						break;
					}
				}
			}

			bool flag = true;
			for(int i=0;i<num_locations;i++)
			{
				// when there is no labeling at all
				if (label.empty()) break;
				// check for completeness of labelling
				if (label[i].empty()) break;
				// else
				if (i==num_locations-1)
				{
					flag = false;
					printf(CYEL ">>>>> [WARNING] : Label has been fully labeled. Proceed? [Y/N]\n" CNOR);
					printf(">>>>> ");
					string mystr3; getline (cin, mystr3);
					if(!strcmp(mystr3.c_str(),"Y"))
					{
						this->GetInteractor()->TerminateApp();
					}
					else
					{
						printf(">>>>> Pick a location with color.\n");
					}
				}
			}
			if(flag) {printf(">>>>> Pick a location with color.\n");}

			// Forward events, camera manipulation
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
		}
		case CLICK_DELETE:
		{
			//std::cout << "Pressed left mouse button." << std::endl;
			int* clickPos = this->GetInteractor()->GetEventPosition();

			// Pick from this location.
			vtkSmartPointer<vtkPointPicker>  picker = vtkSmartPointer<vtkPointPicker>::New();
			picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());

			double rgb[3];
			if (picker->GetActor()!=0)
			{
				picker->GetActor()
					  ->GetMapper()->GetInput()
					  ->GetPointData()->GetScalars()
					  ->GetTuple(picker->GetPointId(),rgb);
			}

			for(int i = 0;i<num_locations;i++)
			{
				if (rgb[0]==color_[i+1][0] &&
					rgb[1]==color_[i+1][1] &&
					rgb[2]==color_[i+1][2])
				{
					cout << ">>>>> Delete label? [Y/N]\n";
					string mystr; getline (cin, mystr);
					while(1)
					{
						if(!strcmp(mystr.c_str(),"Y"))
						{
							cout << ">>>>> [WARNING] : Label has been deleted." << endl;
							location[i] = -1;
							break;
						}
						if(!strcmp(mystr.c_str(),"N"))
						{
							cout << ">>>>> [WARNING] : Label has not been deleted." << endl;
							break;
						}
					}
				}
			}
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
		}
		default :
			vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
			break;
	}
}

void customMouseInteractorStyle::writeText(
	const char* text,
	double *rgb,
	int x,
	int y)
{
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkTextActor> textActor;
	textActor = vtkSmartPointer<vtkTextActor>::New();
	textActor->SetInput ( text );
	textActor->SetPosition ( x, y );
	textActor->GetTextProperty()->SetFontSize ( 10 );
	textActor->GetTextProperty()->SetColor ( rgb[0]/255, rgb[1]/255, rgb[2]/255 );
	this->GetDefaultRenderer()->AddActor2D ( textActor );
	this->GetInteractor()->Render();
}

vtkStandardNewMacro(customMouseInteractorStyle);

// ============================================================================
// COLORCODE
// ============================================================================

void colorCode(
	vector<vector<unsigned char> > &container_)
{
	int N = 1;
	container_.clear();
	container_.resize(N*12);
	// Setup colors
	unsigned char cw[]   = {255, 255, 255};
	unsigned char cy[]   = {255, 255, 0};
	unsigned char co[]   = {255, 127, 0};
	unsigned char cr[]   = {255, 0, 0};
	unsigned char clg[]  = {127, 255, 0};
	unsigned char cg[]   = {0, 255, 0};
	unsigned char cgb[]  = {0, 255, 127};
	unsigned char cc[]   = {0, 255, 255};
	unsigned char clb[]  = {0, 127, 255};
	unsigned char cb[]   = {0, 0, 255};
	unsigned char cpb[]  = {127, 0, 255};
	unsigned char cpr[]  = {255, 0, 127};
	for(int i=0;i<N;i++)
	{
		array2vector(cw,	3,	container_[12*i+0]);
		array2vector(cy, 	3, 	container_[12*i+1]);
		array2vector(co, 	3, 	container_[12*i+2]);
		array2vector(cr, 	3, 	container_[12*i+3]);
		array2vector(clg, 	3,	container_[12*i+4]);
		array2vector(cg, 	3, 	container_[12*i+5]);
		array2vector(cgb, 	3, 	container_[12*i+6]);
		array2vector(cc, 	3, 	container_[12*i+7]);
		array2vector(clb, 	3, 	container_[12*i+8]);
		array2vector(cb, 	3, 	container_[12*i+9]);
		array2vector(cpb, 	3, 	container_[12*i+10]);
		array2vector(cpr, 	3, 	container_[12*i+11]);
	}
}

vtkSmartPointer<vtkPolyDataMapper> dataPoints(
	vector<point_d> points_,
	int num_locations_,
	vector<vector<unsigned char> > color_,
	bool cluster_)
{
	int num_locations = num_locations_;

	// Create the geometry of a point (the coordinate)
	// add point to polydata to create the vertices for glyph
	// creating the vertices with a small cube glyph
	// add points and vertices to polydata
	// giving the points color
	// Create a mapper and actor
	// Create a renderer, render window, and interactor
	// custom mouse
	// Setup the text and add it to the renderer
	vtkSmartPointer<vtkPoints> 					points;
	vtkSmartPointer<vtkPolyData> 				pointsPolydata;
	vtkSmartPointer<vtkVertexGlyphFilter>		vertexFilter;
	vtkSmartPointer<vtkPolyData> 				polydata;
	vtkSmartPointer<vtkUnsignedCharArray> 		colors;
	vtkSmartPointer<vtkPolyDataMapper>			mapper;
//	vtkSmartPointer<vtkActor> 					actor;
//	vtkSmartPointer<vtkRenderer> 				renderer;
//	vtkSmartPointer<vtkRenderWindow> 			renWin;
//	vtkSmartPointer<vtkRenderWindowInteractor> 	renWinInter;
//	vtkSmartPointer<customMouseInteractorStyle> style;
//	vtkSmartPointer<vtkTextActor> 				textActor;

	points 					= vtkSmartPointer<vtkPoints>::New();
	pointsPolydata 			= vtkSmartPointer<vtkPolyData>::New();
	vertexFilter			= vtkSmartPointer<vtkVertexGlyphFilter>::New();
	polydata 				= vtkSmartPointer<vtkPolyData>::New();
	colors 					= vtkSmartPointer<vtkUnsignedCharArray>::New();
	mapper 					= vtkSmartPointer<vtkPolyDataMapper>::New();

	for(int i=0;i<points_.size();i++)
		points->InsertNextPoint(points_[i].x, points_[i].y, points_[i].z);

	pointsPolydata->SetPoints(points);

#if VTK_MAJOR_VERSION <= 5
	vertexFilter->SetInputConnection(pointsPolydata->GetProducerPort());
	vertexFilter->Update();
#else
	vertexFilter->SetInputData(pointsPolydata);
	vertexFilter->Update();
#endif

	polydata->ShallowCopy(vertexFilter->GetOutput());

	if(cluster_)
	{
		colors->SetNumberOfComponents(3);
		colors->SetName ("Colors");
		vector<unsigned char*> color_tmp; color_tmp.resize(color_.size());
		for(int i=0;i<color_.size();i++)
		{
			color_tmp[i] = Calloc(unsigned char,3);
			vector2array(color_[i], color_tmp[i]);
		}
		for(int i=0;i<points_.size();i++)
		{
			if (points_[i].l < num_locations &&
				points_[i].l >= 0)
			{
				colors->InsertNextTypedTuple(color_tmp[points_[i].l+1]);
			}
			else
			{
				colors->InsertNextTypedTuple(color_tmp[0]);
			}
		}
		polydata->GetPointData()->SetScalars(colors);
	}

#if VTK_MAJOR_VERSION <= 5
	  mapper->SetInput(polydata);
#else
	  mapper->SetInputData(polydata);
#endif

	return mapper;
}

void showData(
	vector<point_d> points_,
	vector<string> &labels_,
	vector<string> labels_ref_,
	vector<int> &loc_idx_,
	vector<vector<unsigned char> > color_,
	bool cluster_,
	bool labeling_,
	bool deleting_)
{
	int num_locations = labels_.size();

	vtkSmartPointer<vtkPolyDataMapper>			mapper;
	vtkSmartPointer<vtkActor> 					actor;
	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renWin;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renWinInter;
	vtkSmartPointer<customMouseInteractorStyle> style;
	vtkSmartPointer<vtkTextActor> 				textActor;
	vtkSmartPointer<vtkCamera>					camera;

//	mapper 			= vtkSmartPointer<vtkPolyDataMapper>::New();
	actor 			= vtkSmartPointer<vtkActor>::New();
	renderer 		= vtkSmartPointer<vtkRenderer>::New();
	renWin 			= vtkSmartPointer<vtkRenderWindow>::New();
	renWinInter 	= vtkSmartPointer<vtkRenderWindowInteractor>::New();
	style 			= vtkSmartPointer<customMouseInteractorStyle>::New();
//	textActor 		= vtkSmartPointer<vtkTextActor>::New();
	camera 			= vtkSmartPointer<vtkCamera>::New();

	actor->SetMapper(dataPoints(points_, num_locations, color_, cluster_));
	actor->GetProperty()->SetPointSize(3);
//	actor->GetProperty()->SetColor(1.0, 0.0, 0.0);

	renWin->SetSize(WIN_WIDTH, WIN_HEIGHT); //(width, height)
	renWin->AddRenderer(renderer);
	renWinInter->SetRenderWindow(renWin);

	if (labeling_)
	{
		style->setLeftButton(CLICK_LABEL);
		textActor = vtkSmartPointer<vtkTextActor>::New();
		printf(">>>>> Pick a location with color.\n");
		if(!labels_.empty()) textActor->SetInput (labels_[0].c_str());
		textActor->SetPosition ( 10, (WIN_HEIGHT-20) );
		textActor->GetTextProperty()->SetFontSize ( FONT_SIZE );
		textActor->GetTextProperty()->SetColor ( 1.0, 1.0, 1.0 );
		renderer->AddActor2D (textActor);
	}
	else
	{
		for(int i=0;i<num_locations;i++)
		{
			if(labels_.empty()) continue;
			if(labels_[i].empty()) continue;
			textActor = vtkSmartPointer<vtkTextActor>::New();
			textActor->SetInput(labels_[i].c_str());
			textActor->SetPosition(10, (WIN_HEIGHT-20)-i*FONT_SIZE);
			textActor->GetTextProperty()->SetFontSize(FONT_SIZE);
			textActor->GetTextProperty()
					 ->SetColor((double)color_[i+1][0]/255,
								(double)color_[i+1][1]/255,
								(double)color_[i+1][2]/255);
			renderer->AddActor2D(textActor);
		}

		if (deleting_)
			style->setLeftButton(CLICK_DELETE);
		else
			style->setLeftButton(CLICK_EMPTY);
	}


	renderer->AddActor(actor);
	//renderer->SetBackground(nc->GetColor3d("MidnightBlue").GetData());
	style->SetDefaultRenderer(renderer);
	style->setNumberOfLabels(num_locations);
	style->setLabels(labels_);
	style->setRefLabels(labels_ref_);
	style->setLocations(loc_idx_);
	style->setColors(color_);
	renWinInter->SetInteractorStyle( style );
	renWin->Render();
	renWinInter->Start();

	labels_ = style->getLabels();
	loc_idx_ = style->getLocations();
}

void showConnection(
	Graph Graph_,
	vector<point_d> points_,
	vector<string> &labels_,
	vector<vector<unsigned char> > color_,
	bool show_points)
{
	// [VARIABLES]*************************************************************
	vector<node_tt> nodes 					= Graph_.getNodeList();
	vector<vector<vector<edge_tt> > > edges = Graph_.getListOfEdges();

	int num_locations = nodes.size();

	vector<point_d> locations; locations.resize(num_locations);
	for(int i=0;i<num_locations;i++)
	{
		locations[i] = nodes[i].centroid;
	}

	vtkSmartPointer<vtkLineSource> 				lineSource;
	vtkSmartPointer<vtkPolyDataMapper> 			lineMapper;
	vtkSmartPointer<vtkActor> 					lineActor;
	vtkSmartPointer<vtkPoints> 					points;
	vtkSmartPointer<vtkCellArray> 				lines;
	vtkSmartPointer<vtkPolyData> 				polyData;
	vtkSmartPointer<vtkDoubleArray> 			tubeRadius;
	vtkSmartPointer<vtkTubeFilter> 				tubeFilter;
	vtkSmartPointer<vtkPolyDataMapper> 			tubeMapper;
	vtkSmartPointer<vtkActor> 					tubeActor;
	vtkSmartPointer<vtkUnsignedCharArray> 		colors;
	vtkSmartPointer<vtkRenderer> 				renderer;
	vtkSmartPointer<vtkRenderWindow> 			renWin;
	vtkSmartPointer<vtkRenderWindowInteractor> 	renWinInter;
	vtkSmartPointer<customMouseInteractorStyle> style;
	// *************************************************************[VARIABLES]

	style 		= vtkSmartPointer<customMouseInteractorStyle>::New();
	renderer    = vtkSmartPointer<vtkRenderer>::New();
	renWin      = vtkSmartPointer<vtkRenderWindow>::New();
	renWinInter = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	renWin->SetSize(1280,800); //(width, height)
	renWin->AddRenderer(renderer);
	renWinInter->SetRenderWindow(renWin);

	vtkSmartPointer<vtkPoints> pointspoly;
	vtkSmartPointer<vtkPolygon> polygon;
	vtkSmartPointer<vtkCellArray> polygons;
	vtkSmartPointer<vtkPolyData> polygonPolyData;
	vtkSmartPointer<vtkPolyDataMapper> mapperpoly;
	vtkSmartPointer<vtkActor> actorpoly;

	for(int i=0;i<Sqr(num_locations);i++)
	{
		if(i/num_locations==i%num_locations) continue;
		if(edges[i/num_locations][i%num_locations][0].counter==0) continue;

		// [SECTOR POLYGON]****************************************************
		pointspoly = vtkSmartPointer<vtkPoints>::New();
		polygons = vtkSmartPointer<vtkCellArray>::New();
		polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
		mapperpoly = vtkSmartPointer<vtkPolyDataMapper>::New();
		actorpoly = vtkSmartPointer<vtkActor>::New();

		int loc = LOC_INT;
		int sec = SEC_INT;
		int n1  = i/num_locations;
		int n2  = i%num_locations;

		point_d tmpN[2], Nmax[2], init; init.x = init.y = init.z = 0;
//		init = sector[i][0].loc_start[0];

		vtkIdType ID[4];

		for(int l=0;l<loc;l++)
		{
			tmpN[0] =
					rodriguezVec((double)(2*M_PI*(0)/sec),
							edges[n1][n2][0].tan[l],
							edges[n1][n2][0].nor[l]);
			Nmax[0] =
					multiPoint(
							tmpN[0],
							edges[n1][n2][0].sector_map[l*sec+0]);
			for (int s=0;s<sec;s++)
			{
				int s_tmp = (s+1)%sec;
				tmpN[s_tmp%2] =
						rodriguezVec((double)(2*M_PI*(s_tmp)/sec),
								edges[n1][n2][0].tan[l],
								edges[n1][n2][0].nor[l]);
				Nmax[s_tmp%2] =
						multiPoint(
								tmpN[s_tmp%2],
								edges[n1][n2][0].sector_map[l*sec+s]);


//				// [TANGENT NORMAL LINES PER SECTOR]***************************
//				if(l<5)
//				{
//				lineSource = vtkSmartPointer<vtkLineSource>::New();
//				lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//				lineActor  = vtkSmartPointer<vtkActor>::New();
//				lineSource->SetPoint1(
//						edges[n1][n2][0].loc_start[l].x - init.x,
//						edges[n1][n2][0].loc_start[l].y - init.y,
//						edges[n1][n2][0].loc_start[l].z - init.z);
//				lineSource->SetPoint2(
//						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
//						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
//						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
//				lineMapper->SetInputConnection(lineSource->GetOutputPort());
//				lineActor->GetProperty()->SetLineWidth(5);
//				lineActor->GetProperty()->SetColor(1.0,0.0,1.0);
//				lineActor->SetMapper(lineMapper);
//				renderer->AddActor(lineActor);
//				lineSource = vtkSmartPointer<vtkLineSource>::New();
//				lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//				lineActor  = vtkSmartPointer<vtkActor>::New();
//				lineSource->SetPoint1(
//						edges[n1][n2][0].loc_start[l].x + edges[n1][n2][0].nor[l].x*0.1 - init.x,
//						edges[n1][n2][0].loc_start[l].y + edges[n1][n2][0].nor[l].y*0.1 - init.y,
//						edges[n1][n2][0].loc_start[l].z + edges[n1][n2][0].nor[l].z*0.1 - init.z);
//				lineSource->SetPoint2(
//						edges[n1][n2][0].loc_start[l].x - init.x,
//						edges[n1][n2][0].loc_start[l].y - init.y,
//						edges[n1][n2][0].loc_start[l].z - init.z);
//				lineMapper->SetInputConnection(lineSource->GetOutputPort());
//				lineActor->GetProperty()->SetLineWidth(5);
//				lineActor->GetProperty()->SetColor(1.0,0.0,0.0);
//				lineActor->SetMapper(lineMapper);
//				renderer->AddActor(lineActor);
//				}
//				if(l==3)
//				{
//					lineSource = vtkSmartPointer<vtkLineSource>::New();
//					lineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//					lineActor  = vtkSmartPointer<vtkActor>::New();
//					lineSource->SetPoint1(-0.19178,-0.208788,1.60406);
////					lineSource->SetPoint1(-0.182012,-0.218845,1.61503);
////					lineSource->SetPoint1(-0.187477,-0.221765,1.62085);
//					lineSource->SetPoint2(
//							edges[n1][n2][0].loc_mid[l].x - init.x,
//							edges[n1][n2][0].loc_mid[l].y - init.y,
//							edges[n1][n2][0].loc_mid[l].z - init.z);
//					lineMapper->SetInputConnection(lineSource->GetOutputPort());
//					lineActor->GetProperty()->SetLineWidth(5);
//					lineActor->GetProperty()->SetColor(1.0,1.0,0.0);
//					lineActor->SetMapper(lineMapper);
//					renderer->AddActor(lineActor);
//				}
//				// ***************************[TANGENT NORMAL LINES PER SECTOR]

				pointspoly->InsertPoint(
						(l*sec+s)*8+0,
						edges[n1][n2][0].loc_start[l].x + Nmax[s%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+1,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+2,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+3,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);
				ID[0] = (l*sec+s)*8+0;
				ID[1] = (l*sec+s)*8+1;
				ID[2] = (l*sec+s)*8+3;
				ID[3] = (l*sec+s)*8+2;
				polygons->InsertNextCell(4,ID);

				pointspoly->InsertPoint(
						(l*sec+s)*8+4,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+5,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);
				Nmax[s_tmp%2] =
						multiPoint(
								tmpN[s_tmp%2],
								edges[n1][n2][0].sector_map[l*sec+s_tmp]);
				pointspoly->InsertPoint(
						(l*sec+s)*8+6,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+7,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);
				ID[0] = (l*sec+s)*8+4;
				ID[1] = (l*sec+s)*8+5;
				ID[2] = (l*sec+s)*8+7;
				ID[3] = (l*sec+s)*8+6;
				polygons->InsertNextCell(4,ID);
			}
		}

		polygonPolyData->SetPoints(pointspoly);
		polygonPolyData->SetPolys(polygons);

#if VTK_MAJOR_VERSION <= 5
		mapperpoly->SetInput(polygonPolyData);
#else
		mapperpoly->SetInputData(polygonPolyData);
#endif

		actorpoly->SetMapper(mapperpoly);
		actorpoly->GetProperty()->SetColor(0.0,1.0,0.0);
//		actorpoly->GetProperty()->LightingOff();
//		actorpoly->SetOrigin(p_mid_tmp.x,p_mid_tmp.y,p_mid_tmp.z);
		renderer->AddActor(actorpoly);
		// ****************************************************[SECTOR POLYGON]

		// [SECTOR POLYGON]****************************************************
		pointspoly = vtkSmartPointer<vtkPoints>::New();
		polygons = vtkSmartPointer<vtkCellArray>::New();
		polygonPolyData = vtkSmartPointer<vtkPolyData>::New();
		mapperpoly = vtkSmartPointer<vtkPolyDataMapper>::New();
		actorpoly = vtkSmartPointer<vtkActor>::New();

		for(int l=0;l<loc;l++)
		{
			tmpN[0] =
					rodriguezVec(
							(double)(2*M_PI*(0)/sec),
							edges[n1][n2][0].tan[l],
							edges[n1][n2][0].nor[l]);
			Nmax[0] = multiPoint(tmpN[0],
								 edges[n1][n2][0].sector_const[l*sec+0]);
			for (int s=0;s<sec;s++)
			{
				int s_tmp = (s+1)%sec;
				tmpN[s_tmp%2] = rodriguezVec((double)(2*M_PI*(s_tmp)/sec),
										  edges[n1][n2][0].tan[l],
										  edges[n1][n2][0].nor[l]);
				Nmax[s_tmp%2] =
						multiPoint(
								tmpN[s_tmp%2],
								edges[n1][n2][0].sector_const[l*sec+s]);

				pointspoly->InsertPoint(
						(l*sec+s)*8+0,
						edges[n1][n2][0].loc_start[l].x + Nmax[s%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+1,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+2,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+3,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);
				ID[0] = (l*sec+s)*8+0;
				ID[1] = (l*sec+s)*8+1;
				ID[2] = (l*sec+s)*8+3;
				ID[3] = (l*sec+s)*8+2;
				polygons->InsertNextCell(4,ID);

				pointspoly->InsertPoint(
						(l*sec+s)*8+4,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+5,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);
				Nmax[s_tmp%2] =
						multiPoint(
								tmpN[s_tmp%2],
								edges[n1][n2][0].sector_const[l*sec+s_tmp]);
				pointspoly->InsertPoint(
						(l*sec+s)*8+6,
						edges[n1][n2][0].loc_start[l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_start[l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_start[l].z + Nmax[s_tmp%2].z - init.z);
				pointspoly->InsertPoint(
						(l*sec+s)*8+7,
						edges[n1][n2][0].loc_end  [l].x + Nmax[s_tmp%2].x - init.x,
						edges[n1][n2][0].loc_end  [l].y + Nmax[s_tmp%2].y - init.y,
						edges[n1][n2][0].loc_end  [l].z + Nmax[s_tmp%2].z - init.z);

				ID[0] = (l*sec+s)*8+4;
				ID[1] = (l*sec+s)*8+5;
				ID[2] = (l*sec+s)*8+7;
				ID[3] = (l*sec+s)*8+6;

				polygons->InsertNextCell(4,ID);
			}
		}

		polygonPolyData->SetPoints(pointspoly);
		polygonPolyData->SetPolys(polygons);

#if VTK_MAJOR_VERSION <= 5
		mapperpoly->SetInput(polygonPolyData);
#else
		mapperpoly->SetInputData(polygonPolyData);
#endif

		actorpoly->SetMapper(mapperpoly);
		actorpoly->GetProperty()->SetColor(1.0,1.0,0.0);
//		actorpoly->GetProperty()->LightingOff();
//		actorpoly->SetOrigin(p_mid_tmp.x,p_mid_tmp.y,p_mid_tmp.z);
		renderer->AddActor(actorpoly);
		// ****************************************************[SECTOR POLYGON]
	}

	// [ADDING DATA]***********************************************************
	if (show_points)
	{
		vtkSmartPointer<vtkTextActor> 		textActor;
		vtkSmartPointer<vtkActor> 			actor;
		actor  		= vtkSmartPointer<vtkActor>::New();
		actor->SetMapper(dataPoints(points_, num_locations, color_, true));
		actor->GetProperty()->SetPointSize(5);
		renderer->AddActor(actor);
		style->setNumberOfLabels(num_locations);
		style->setLabels(labels_);
		style->setColors(color_);
		for(int i=0;i<num_locations;i++)
		{
			textActor 	= vtkSmartPointer<vtkTextActor>::New();
			if(labels_.empty()) continue;
			if(labels_[i].empty()) continue;
			textActor->SetInput(labels_[i].c_str());
			textActor->SetPosition(10, (WIN_HEIGHT-20)-i*FONT_SIZE);
			textActor->GetTextProperty()->SetFontSize(FONT_SIZE);
			textActor->GetTextProperty()
					 ->SetColor((double)color_[i+1][0]/255,
								(double)color_[i+1][1]/255,
								(double)color_[i+1][2]/255);
			renderer->AddActor2D(textActor);
		}
	}
	// ***********************************************************[ADDING DATA]

	style->SetDefaultRenderer(renderer);
	style->setLeftButton(CLICK_EMPTY);
	renderer->SetBackground(0.0,0.0,0.0);
	renWinInter->SetInteractorStyle(style);
	renWin->Render();
	renWinInter->Start();
}

void showConnectionTest(
	vector<point_d> points_,
	vector<string> &labels_,
	Graph Graph_,
	vector<vector<unsigned char> > color_,
	bool show_points)
{
	vector<node_tt> nodes 			= Graph_.getNodeList();
	sector_para_t sector_para 		= Graph_.getSectorPara();
	vector<vector<edge_tt> > sector = Graph_.getEdgeList();

	double orientation[3];

	int num_locations = nodes.size();

	vector<point_d> locations; locations.resize(num_locations);
	for(int i=0;i<num_locations;i++) {locations[i] = nodes[i].centroid;}


	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
	vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkDoubleArray> tubeRadius = vtkSmartPointer<vtkDoubleArray>::New();
	vtkSmartPointer<vtkTubeFilter> tube = vtkSmartPointer<vtkTubeFilter>::New();
	vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
	vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

for(int iii=0;iii<20;iii++)
{
	points = vtkSmartPointer<vtkPoints>::New();
	lines = vtkSmartPointer<vtkCellArray>::New();
	polyData = vtkSmartPointer<vtkPolyData>::New();
	tubeRadius = vtkSmartPointer<vtkDoubleArray>::New();
	tube = vtkSmartPointer<vtkTubeFilter>::New();
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor = vtkSmartPointer<vtkActor>::New();

	// Create points and cells for the spiral
	for(int i = 0; i < 1; i++)
	{
		points->InsertPoint(i*2+0,
			sector[1][0].loc_start[i+iii].x ,
			sector[1][0].loc_start[i+iii].y,
			sector[1][0].loc_start[i+iii].z);
		points->InsertPoint(i*2+1,
			sector[1][0].loc_end[i+iii].x ,
			sector[1][0].loc_end[i+iii].y,
			sector[1][0].loc_end[i+iii].z);
	}

//	for(int ii = 0; ii < 5; ii++)
//	{
//	cout << sector[1][0].loc_start[ii].x << " " <<
//			sector[1][0].loc_start[ii].y << " " <<
//			sector[1][0].loc_start[ii].z << " " <<
//			sector[1][0].loc_end[ii].x << " " <<
//			sector[1][0].loc_end[ii].y << " " <<
//			sector[1][0].loc_end[ii].z << " " << endl;
//	}

	lines->InsertNextCell(2);
	for (int i = 0; i < 2; i++)
	{
		lines->InsertCellPoint(i);
	}

	polyData->SetPoints(points);
	polyData->SetLines(lines);

	// Varying tube radius using sine-function
	tubeRadius->SetName("TubeRadius");
	tubeRadius->SetNumberOfTuples(2);
	for (int i=0 ;i<2 ; i++)
	{
		tubeRadius->SetTuple1(i, 0.01);
	}
	polyData->GetPointData()->AddArray(tubeRadius);
	polyData->GetPointData()->SetActiveScalars("TubeRadius");


	#if VTK_MAJOR_VERSION <= 5
	  tube->SetInput(polyData);
	#else
	  tube->SetInputData(polyData);
	#endif
	  tube->SetNumberOfSides(10);
	  tube->SetVaryRadiusToVaryRadiusByAbsoluteScalar();


	mapper->SetInputConnection(tube->GetOutputPort());
	mapper->ScalarVisibilityOn();
	mapper->SetScalarModeToUsePointFieldData();

	actor->SetMapper(mapper);


	renderer->AddActor(actor);

}

// [ADDING DATA]***********************************************************
if (show_points)
{
	vtkSmartPointer<vtkPolyDataMapper> 	mapper;
	vtkSmartPointer<vtkActor> 			actor;
	mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
	actor  = vtkSmartPointer<vtkActor>::New();
	mapper =  dataPoints(points_, num_locations, color_, true);
	actor->SetMapper(mapper);
	actor->GetProperty()->SetPointSize(5);
	renderer->AddActor(actor);
}
// ***********************************************************[ADDING DATA]


	renderer->SetBackground(.2, .3, .4);

	vtkSmartPointer<vtkRenderWindow> renWin =
	vtkSmartPointer<vtkRenderWindow>::New();
	vtkSmartPointer<vtkRenderWindowInteractor>
	iren = vtkSmartPointer<vtkRenderWindowInteractor>::New();

	iren->SetRenderWindow(renWin);
	renWin->AddRenderer(renderer);
	renWin->SetSize(500, 500);
	renWin->Render();

	vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
	vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
	iren->SetInteractorStyle(style);

	iren->Start();

}

void plotData(
	vector<double> x,
	vector<double> y)
{
	vtkSmartPointer<vtkTable> 		table;
	vtkSmartPointer<vtkFloatArray> 	arrX;
	vtkSmartPointer<vtkFloatArray> 	arrY;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> 	chart;

	table =	vtkSmartPointer<vtkTable>::New();
	arrX  = vtkSmartPointer<vtkFloatArray>::New();
	arrY  = vtkSmartPointer<vtkFloatArray>::New();
	view  = vtkSmartPointer<vtkContextView>::New();
	chart =	vtkSmartPointer<vtkChartXY>::New();

	// Create a table with some points in it
	arrX->SetName("X Axis");
	arrY->SetName("Y Axis");
	table->AddColumn(arrX);
	table->AddColumn(arrY);

	// Fill in the table with values
	int numPoints = x.size();
	table->SetNumberOfRows(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
	}

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1280,800); //(width, height)

	// Add multiple line plots, setting the colors etc
	view->GetScene()->AddItem(chart);

	vtkPlot *line;
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
  line->SetInput(table, 0, 1);
#else
  line->SetInputData(table, 0, 1);
#endif
  line->SetColor(255, 0, 0, 255);
  line->SetWidth(2.0);

//  // For dotted line, the line type can be from 2 to 5 for different dash/dot
//  // patterns (see enum in vtkPen containing DASH_LINE, value 2):
//#ifndef WIN32
//  line->GetPen()->SetLineType(vtkPen::DASH_LINE);
//#endif
//  // (ifdef-ed out on Windows because DASH_LINE does not work on Windows
//  //  machines with built-in Intel HD graphics card...)
//
//  //view->GetRenderWindow()->SetMultiSamples(0);

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}

void plotData(
	vector<double> x,
	vector<double> y,
	vector<double> x2,
	vector<double> y2)
{
	vtkSmartPointer<vtkTable> 		table;
	vtkSmartPointer<vtkFloatArray> 	arrX;
	vtkSmartPointer<vtkFloatArray> 	arrY;
	vtkSmartPointer<vtkFloatArray> 	arrY2;
	vtkSmartPointer<vtkContextView> view;
	vtkSmartPointer<vtkChartXY> 	chart;

	table =	vtkSmartPointer<vtkTable>::New();
	arrX  = vtkSmartPointer<vtkFloatArray>::New();
	arrY  = vtkSmartPointer<vtkFloatArray>::New();
	arrY2 = vtkSmartPointer<vtkFloatArray>::New();
	view  = vtkSmartPointer<vtkContextView>::New();
	chart =	vtkSmartPointer<vtkChartXY>::New();

	// Create a table with some points in it
	arrX->SetName("X Axis");
	arrY->SetName("Y Axis");
	arrY2->SetName("Y2 Axis");
	table->AddColumn(arrX);
	table->AddColumn(arrY);
	table->AddColumn(arrY2);

	// Fill in the table with values
	int numPoints = x.size();
	table->SetNumberOfRows(numPoints);
	for (int i = 0; i < numPoints; ++i)
	{
		table->SetValue(i, 0, x[i]);
		table->SetValue(i, 1, y[i]);
		table->SetValue(i, 2, y2[i]);
	}

	// Set up the view
	view->GetRenderer()->SetBackground(1.0, 1.0, 1.0);
	view->GetRenderer()->GetRenderWindow()->SetSize(1280,800); //(width, height)

	// Add multiple line plots, setting the colors etc
	view->GetScene()->AddItem(chart);

	vtkPlot *line;
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
	line->SetInput(table, 0, 1);
#else
	line->SetInputData(table, 0, 1);
#endif
	line->SetColor(255, 0, 0, 255);
	line->SetWidth(2.0);
	line = chart->AddPlot(vtkChart::LINE);
#if VTK_MAJOR_VERSION <= 5
	line->SetInput(table, 0, 2);
#else
	line->SetInputData(table, 0, 2);
#endif
	line->SetColor(0, 255, 0, 255);
	line->SetWidth(12.0);

//  // For dotted line, the line type can be from 2 to 5 for different dash/dot
//  // patterns (see enum in vtkPen containing DASH_LINE, value 2):
//#ifndef WIN32
//  line->GetPen()->SetLineType(vtkPen::DASH_LINE);
//#endif
//  // (ifdef-ed out on Windows because DASH_LINE does not work on Windows
//  //  machines with built-in Intel HD graphics card...)
//
//  //view->GetRenderWindow()->SetMultiSamples(0);

	// Start interactor
	view->GetInteractor()->Initialize();
	view->GetInteractor()->Start();
}