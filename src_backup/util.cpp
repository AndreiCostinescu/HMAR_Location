/*
 * util.cpp
 *
 *  Created on: Jan 6, 2017
 *      Author: chen
 */

//#define DBSCAN

#include "util.h"

#ifdef PC
	string SCENE_ = "../Scene/";
#else
	string SCENE_ = "Scene/";
#endif

// ============================================================================
// Data
// ============================================================================

void parseData2Point(
	vector<vector<string> > data_,
	vector<point_d> 		&points_,
	vector<int> 			&contact_)
{
	reshapeVector(points_, data_.size());
	reshapeVector(contact_,data_.size());
	for(int i=0;i<data_.size();i++)
	{
		contact_[i]   = atoi(data_[i][1].c_str());
		points_ [i].x = atof(data_[i][2].c_str());
		points_ [i].y = atof(data_[i][3].c_str());
		points_ [i].z = atof(data_[i][4].c_str());
		points_ [i].l = UNCLASSIFIED;
	}
}

void preprocessDataLive(
	point_d pos_,
	vector< vector< point_d > > &pva_mem_, // motion -> length(empty at beginning)
	vector<point_d> &pva_avg_, //motion
	unsigned int window_)
{
	point_d vel = minusPoint(pos_, pva_avg_[0]);
	point_d acc = minusPoint(vel , pva_avg_[1]);
	vector<point_d> tmp; tmp.resize(3);
	tmp[0] = pos_; tmp[1] = vel; tmp[2] = acc;
	for(int i=0;i<3;i++)
	{
		if(pva_mem_[i].size() == window_)
		{
			pva_avg_[i] = movingAverage(tmp[i], pva_mem_[i]);
		}
		else if (pva_mem_[i].size()>0)
		{
			pva_avg_[i] = averagePointIncrement(tmp[i], pva_mem_[i]);
		}
		else
		{
			pva_mem_[i].push_back(tmp[i]);
			pva_avg_[i] = tmp[i];
			for(int ii=i+1;ii<3;ii++)
			{
				pva_avg_[ii].x =
				pva_avg_[ii].y =
				pva_avg_[ii].z =
				pva_avg_[ii].l = 0.0;
			}
			break;
		}
		if(i==0)	{ pva_avg_[i].l = UNCLASSIFIED;	}
		else		{ pva_avg_[i].l = 0.0;			}
	}
}


// ============================================================================
// Modules
// ============================================================================

int learnSector(
	string dirname_,
	string scene,
	string object)
{
	// [VARIABLES]*************************************************************
	string path;
	vector<int> 				file_eof;
	vector<int> 				contact;
	vector<int> 				contact_mem;
	vector<int> 				last_loc;
	map<string,pair<int,int> >	action_cat;
	vector<string> 				action_label;
	vector<vector<string> > 	data;
	vector<point_d> 			points;
	vector<point_d> 			curve_mem;
	vector<point_d> 			pva_avg1; pva_avg1.resize(3);
	vector<vector<point_d> > 	pva_avg; // length->motion
	vector<vector<point_d> > 	pva_mem; // motion->length
	vector<vector<int> >		filter_mask;
	vector<int>					obj_mask;
	predict_t					predict;
	Graph 						Graph_main(scene, object);
	printer(1);
	// *************************************************************[VARIABLES]

	// [ACTION LABELS]*********************************************************
	path =  SCENE_ + "/action_label.txt";
	data.clear();
	readFile(path.c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=0;i<data.size();i++)
		{
			if (i<3)
			{
				pair<int,int> tmp_pair(
						atoi(data[i][1].c_str()),
						atoi(data[i][2].c_str()));
				action_cat[data[i][0]] = tmp_pair;
			}
			else
			{
				action_label.push_back(data[i][0]);
			}
		}
		Graph_main.setActionCategory(action_cat);
		Graph_main.setActionLabel(action_label);
		Graph_main.initFilter();
	}
	else
	{
		printer(3);
		return EXIT_FAILURE;
	}
	printer(2);
	// *********************************************************[ACTION LABELS]

	// [OBJ MASK]**************************************************************
	path =  SCENE_ + scene + "/" + object + "/obj_action_label.txt";
	data.clear();
	readFile(path.c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=3;i<data.size();i++)
		{
			obj_mask.push_back(atoi(data[i][0].c_str()));
		}
		Graph_main.setInitFilter(obj_mask);
	}
	else
	{
		printer(5);
		return EXIT_FAILURE;
	}
	printer(4);
	// **************************************************************[OBJ MASK]

//	// [FILTER MASK]***********************************************************
//	path =  SCENE_ + scene + "/" + object + "/filter_mask.txt";
//	data.clear();
//	readFile(path.c_str(), data , ',');
//	if (!data.empty())
//	{
//		filter_mask.resize(data[0].size());
//		for(int i=0;i<data.size();i++)
//		{
//			for(int ii=0;ii<data[i].size();ii++)
//			{
//				filter_mask[ii][i] = atoi(data[i][ii].c_str());
//			}
//		}
//		// options : c_cur, c_vel, c_sur, c_win
//		Graph_main.setFilter("c_cur", filter_mask[0]);
//		Graph_main.setFilter("c_vel", filter_mask[1]);
//		Graph_main.setFilter("c_sur", filter_mask[2]);
//		Graph_main.setFilter("c_win", filter_mask[3]);
//	}
//	// ***********************************************************[FILTER MASK]

	// [SAVED LOCATION]********************************************************
	path =  SCENE_ + scene + "/" + object + "/data_loc.txt";
	data.clear();
	readFile(path.c_str(), data , ',');
	if (!data.empty())
	{
		node_tt node_tmp = {};
		for(int i=0;i<data.size();i++)
		{
			node_tmp.name 		=      data[i][0];
			node_tmp.centroid.x	= atof(data[i][1].c_str());
			node_tmp.centroid.y	= atof(data[i][2].c_str());
			node_tmp.centroid.z	= atof(data[i][3].c_str());
			node_tmp.centroid.l	= atof(data[i][4].c_str());
			node_tmp.index    	= i;
			Graph_main.setNode(node_tmp);
			Graph_main.addEmptyEdgeForNewNode(i);
			Graph_main.expandFilter(i+1);
		}
		for(int i=0;i<data.size();i++)
		{
			Graph_main.updateFilter(i);
		}
		printer(6);
	}
	else
	{
		printer(7);
	}
	// ********************************************************[SAVED LOCATION]

	// [READ FILE]*************************************************************
	data.clear();
	struct dirent **namelist;
	string name;
	int n = scandir(dirname_.c_str(), &namelist, fileSelect, alphasort);
	if (n == 0) return 0;
	for(int i=0;i<n;i++)
	{
		name = dirname_ + namelist[i]->d_name;
		readFile(name.c_str(), data , ',');
		file_eof.push_back(data.size());
	}
	printer(8);
	// *************************************************************[READ FILE]

	// [PARSE DATA]************************************************************
	parseData2Point(data, points, contact);
	contact_mem = contact;
	for(int i=30;i<contact.size();i++)
	{
		contact[i] =
				(int)(
						(float)accumulate(
								contact_mem.begin()+i-30,
								contact_mem.begin()+i, 0) / 30);
	}
	printer(9);
	// ************************************************************[PARSE DATA]

	// [LOOP THROUGH FILES]****************************************************
	int tmp_id1 = 0;
	for(int i=0;i<file_eof.size();i++)
	{
		// ====================================================================
		// TESTING (ONLINE PREDICTION)
		// ====================================================================
		if (i==file_eof.size()-1)
		{

			path =  SCENE_ + scene + "/" + object + "/data_loc.txt";
			writeFile(Graph_main, path, 0);

			printf("\n******************************************************************************\n");
			printf("* TESTING START                                                              *\n");
			printf("******************************************************************************\n");

			vector<point_d> point_tmp(
					points.begin() + tmp_id1,
					points.begin() + file_eof[i]-1);
			int p1 = point_tmp.size();
			reshapeVector(pva_avg, p1);
			reshapeVector(pva_mem, 3);
			reshapeVector(last_loc, Graph_main.getNumberOfNodes());
			reshapePredict(predict, Graph_main.getNumberOfNodes());
			bool flag_last=false;
			double pow_dec = 1.0;
			int label1 = -1;
			Graph Graph_update = Graph_main;
			vector<double> x;
			vector<vector<double> > y; y.resize(10);
			for(int ii=0;ii<p1;ii++)
			{
				if (ii == 0) 	{ pva_avg[ii] = pva_avg1;		}
				else			{ pva_avg[ii] = pva_avg[ii-1];	}
				preprocessDataLive(
						points[ii+tmp_id1], pva_mem, pva_avg[ii], FILTER_WIN);
				predictAction(
						Graph_main,
						Graph_update,
						contact[ii+tmp_id1],
						pva_avg[ii],
						curve_mem,
						predict,
						label1,
						last_loc,
						false);



				map<string,pair<int,int> > ac_tmp;
				vector<string> al_tmp;
				vector<map<string, double> > f_tmp;
				Graph_main.getActionCategory(ac_tmp);
				Graph_main.getActionLabel(al_tmp);
				Graph_main.getPrediction(f_tmp);


				if (f_tmp.size()>0)
				{
					x.push_back(ii);
					for(int iii=0;iii<Graph_main.getNumberOfNodes();iii++)
					{
//						y[iii].push_back(f_tmp[iii]["THROW"]);
//						y[iii].push_back(f_tmp[iii]["THROW"]);
//						y[iii].push_back(f_tmp[iii]["CUT"]);
//						y[iii].push_back(f_tmp[iii]["CLEAN"]);
//						y[iii].push_back(f_tmp[iii]["SCAN"]);
						y[iii].push_back(f_tmp[iii]["WINDOW"]);
//						y[iii].push_back(f_tmp[iii]["SLIDE"]);
//						y[iii].push_back(f_tmp[iii]["CURVE"]);
//						y[iii].push_back(f_tmp[iii]["MOVE"]);
//						y[iii].push_back(f_tmp[iii]["STOP"]);
					}
				}



			}
			plotData(x,y[0]);
			plotData(x,y[1]);

//			vector<point_d> point_zero; vector<string> label_zero;
//			for(int ii=0;ii<pva_avg.size();ii++) point_zero.push_back(pva_avg[ii][0]);
//			vector<vector<unsigned char> > color_code; colorCode(color_code);
//			showConnection(Graph_update, point_zero, label_zero, color_code, true);

			tmp_id1 = file_eof[i];
			printf("******************************************************************************\n");
			printf("* TESTING END                                                                *\n");
			printf("******************************************************************************\n\n");
		}
		// ====================================================================
		// TRAINING
		// ====================================================================
		else
		{
			// [PREPROCESS DATA]***********************************************
			vector<point_d> point_dmp(
					points.begin() + tmp_id1,
					points.begin() + file_eof[i]-1);
			reshapeVector(pva_avg, point_dmp.size());
			reshapeVector(pva_mem, 3);
			for(int ii=0;ii<point_dmp.size();ii++)
			{
				if (ii == 0) 	{ pva_avg[ii] = pva_avg1;		}
				else			{ pva_avg[ii] = pva_avg[ii-1];	}
				preprocessDataLive(
						points[ii+tmp_id1], pva_mem, pva_avg[ii], FILTER_WIN);
			}
			printer(10);
			// ***********************************************[PREPROCESS DATA]
			// [LOCATION AND SECTOR-MAP]***************************************
			buildLocationArea(Graph_main, pva_avg, contact);
			printer(11);
			buildSectorMap   (Graph_main, pva_avg, contact);
			printer(12);
			// ***************************************[LOCATION AND SECTOR-MAP]
			tmp_id1 = file_eof[i];
		}
	}







//	directoryCheck(SCENE_ + scene + "/" + object);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_beg.txt";
//	writeLearnedDataFile(Graph, path, 0);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_mid.txt";
//	writeLearnedDataFile(Graph, path, 1);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_end.txt";
//	writeLearnedDataFile(Graph, path, 2);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_tangent.txt";
//	writeLearnedDataFile(Graph, path, 3);
//	path = 	SCENE_ + scene + "/" + object + "/loc_data_normal.txt";
//	writeLearnedDataFile(Graph, path, 4);
//	path = 	SCENE_ + scene + "/" + object + "/counter.txt";
//	writeLearnedDataFile(Graph, path, 5);
//	path = 	SCENE_ + scene + "/" + object + "/sec_data_max.txt";
//	writeLearnedDataFile(Graph, path, 6);
//	path = 	SCENE_ + scene + "/" + object + "/sec_data_const.txt";
//	writeLearnedDataFile(Graph, path, 7);
//	printf("Creating sectors for connection between the clusters (action locations)......Complete\n");
	// *************************************************** [LOOP THROUGH FILES]
	return EXIT_SUCCESS;
}

// ============================================================================
// EXTRAS
// ============================================================================

void outputMsg(
	msg_t MSG_,
	Graph Graph_)
{
	switch(MSG_.msg)
	{
		case 1 :
			// 1. message for prediction during motion.
			if (VERBOSE == 0 || VERBOSE == 1)
			{
				printf("Nr:%04d,  ", MSG_.idx);
				printf("LABEL : ");
				if (MSG_.label.mov < 0)
				{
					printf("NULL ");
				}
				else if (MSG_.label.mov == 1)
				{
					for(int ii=0;ii<MSG_.num_sur;ii++)
					{
						if (MSG_.label.sur[ii] > 0)
						{
							printf("%s on surface %d  ",
									Graph_.getMovLabel()[MSG_.label.mov].c_str(),
									ii);
							break;
						}
					}
				}
				else
				{
					printf("%s  ",
							Graph_.getMovLabel()[MSG_.label.mov].c_str());
				}
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					printf(" %.4f ", MSG_.pred.pred_err[ii]);
				}
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					if (MSG_.pred.pred[ii] == RANGE_IN)
					{
						printf(" %s %.4f ",
								Graph_.getNode(ii).name.c_str(),
								MSG_.pred.pred_in[ii]);
					}
				}
				printf("\n");
			}
			break;

		case 2 :
			// 2. message for prediction for location areas.
			if (VERBOSE == 0 || VERBOSE == 2)
			{
				printf("Nr:%04d,  ", MSG_.idx);
				printf("LABEL : ");
				for(int ii=0;ii<MSG_.num_loc;ii++)
				{
					if (MSG_.label.loc[ii] > 0)
					{
						if (MSG_.label.mov < 0)
						{
							printf("NULL ");
						}
						else if (MSG_.label.mov == 1)
						{
							for(int ii=0;ii<MSG_.num_sur;ii++)
							{
								if (MSG_.label.sur[ii] > 0)
								{
									printf("%s on surface %d ",
											Graph_.getMovLabel()
												[MSG_.label.mov].c_str(),
											ii);
									break;
								}
							}
						}
						else
						{
							printf("%s ",
									Graph_.getMovLabel()
										[MSG_.label.mov].c_str());
						}

						printf("%s ",
								Graph_.getNode(ii).name.c_str());
						break;
					}
					else if (MSG_.label.loc[ii] < 0)
					{
						printf("Empty location Label.  ");
						if (MSG_.label.mov < 0)
						{
							printf("NULL ");
						}
						else if (MSG_.label.mov == 1)
						{
							for(int ii=0;ii<MSG_.num_sur;ii++)
							{
								if (MSG_.label.sur[ii] > 0)
								{
									printf("%s on surface %d ",
											Graph_.getMovLabel()
												[MSG_.label.mov].c_str(),
											ii);
									break;
								}
							}
						}
						else
						{
							printf("%s ",
									Graph_.getMovLabel()
										[MSG_.label.mov].c_str());
						}
						break;
					}
				}
				printf("\n");
			}
			break;

		case 3:
			// 3. LABEL ONLY MESSSAGE
			if (VERBOSE == 3)
			{
				if (MSG_.loc_idx < 0)
				{
					printf("Nr:%04d,  ", MSG_.idx);
					printf("LABEL : ");
					if (MSG_.label.mov < 0)
					{
						printf("NULL ");
					}
					else if (MSG_.label.mov == 1)
					{
						for(int ii=0;ii<MSG_.num_sur;ii++)
						{
							if (MSG_.label.sur[ii] > 0)
							{
								printf("%s on surface %d ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str(),
										ii);
								break;
							}
						}
					}
					else
					{
						printf("%s ",
								Graph_.getMovLabel()[MSG_.label.mov].c_str());
					}
					if (*max_element(
							MSG_.pred.pred_in.begin(),
							MSG_.pred.pred_in.end()) > 0)
					{
						unsigned int tmptmp =
								distance(
										MSG_.pred.pred_in.begin(),
										max_element(
												MSG_.pred.pred_in.begin(),
												MSG_.pred.pred_in.end()));
						printf("%s  %.4f  ",
								Graph_.getNode(tmptmp).name.c_str(),
								*max_element(
										MSG_.pred.pred_in.begin(),
										MSG_.pred.pred_in.end()));
					}
					else
					{
						unsigned int tmptmp =
								distance(
										MSG_.pred.pred_err.begin(),
										max_element(
												MSG_.pred.pred_err.begin(),
												MSG_.pred.pred_err.end()));
						printf("%s  %.4f  ",
								Graph_.getNode(tmptmp).name.c_str(),
								*max_element(
										MSG_.pred.pred_err.begin(),
										MSG_.pred.pred_err.end()));
					}
					printf("\n");
				}
				else
				{
					printf("Nr:%04d,  ", MSG_.idx);
					printf("LABEL : ");
					for(int ii=0;ii<MSG_.num_loc;ii++)
					{
						if (MSG_.label.loc[ii] > 0)
						{
							if (MSG_.label.mov < 0)
							{
								printf("NULL ");
							}
							else if (MSG_.label.mov == 1)
							{
								for(int ii=0;ii<MSG_.num_sur;ii++)
								{
									if (MSG_.label.sur[ii] > 0)
									{
										printf("%s on surface %d ",
												Graph_.getMovLabel()
													[MSG_.label.mov].c_str(),
												ii);
										break;
									}
								}
							}
							else
							{
								printf("%s ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str());
							}

							printf("%s ",
									Graph_.getNode(ii).name.c_str());
							break;
						}
						else if (MSG_.label.loc[ii] < 0)
						{
							printf("Empty location Label.  ");
							if (MSG_.label.mov < 0)
							{
								printf("NULL ");
							}
							else if (MSG_.label.mov < 0)
							{
								for(int ii=0;ii<MSG_.num_sur;ii++)
								{
									if (MSG_.label.sur[ii] > 0)
									{
										printf("%s on surface %d ",
												Graph_.getMovLabel()
													[MSG_.label.mov].c_str(),
												ii);
										break;
									}
								}
							}
							else
							{
								printf("%s ",
										Graph_.getMovLabel()
											[MSG_.label.mov].c_str());
							}
							break;
						}
					}
					printf("\n");
				}
			}
			break;
	}
}





//---------------------------------------------------------------------------------------------------------------------



//---------------------------------------------------------------------------------------------------------------------
// General Functions

void writePointFile(
	point_d *p,
	unsigned int num_points)
{
	remove("data.txt");
	for(unsigned int i=0;i<num_points;i++)
	{
		// write values into data.txt
		std::ofstream write_file("data.txt", std::ios::app);
		write_file << p[i].x << ","
				   << p[i].y << ","
				   << p[i].z << ","
				   << p[i].l
				   << "\n";
	}
}

bool checkSurfaceRange(
	point_d pos_,
	point_d pos_surface_,
	vector<double> surface_,
	double surface_limit_,
	double surface_range_limit_)
{
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceRange(pos_, pos_surface_, surface_) < surface_range_limit_)
			return true;
		else
			return false;
	else
		return false;
}

int checkMoveSlide(
	point_d pos_,
	point_d vel_,
	vector<double> surface_,
	double surface_limit_,
	double angle_limit_)
{
	cout << surfaceDistance(pos_, surface_) << "  ";
	cout << surfaceAngle(vel_, surface_) << endl;
	if(surfaceDistance(pos_, surface_) < surface_limit_) // less than 10 cm off the surface
		if(surfaceAngle(vel_, surface_) > angle_limit_)
			return 1;
		else
			return 0;
	else
		return 0;
}

double checkMoveSlideOutside(
		point_d pos_,
		point_d vel_,
		double **surface_,
		unsigned int num_surfaces_)
{
	vector<double> A; A.resize(3);
	double dir_tmp = INFINITY, dist_tmp = INFINITY;
	for(int i=0;i<num_surfaces_;i++)
	{
		A[0] = surface_[i][0];
		A[1] = surface_[i][1];
		A[2] = surface_[i][2];
		dir_tmp  = l2Norm(crossProduct(A,point2vector(vel_)));
		dist_tmp = surface_[i][0]*pos_.x +
						  surface_[i][1]*pos_.y +
						  surface_[i][2]*pos_.z -
						  surface_[i][3];
//		cout << dir_tmp << " " << dist_tmp << " ";
	}
	return 0.0;
}


/*
// UNUSED---------------------------------------------------------------------------------------------------------------------
void writeSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > sector_,
	int num_locations_,
	int num_location_intervals_,
	int num_sector_intervals_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << num_locations_ 		  << "\n";
		write_file << num_location_intervals_ << "\n";
		write_file << num_sector_intervals_   << "\n";
		for(int i=0;i<sector_.size();i++)
		{
			for(int ii=0;ii<sector_[i].size();ii++)
			{
				for(int iii=0;iii<sector_[i][ii].size();iii++)
				{
					write_file << sector_[i][ii][iii];
					if (i<sector_[i][ii].size()-1)
						write_file << ",";
				}
				write_file << "\n";
			}
		}
	}
}

void readSectorConstraintFile(
	string path_,
	vector<vector<vector<double> > > &sector_)
{
	vector<vector<string> > data;
	readFile(path_.c_str(), data , ',');

	if(data.empty())
	{
		printf("[WARNING] : Sector data is empty.");
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data[0][0].c_str());
		num_location_intervals 	= atoi(data[1][0].c_str());
		num_sector_intervals 	= atoi(data[2][0].c_str());

	    reshapeVector(sector_,Sqr(num_locations));
	    vector<vector<double> > sector1(num_location_intervals);
		       vector<double> 	sector2(num_sector_intervals);

		for(int i=0;i<Sqr(num_locations);i++)
		{
			sector_[i] = sector1;
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				sector_[i][ii] = sector2;
			}
		}

		for(int i=0;i<num_locations;i++)
		{
			for(int ii=0;ii<num_location_intervals;ii++)
			{
				int tmp = i*num_location_intervals+ii+3;
				for(int iii=0;iii<num_sector_intervals;iii++)
				{
					sector_[i][ii][iii] = atof(data[tmp][iii].c_str());
				}
			}
		}
	}
	// Should we add the option to edit the data?
}

void writeLocLabelFile(
	Graph Graph_,
	string path_)
{
	if (!ifstream(path_))
	{
		vector<node_tt> node_tmp = Graph_.getNodeList();
		ofstream write_file(path_, ios::app);
		for(int i=0;i<node_tmp.size();i++)
		{
			write_file << node_tmp[i].name       << ","
					   << node_tmp[i].location.x << ","
					   << node_tmp[i].location.y << ","
					   << node_tmp[i].location.z << ","
					   << node_tmp[i].boundary   << ","
					   << node_tmp[i].surface	 << ","
					   << node_tmp[i].surface_boundary;
			write_file << "\n";
		}
	}
}

void writeMovLabelFile(
	string path_,
	vector<string> label_)
{
	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		for(int i=0;i<label_.size();i++)
		{
			write_file << label_[i];
			if (i<label_.size()-1)
				write_file << ",";
		}
		write_file << "\n";
	}
}

void writeCounterFile(
	Graph Graph_,
	string path_,
	int type_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				write_file << Graph_.getCounter(i,ii)
						   << "\n";
			}
		}
	}
}

void writeSectorFile(
	Graph Graph_,
	string path_,
	int type_)
{
	vector<node_tt> nodes = Graph_.getNodeList();
	int num_locations = nodes.size();
	vector<vector<edge_tt> > edges = Graph_.getEdgeList();

	sector_para_t para = Graph_.getSectorPara();

	if (!ifstream(path_))
	{
		ofstream write_file(path_, ios::app);
		write_file << "Locations," 			<< num_locations 	<< "\n";
		write_file << "Location Intervals," << para.loc_int 	<< "\n";
		write_file << "Sector Intervals," 	<< para.sec_int 	<< "\n";
		for(int i=0;i<edges.size();i++)
		{
			for(int ii=0;ii<edges[i].size();ii++)
			{
				vector<double> sector       = edges[i][ii].sector_map;
				vector<double> sector_const = edges[i][ii].sector_const;
				write_file << "Edge,"    << i
						   << ",Number," << ii << "\n";
				for(int iii=0;iii<sector.size();iii++)
				{
					switch(type_)
					{
						case 0:
							write_file << sector[iii];
							break;
						case 1:
							write_file << sector_const[iii];
							break;
					}
					if (iii<sector.size()-1)
						write_file << ",";
					else
						write_file << "\n";
				}
			}
		}
	}
}

*/




