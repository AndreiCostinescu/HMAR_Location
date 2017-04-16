/*
 * readWriteFIle.cpp
 *
 *  Created on: Mar 14, 2017
 *      Author: chen
 */

#include "readWriteFile.h"

bool copyFile(
	string SRC,
	string DEST)
{
	ifstream src (SRC,  ios::binary);
	ofstream dest(DEST, ios::binary);
	dest << src.rdbuf();
	return src && dest;
}

bool directoryCheck(
	string path_ )
{
	if (path_.empty())
	{
		return false;
	}
	DIR *dir;
	bool exist = false;
	dir = opendir(path_.c_str());
	if (dir != NULL)
	{
		exist = true; closedir(dir);
	}
	if (!exist)
	{
		mkdir(path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	}
	return exist;
}

int folderSelect1(
	const struct dirent *entry)
{
	if (entry->d_name[2] == '_')	{return 1;}
	else							{return 0;}
}

int folderSelect2(
	const struct dirent *entry)
{
	size_t found_extension = string(entry->d_name).find(".");
	if ((int)found_extension == -1) {return 1;}
	else							{return 0;}
}

int fileSelect(
	const struct dirent *entry)
{
	size_t found_extension = string(entry->d_name).find(".txt");
	if ((int)found_extension == -1) {return 0;}
	else							{return 1;}
}

int readFile(
	const char *name,
	vector<vector<string> > &data_,
	char delimiter)
{
	ifstream src_file( name );
	while (src_file)
	{
		string file_line_;
		if (!getline( src_file, file_line_ )) break;
		istringstream line_( file_line_ );
		vector <string> data_line_;
		while (line_)
		{
		   string word;
		   if (!getline( line_, word, delimiter)) break;
		   data_line_.push_back( word );
		}
		data_.push_back( data_line_ );
	}

	if (!src_file.eof()) {return EXIT_FAILURE;}
	else 				 {return EXIT_SUCCESS;}
}

void writeFile(
	Graph *Graph_,
	const char *path_,
	int option_)
{
	switch(option_)
	{
		case 0:
			if (!ifstream(path_))
			{
				vector<node_tt> node_tmp = Graph_->getNodeList();
				ofstream write_file(path_, ios::app);
				for(int i=0;i<node_tmp.size();i++)
				{
					write_file << node_tmp[i].name      	<< ","
							   << node_tmp[i].centroid.x	<< ","
							   << node_tmp[i].centroid.y 	<< ","
							   << node_tmp[i].centroid.z 	<< ","
							   << node_tmp[i].centroid.l 	<< ","
							   << node_tmp[i].surface 		<< ","
							   << node_tmp[i].contact;
					write_file << "\n";
				}
			}
			else
			{
				remove(path_);
				vector<node_tt> node_tmp = Graph_->getNodeList();
				ofstream write_file(path_, ios::app);
				for(int i=0;i<node_tmp.size();i++)
				{
					write_file << node_tmp[i].name      	<< ","
							   << node_tmp[i].centroid.x	<< ","
							   << node_tmp[i].centroid.y 	<< ","
							   << node_tmp[i].centroid.z 	<< ","
							   << node_tmp[i].centroid.l;
					write_file << "\n";
				}
			}
			break;
		case 1:
			if (!ifstream(path_))
			{
				int num_location = Graph_->getNodeList().size();
				vector<double> sec;
				vector<point_d> data;
				vector<vector<vector<edge_tt> > > edges = Graph_->getListOfEdges();

				ofstream write_file(path_, ios::app);
				write_file << "Locations," 			<< num_location << "\n";
				write_file << "Location Intervals," << LOC_INT		<< "\n";
				write_file << "Sector Intervals," 	<< SEC_INT		<< "\n";
				for(int i=0;i<edges.size();i++)
				{
					for(int ii=0;ii<edges[i].size();ii++)
					{

						for(int iii=0;iii<edges[i][ii].size();iii++)
						{
							write_file << i << "," << ii << "," << iii << "\n";
							int tmp = 0;
							while (tmp>=0)
							{
								data.clear(); sec.clear();
								switch (tmp)
								{
									case 0:
										data = edges[i][ii][0].loc_start;
										break;
									case 1:
										data = edges[i][ii][0].loc_mid;
										break;
									case 2:
										data = edges[i][ii][0].loc_end;
										break;
									case 3:
										data = edges[i][ii][0].tan;
										break;
									case 4:
										data = edges[i][ii][0].nor;
										break;
									case 5:
										write_file << Graph_->getEdgeCounter(i,ii,0) << "\n";
										break;
									case 6:
										sec = edges[i][ii][0].sector_map;
										for(int iv=0;iv<sec.size();iv++)
										{
											write_file << sec[iv];
											if (iv < sec.size()-1)	{ write_file << ",";  }
											else 					{ write_file << "\n"; }
										}
										tmp = -2;
										break;
								}
								tmp++;
								if (tmp > 4) {continue;}
								for(int iv=0;iv<LOC_INT;iv++)
								{
									write_file << data[iv].x << ",";
									write_file << data[iv].y << ",";
									write_file << data[iv].z;
									if (iv<data.size()-1) 	{ write_file << ",";  }
									else 					{ write_file << "\n"; }
								}
							}
						}
					}
				}
			}
			break;
		default:
			break;
//			if (!ifstream(path_))
//			{
//				int num_location = Graph_->getNodeList().size();
//				vector<double> sec;
//				vector<point_d> data;
//				vector<vector<vector<edge_tt> > > edges = Graph_->getListOfEdges();
//
//				ofstream write_file(path_, ios::app);
//				write_file << "Locations," 			<< num_location << "\n";
//				write_file << "Location Intervals," << LOC_INT		<< "\n";
//				write_file << "Sector Intervals," 	<< SEC_INT		<< "\n";
//				for(int i=0;i<edges.size();i++)
//				{
//					for(int ii=0;ii<edges[i].size();ii++)
//					{
//						data.clear(); sec.clear();
//						write_file << "Edge,"    << i
//								   << ",Number," << ii << "\n";
//						switch (option_)
//						{
//							case 10:
//								data = edges[i][ii][0].loc_start;
//								break;
//							case 11:
//								data = edges[i][ii][0].loc_mid;
//								break;
//							case 12:
//								data = edges[i][ii][0].loc_end;
//								break;
//							case 13:
//								data = edges[i][ii][0].tan;
//								break;
//							case 14:
//								data = edges[i][ii][0].nor;
//								break;
//							case 15:
//								write_file << Graph_->getEdgeCounter(i,ii,0) << "\n";
//								break;
//							case 16:
//								sec = edges[i][ii][0].sector_map;
//								for(int iii=0;iii<sec.size();iii++)
//								{
//									write_file << sec[iii];
//									if (iii < sec.size()-1)	{ write_file << ",";  }
//									else 					{ write_file << "\n"; }
//								}
//								break;
//							case 17:
//								sec = edges[i][ii][0].sector_const;
//								for(int iii=0;iii<sec.size();iii++)
//								{
//									write_file << sec[iii];
//									if (iii < sec.size()-1)	{ write_file << ",";  }
//									else 					{ write_file << "\n"; }
//								}
//								break;
//						}
//						if (option_ > 14) {continue;}
//						for(int iii=0;iii<LOC_INT;iii++)
//						{
//							write_file << data[iii].x << ",";
//							write_file << data[iii].y << ",";
//							write_file << data[iii].z;
//							if (iii<data.size()-1) 	{ write_file << ",";  }
//							else 					{ write_file << "\n"; }
//						}
//					}
//				}
//			}
	}
}

int readFileExt(
	Graph *Graph_,
	const char *path_,
	int option_)
{
	vector<vector<string> > data;
	switch(option_)
	{
		case 0:
		{
			readFile(path_, data , ',');
			if (!data.empty())
			{
				node_tt node_tmp = {};
				for(int i=0;i<data.size();i++)
				{
					if (Graph_->getNode(i,node_tmp)==EXIT_SUCCESS)
					{return EXIT_SUCCESS;}
					node_tmp.name 		=      data[i][0];
					node_tmp.centroid.x	= atof(data[i][1].c_str());
					node_tmp.centroid.y	= atof(data[i][2].c_str());
					node_tmp.centroid.z	= atof(data[i][3].c_str());
					node_tmp.centroid.l	= atof(data[i][4].c_str());
					node_tmp.index    	= i;
					node_tmp.surface 	= atoi(data[i][5].c_str());
					node_tmp.contact 	= atoi(data[i][6].c_str());
					Graph_->setNode(node_tmp);
					Graph_->addEmptyEdgeForNewNode(i);
//					Graph_->expandFilter(i+1);
				}
//				for(int i=0;i<data.size();i++) { Graph_->updateFilter(i); }
				printer(6);
			}
			else
			{
				printer(7);
				return EXIT_FAILURE;
			}
			break;
		}
		case 1:
		{
			readFile(path_, data , ',');
			if (!data.empty())
			{
				vector<int>	obj_mask;
				for(int i=3;i<data.size();i++)
				{
					obj_mask.push_back(atoi(data[i][0].c_str()));
				}
				Graph_->setInitFilter(obj_mask);
				printer(4);
			}
			else
			{
				printer(5);
				return EXIT_FAILURE;
			}
			break;
		}
		case 2:
		{
			readFile(path_, data , ',');
			if (!data.empty())
			{
				map<string,pair<int,int> >	action_cat;
				vector<string> 				action_label;
				for(int i=0;i<data.size();i++)
				{
					if (data[i].size()>1)
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
				Graph_->setActionCategory(action_cat);
				Graph_->setActionLabel(action_label);
				Graph_->initFilter();
				printer(2);
			}
			else
			{
				printer(3);
				return EXIT_FAILURE;
			}
			break;
		}
		case 3:
		{
			pair<string,string> tmp(".",".");
			readFile(path_, data , ',');
			if (!data.empty())
			{
				map<string,pair<string,string> > obj_label;
				for(int i=0;i<data.size();i++)
				{
					tmp.first	= data[i][1];
					tmp.second	= data[i][2];
					obj_label[data[i][0]] = tmp;
				}
				Graph_->setObjectLabel(obj_label);
				printer(4);
			}
			else
			{
				printer(5);
				return EXIT_FAILURE;
			}
			break;
		}
		case 4:
		{
			readFile(path_, data , ',');
			if (!data.empty())
			{
				int num_locations, num_location_intervals, num_sector_intervals;
				num_locations 			= atoi(data[0][1].c_str());
				num_location_intervals 	= atoi(data[1][1].c_str());
				num_sector_intervals 	= atoi(data[2][1].c_str());

				int c,l1,l2,l3; c=l1=l2=l3=0;
				for(int i=3;i<data.size();i++)
				{
					c=(i-3)%8;
					edge_tt edge_tmp = Graph_->getEdge(l1,l2,l3);
					switch (c)
					{
						case 0:
							l1 = atoi(data[i][0].c_str());
							l2 = atoi(data[i][1].c_str());
							l3 = atoi(data[i][2].c_str());
							break;
						case 1:
							for(int iii=0;iii<num_location_intervals;iii++)
							{
								edge_tmp.loc_start[iii].x = atof(data[i][iii*3+0].c_str());
								edge_tmp.loc_start[iii].y = atof(data[i][iii*3+1].c_str());
								edge_tmp.loc_start[iii].z = atof(data[i][iii*3+2].c_str());
							}
						case 2:
							for(int iii=0;iii<num_location_intervals;iii++)
							{
								edge_tmp.loc_mid[iii].x = atof(data[i][iii*3+0].c_str());
								edge_tmp.loc_mid[iii].y = atof(data[i][iii*3+1].c_str());
								edge_tmp.loc_mid[iii].z = atof(data[i][iii*3+2].c_str());
							}
							break;
						case 3:
							for(int iii=0;iii<num_location_intervals;iii++)
							{
								edge_tmp.loc_end[iii].x = atof(data[i][iii*3+0].c_str());
								edge_tmp.loc_end[iii].y = atof(data[i][iii*3+1].c_str());
								edge_tmp.loc_end[iii].z = atof(data[i][iii*3+2].c_str());
							}
							break;
						case 4:
							for(int iii=0;iii<num_location_intervals;iii++)
							{
								edge_tmp.tan[iii].x = atof(data[i][iii*3+0].c_str());
								edge_tmp.tan[iii].y = atof(data[i][iii*3+1].c_str());
								edge_tmp.tan[iii].z = atof(data[i][iii*3+2].c_str());
							}
							break;
						case 5:
							for(int iii=0;iii<num_location_intervals;iii++)
							{
								edge_tmp.nor[iii].x = atof(data[i][iii*3+0].c_str());
								edge_tmp.nor[iii].y = atof(data[i][iii*3+1].c_str());
								edge_tmp.nor[iii].z = atof(data[i][iii*3+2].c_str());
							}
							break;
						case 6:
							edge_tmp.counter = atoi(data[i][0].c_str());
							break;
						case 7:
							for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
							{
								edge_tmp.sector_map[iii] = atof(data[i][iii].c_str());
							}
							break;
					}
					Graph_->setEdge(l1,l2,l3,edge_tmp);
				}
				printer(41);
			}
			else
			{
				printer(42);
			}
			break;
		}
		default:
			break;
//		{
//			readFile(path_, data , ',');
//			if (!data.empty())
//			{
//				int num_locations, num_location_intervals, num_sector_intervals;
//				num_locations 			= atoi(data[0][1].c_str());
//				num_location_intervals 	= atoi(data[1][1].c_str());
//				num_sector_intervals 	= atoi(data[2][1].c_str());
//
//				int c = 4;
//				for(int i=0;i<num_locations;i++)
//				{
//					for(int ii=0;ii<num_locations;ii++)
//					{
//						edge_tt edge_tmp = Graph_->getEdge(i,ii,0);
//						switch(option_)
//						{
//							case 10:
//								for(int iii=0;iii<num_location_intervals;iii++)
//								{
//									edge_tmp.loc_start[iii].x = atof(data[c][iii*3+0].c_str());
//									edge_tmp.loc_start[iii].y = atof(data[c][iii*3+1].c_str());
//									edge_tmp.loc_start[iii].z = atof(data[c][iii*3+2].c_str());
//								}
//								break;
//							case 11:
//								for(int iii=0;iii<num_location_intervals;iii++)
//								{
//									edge_tmp.loc_mid[iii].x = atof(data[c][iii*3+0].c_str());
//									edge_tmp.loc_mid[iii].y = atof(data[c][iii*3+1].c_str());
//									edge_tmp.loc_mid[iii].z = atof(data[c][iii*3+2].c_str());
//								}
//								break;
//							case 12:
//								for(int iii=0;iii<num_location_intervals;iii++)
//								{
//									edge_tmp.loc_end[iii].x = atof(data[c][iii*3+0].c_str());
//									edge_tmp.loc_end[iii].y = atof(data[c][iii*3+1].c_str());
//									edge_tmp.loc_end[iii].z = atof(data[c][iii*3+2].c_str());
//								}
//								break;
//							case 13:
//								for(int iii=0;iii<num_location_intervals;iii++)
//								{
//									edge_tmp.tan[iii].x = atof(data[c][iii*3+0].c_str());
//									edge_tmp.tan[iii].y = atof(data[c][iii*3+1].c_str());
//									edge_tmp.tan[iii].z = atof(data[c][iii*3+2].c_str());
//								}
//								break;
//							case 14:
//								for(int iii=0;iii<num_location_intervals;iii++)
//								{
//									edge_tmp.nor[iii].x = atof(data[c][iii*3+0].c_str());
//									edge_tmp.nor[iii].y = atof(data[c][iii*3+1].c_str());
//									edge_tmp.nor[iii].z = atof(data[c][iii*3+2].c_str());
//								}
//								break;
//							case 15:
//								edge_tmp.counter = atoi(data[c][0].c_str());
//								break;
//							case 16:
//								for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
//								{
//									edge_tmp.sector_map[iii] = atof(data[c][iii].c_str());
//								}
//								break;
//							case 17:
//								for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
//								{
//									edge_tmp.sector_const[iii] = atof(data[c][iii].c_str());
//								}
//								break;
//						}
//						Graph_->setEdge(i,ii,0,edge_tmp);
//						c += 2;
//					}
//				}
//				printer(41);
//			}
//			else
//			{
//				printer(42);
//			}
//		}
	}
	return EXIT_SUCCESS;
}

int readFileLabel(
	string path_,
	map<int,vector<string> > &label_)
{
	vector<vector<string> > data;
	readFile((path_ + "label.txt").c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=0;i<data.size();i++)
		{
			vector<string> tmp;
			for(int ii=1;ii<data[i].size();ii++)
			{
				tmp.push_back(data[i][ii]);
			}
			label_[atoi(data[i][0].c_str())] = tmp;
		}
		printer(37);
	}
	else
	{
		printer(36);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}

int readFileKB(
	string path_,
	kb_t &kb_)
{
	vector<vector<string> > data;
	readFile((path_ + "surface.txt").c_str(), data , ',');
	if (!data.empty())
	{
		kb_.surface.resize(data.size());
		kb_.surface_eq.resize(data.size());
		for(int i=0;i<data.size();i++)
		{
			for(int ii=1;ii<5;ii++)
			{
				kb_.surface_eq[i].push_back(atof(data[i][ii].c_str()));
			}
			kb_.surface[i].x = atof(data[i][5].c_str());
			kb_.surface[i].y = atof(data[i][6].c_str());
			kb_.surface[i].z = atof(data[i][7].c_str());
			kb_.surface[i].l = UNCLASSIFIED;
		}
		printer(39);
	}
	else
	{
		printer(38);
		return EXIT_FAILURE;
	}

	data.clear();
	readFile((path_ + "surface_limit.txt").c_str(), data , ',');
	if (!data.empty())
	{
		kb_.surface_lim.resize(data.size());
		for(int i=0;i<data.size();i++)
		{
			kb_.surface_lim[i] = atof(data[i][0].c_str());
		}
		printer(39);
	}
	else
	{
		printer(38);
		return EXIT_FAILURE;
	}

	data.clear();
	readFile((path_ + "action_label.txt").c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=0;i<data.size();i++)
		{
			if (data[i].size()>1)
			{
				pair<int,int> tmp_pair(
						atoi(data[i][1].c_str()),
						atoi(data[i][2].c_str()));
				kb_.ac[data[i][0]] = tmp_pair;
			}
			else
			{
				kb_.al.push_back(data[i][0]);
			}
		}
		printer(2);
	}
	else
	{
		printer(3);
		return EXIT_FAILURE;
	}

	pair<string,string> tmp(".",".");
	data.clear();
	readFile((path_ + "obj_action_label.txt").c_str(), data , ',');
	if (!data.empty())
	{
		for(int i=0;i<data.size();i++)
		{
			tmp.first	= data[i][1];
			tmp.second	= data[i][2];
			kb_.ol[data[i][0]] = tmp;
		}
		printer(4);
	}
	else
	{
		printer(5);
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
