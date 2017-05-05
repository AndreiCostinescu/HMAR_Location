/*
 * ReadFile.cpp
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 */

#include "ReadFile.h"

ReadFile::ReadFile()
{
	n=nn=nnn=0;
	list0=list1=list2={};
	pair_tmp=make_pair(-1,"");
	data_rf.clear();
	idx.clear();
	map_tmp.clear();
}

ReadFile::~ReadFile() {}

int ReadFile::ReadFileName(
	string dir_name_,
	vector<int> idx_,
	map<int,map<int,pair<int,string> > > &file_list_,
	int &sub_num_)
{
	{
		n=nn=nnn=0;
		list0=list1=list2={};
		pair_tmp=make_pair(-1,"");
		idx.clear();
		map_tmp.clear();
	}

	// Reading the subject directory
	// Reading the action directory
	// Reading the individual action data file

	n = scandir(dir_name_.c_str(), &list0, folderSelect1, alphasort);
	if (n==0)	{printer(32); return EXIT_FAILURE;	}
	else		{printer(33);						}
	sub_num_ = n;

	for(int f=0;f<n;f++)
	{
		string dir_s = dir_name_ + "/" + list0[f]->d_name;

		nn = scandir(dir_s.c_str(), &list1, folderSelect2, alphasort);
		if (nn==0)	{printer(25); return EXIT_FAILURE;	}
		else		{printer(26);						}

		int c = 0;
		map_tmp.clear();
		idx.clear(); idx = idx_;

		for(int ff=0;ff<nn;ff++)
		{
			char num[4]; sprintf(num, "%03d", idx[0]);

			// takes only the action specify by idx
			if (strcmp(list1[ff]->d_name, num)) continue;

			dir_s = dir_name_ + "/" + list0[f]->d_name + "/" + list1[ff]->d_name;

			nnn = scandir(dir_s.c_str(), &list2, fileSelect, alphasort);
			for(int fff=0;fff<nnn;fff++)
			{
				pair_tmp.first 	= idx[0]; //start with 001
				pair_tmp.second = dir_s + "/" + list2[fff]->d_name;
				map_tmp[c] = pair_tmp;
				c += 1;
			}

			idx.erase(idx.begin());
		}

		file_list_[f] = map_tmp;
	}

	return EXIT_SUCCESS;
}

int ReadFile::ReadLabelFileName(
	string dir_name_,
	map<string, string> &label_list_)
{
	n=0; list0={};

	n = scandir(dir_name_.c_str(), &list0, fileSelect, alphasort);

	if (n==0)
	{
		return EXIT_FAILURE;
	}
	else
	{
		for(int f=0;f<n;f++)
		{
			string tmp = string(list0[f]->d_name);
			tmp.erase(tmp.begin()+(tmp.find(".")),tmp.end());
			label_list_[tmp] = dir_name_ + string(list0[f]->d_name);
		}
		return EXIT_SUCCESS;
	}
}

void ReadFile::ClearRF()
{
	n=nn=nnn=0;
	list0=list1=list2={};
	pair_tmp=make_pair(-1,"");
	data_rf.clear();
	idx.clear();
	map_tmp.clear();
}

bool ReadFile::CopyFile(
	string SRC,
	string DEST)
{
	ifstream src (SRC,  ios::binary);
	ofstream dest(DEST, ios::binary);
	dest << src.rdbuf();
	return src && dest;
}

bool ReadFile::DirectoryCheck(
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
		exist = true;
		closedir(dir);
	}
	if (!exist)
	{
		mkdir(path_.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	}
	return exist;
}

int ReadFile::ReadFile_(
	string path_,
	char delimiter)
{
	data_rf.clear();

	ifstream src_file(path_);
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
		data_rf.push_back( data_line_ );
	}

	if (!src_file.eof()) {return EXIT_FAILURE;}
	else 				 {return EXIT_SUCCESS;}
}

int ReadFile::ReadFileLabel(
	string path_,
	map<int,vector<string> > &label_)
{
	label_.clear();

	if (this->ReadFile_(path_ + "label.txt", ',')==EXIT_FAILURE)
	{
		printer(36);
		return EXIT_FAILURE;
	}
	else
	{
		for(n=0;n<data_rf.size();n++)
		{
			vector<string> tmp;
			for(int nn=1;nn<data_rf[n].size();nn++)
			{
				tmp.push_back(data_rf[n][nn]);
			}
			label_[atoi(data_rf[n][0].c_str())] = tmp;
		}
		printer(37);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileKB(
	string path_,
	kb_t &kb_)
{
	kb_ = {};

	if (this->ReadFile_(path_ + "surface.txt", ',')==EXIT_FAILURE)
	{
		printer(38);
		return EXIT_FAILURE;
	}
	else
	{
		kb_.surface.resize(data_rf.size());
		kb_.surface_eq.resize(data_rf.size());
		for(int i=0;i<data_rf.size();i++)
		{
			for(int ii=1;ii<5;ii++)
			{
				kb_.surface_eq[i].push_back(atof(data_rf[i][ii].c_str()));
			}
			kb_.surface[i].x = atof(data_rf[i][5].c_str());
			kb_.surface[i].y = atof(data_rf[i][6].c_str());
			kb_.surface[i].z = atof(data_rf[i][7].c_str());
			kb_.surface[i].l = UNCLASSIFIED;
		}
		printer(39);
	}

	if (this->ReadFile_(path_ + "surface_limit.txt", ',')==EXIT_FAILURE)
	{
		printer(45);
		return EXIT_FAILURE;
	}
	else
	{
		kb_.surface_lim.resize(data_rf.size());
		for(int i=0;i<data_rf.size();i++)
		{
			kb_.surface_lim[i] = atof(data_rf[i][1].c_str());
		}
		printer(46);
	}

	if (this->ReadFile_(path_ + "action_label.txt", ',')==EXIT_FAILURE)
	{
		printer(3);
		return EXIT_FAILURE;
	}
	else
	{
		for(int i=0;i<data_rf.size();i++)
		{
			if (data_rf[i].size()>1)
			{
				pair<int,int> tmp_pair(
						atoi(data_rf[i][1].c_str()),
						atoi(data_rf[i][2].c_str()));
				kb_.ac[data_rf[i][0]] = tmp_pair;
			}
			else
			{
				kb_.al.push_back(data_rf[i][0]);
			}
		}
		printer(2);
	}

	if (this->ReadFile_(path_ + "obj_action_label.txt", ',')==EXIT_FAILURE)
	{
		printer(5);
		return EXIT_FAILURE;
	}
	else
	{
		for(int i=0;i<data_rf.size();i++)
		{
			map<string,string> tmp2;
			for(int ii=0;ii<(data_rf[i].size()-1)/2;ii++)
			{
				tmp2[data_rf[i][ii*2+1]] = data_rf[i][ii*2+2];
			}
			kb_.ol[data_rf[i][0]] = tmp2;
		}
		printer(4);
	}
	return EXIT_SUCCESS;
}

int ReadFile::ReadFileLA(
	Graph *Graph_,
	string path_)
{
	if (this->ReadFile_(path_, ',')==EXIT_FAILURE)
	{
		printer(7);
		return EXIT_FAILURE;
	}
	else
	{
		node_tt node_tmp = {};
		for(int i=0;i<data_rf.size();i++)
		{
			if (Graph_->GetNode(i,node_tmp)==EXIT_SUCCESS)
			{return EXIT_SUCCESS;}
			node_tmp.name 		=      data_rf[i][0];
			node_tmp.centroid.x	= atof(data_rf[i][1].c_str());
			node_tmp.centroid.y	= atof(data_rf[i][2].c_str());
			node_tmp.centroid.z	= atof(data_rf[i][3].c_str());
			node_tmp.centroid.l	= atof(data_rf[i][4].c_str());
			node_tmp.index    	= i;
			node_tmp.surface 	= atoi(data_rf[i][5].c_str());
			node_tmp.contact 	= atoi(data_rf[i][6].c_str());
			Graph_->SetNode(node_tmp);
			Graph_->addEmptyEdgeForNewNode(i);
		}
		printer(6);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileGraph(
	Graph *Graph_,
	string path_)
{
	if (this->ReadFile_(path_, ',')==EXIT_FAILURE)
	{
		printer(42);
		return EXIT_FAILURE;
	}
	else
	{
		int num_locations, num_location_intervals, num_sector_intervals;
		num_locations 			= atoi(data_rf[0][1].c_str());
		num_location_intervals 	= atoi(data_rf[1][1].c_str());
		num_sector_intervals 	= atoi(data_rf[2][1].c_str());

		int c,l1,l2,l3; c=l1=l2=l3=0;
		for(int i=3;i<data_rf.size();i++)
		{
			c=(i-3)%8;
			edge_tt edge_tmp = Graph_->GetEdge(l1,l2,l3);
			switch (c)
			{
				case 0:
					l1 = atoi(data_rf[i][0].c_str());
					l2 = atoi(data_rf[i][1].c_str());
					l3 = atoi(data_rf[i][2].c_str());
					break;
				case 1:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.loc_start[iii].x = atof(data_rf[i][iii*3+0].c_str());
						edge_tmp.loc_start[iii].y = atof(data_rf[i][iii*3+1].c_str());
						edge_tmp.loc_start[iii].z = atof(data_rf[i][iii*3+2].c_str());
					}
					break;
				case 2:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.loc_mid[iii].x = atof(data_rf[i][iii*3+0].c_str());
						edge_tmp.loc_mid[iii].y = atof(data_rf[i][iii*3+1].c_str());
						edge_tmp.loc_mid[iii].z = atof(data_rf[i][iii*3+2].c_str());
					}
					break;
				case 3:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.loc_end[iii].x = atof(data_rf[i][iii*3+0].c_str());
						edge_tmp.loc_end[iii].y = atof(data_rf[i][iii*3+1].c_str());
						edge_tmp.loc_end[iii].z = atof(data_rf[i][iii*3+2].c_str());
					}
					break;
				case 4:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.tan[iii].x = atof(data_rf[i][iii*3+0].c_str());
						edge_tmp.tan[iii].y = atof(data_rf[i][iii*3+1].c_str());
						edge_tmp.tan[iii].z = atof(data_rf[i][iii*3+2].c_str());
					}
					break;
				case 5:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.nor[iii].x = atof(data_rf[i][iii*3+0].c_str());
						edge_tmp.nor[iii].y = atof(data_rf[i][iii*3+1].c_str());
						edge_tmp.nor[iii].z = atof(data_rf[i][iii*3+2].c_str());
					}
					break;
				case 6:
					edge_tmp.counter = atoi(data_rf[i][0].c_str());
					break;
				case 7:
					for(int iii=0;iii<num_location_intervals*num_sector_intervals;iii++)
					{
						edge_tmp.sector_map[iii] = atof(data_rf[i][iii].c_str());
					}
					break;
			}
			Graph_->SetEdge(l1,l2,l3,edge_tmp);
		}
		printer(41);
		return EXIT_SUCCESS;
	}
}

