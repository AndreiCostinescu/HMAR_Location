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

int ReadFile::ReadSurfaceFile(
	string path_,
	vector<Matrix3d> &rotation_,
	vector<Vector4d> &planeeq_,
	vector<Vector3d> &boxmin_,
	vector<Vector3d> &boxmid_,
	vector<Vector3d> &boxmax_)
{
	if (this->ReadFile_(path_, ',')==EXIT_FAILURE)
	{
		printer(38);
		return EXIT_FAILURE;
	}
	else
	{
		int c = 0, counter = 0;
		double tmp;
		Vector4d planeeq_tmp=Vector4d::Zero();
		Vector3d boxmin_tmp, boxmid_tmp, boxmax_tmp, boxmidrot_tmp, y_tmp(0,1,0);
		boxmin_tmp=boxmid_tmp=boxmax_tmp=Vector3d::Zero();
		for(int i=0;i<data_rf.size();i++)
		{
			if(c != atoi(data_rf[i][0].c_str()))
			{
				c = atoi(data_rf[i][0].c_str());
				tmp = planeeq_tmp[3];
				planeeq_tmp = V3d4d(V4d3d((planeeq_tmp/counter)).normalized());
				planeeq_tmp[3] = tmp/counter;
				boxmin_tmp = boxmin_tmp/counter;
				boxmax_tmp = boxmax_tmp/counter;
				boxmid_tmp = boxmid_tmp/counter;
				counter = 0;

				Matrix3d R = rodriguezRot(y_tmp, V4d3d(planeeq_tmp));
				rotation_.push_back(R);
				boxmidrot_tmp 	= R * boxmid_tmp;
				boxmin_tmp 		= R * boxmin_tmp - boxmidrot_tmp;
				boxmax_tmp 		= R * boxmax_tmp - boxmidrot_tmp;

				for(int xyz=0;xyz<3;xyz++)
				{
					tmp = boxmin_tmp[xyz];
					if (boxmax_tmp[xyz] < boxmin_tmp[xyz])
					{
						boxmin_tmp[xyz] = boxmax_tmp[xyz];
						boxmax_tmp[xyz] = tmp;
					}
					tmp = boxmax_tmp[xyz] - boxmin_tmp[xyz];
					boxmin_tmp[xyz] -= tmp/3.0;
					boxmax_tmp[xyz] += tmp/3.0;
				}

				planeeq_.push_back(planeeq_tmp);
				boxmin_.push_back(boxmin_tmp);
				boxmid_.push_back(boxmid_tmp);
				boxmax_.push_back(boxmax_tmp);

				planeeq_tmp=Vector4d::Zero();
				boxmin_tmp=boxmid_tmp=boxmax_tmp=Vector3d::Zero();
			}

			counter++;
			for(int ii=0;ii<4;ii++)
				planeeq_tmp[ii] += atof(data_rf[i][1+ii].c_str());
			for(int ii=0;ii<3;ii++)
				boxmid_tmp[ii] += atof(data_rf[i][5+ii].c_str());
			for(int ii=0;ii<3;ii++)
				boxmin_tmp[ii] += atof(data_rf[i][8+ii].c_str());
			for(int ii=0;ii<3;ii++)
				boxmax_tmp[ii] += atof(data_rf[i][11+ii].c_str());

			if(i==data_rf.size()-1)
			{
				tmp = planeeq_tmp[3];
				planeeq_tmp = V3d4d(V4d3d((planeeq_tmp/counter)).normalized());
				planeeq_tmp[3] = tmp/counter;
				boxmin_tmp = boxmin_tmp/counter;
				boxmax_tmp = boxmax_tmp/counter;
				boxmid_tmp = boxmid_tmp/counter;
				counter = 0;

				Matrix3d R = rodriguezRot(y_tmp, V4d3d(planeeq_tmp));
				rotation_.push_back(R);
				boxmidrot_tmp 	= R * boxmid_tmp;
				boxmin_tmp 		= R * boxmin_tmp - boxmidrot_tmp;
				boxmax_tmp 		= R * boxmax_tmp - boxmidrot_tmp;

				for(int xyz=0;xyz<3;xyz++)
				{
					tmp = boxmin_tmp[xyz];
					if (boxmax_tmp[xyz] < boxmin_tmp[xyz])
					{
						boxmin_tmp[xyz] = boxmax_tmp[xyz];
						boxmax_tmp[xyz] = tmp;
					}
					tmp = boxmax_tmp[xyz] - boxmin_tmp[xyz];
					boxmin_tmp[xyz] -= tmp/3.0;
					boxmax_tmp[xyz] += tmp/3.0;
				}

				planeeq_.push_back(planeeq_tmp);
				boxmin_.push_back(boxmin_tmp);
				boxmid_.push_back(boxmid_tmp);
				boxmax_.push_back(boxmax_tmp);

				planeeq_tmp=Vector4d::Zero();
				boxmin_tmp=boxmid_tmp=boxmax_tmp=Vector3d::Zero();
			}
		}
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
		kb_.surface_mid.resize(data_rf.size());
		kb_.surface_min.resize(data_rf.size());
		kb_.surface_max.resize(data_rf.size());
		kb_.surface_eq.resize(data_rf.size());
		kb_.surface_rot.resize(data_rf.size());
		for(int i=0;i<data_rf.size();i++)
		{
			for(int ii=0;ii<4;ii++)
				kb_.surface_eq[i][ii]  = atof(data_rf[i][1+ii].c_str());
			for(int ii=0;ii<3;ii++)
				kb_.surface_mid[i][ii] = atof(data_rf[i][5+ii].c_str());
			for(int ii=0;ii<3;ii++)
				kb_.surface_min[i][ii] = atof(data_rf[i][8+ii].c_str());
			for(int ii=0;ii<3;ii++)
				kb_.surface_max[i][ii] = atof(data_rf[i][11+ii].c_str());
			for(int ii=0;ii<3;ii++)
				for(int iii=0;iii<3;iii++)
					kb_.surface_rot[i](ii,iii) = atof(data_rf[i][14+(ii*3+iii)].c_str());
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
	vector<string> al_,
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
			node_tmp = Graph_->GetNode(i);
			if (!strcmp(node_tmp.name.c_str(),""))
			{
				node_tmp.name 		= al_[atof(data_rf[i][0].c_str())];
				node_tmp.index    	= i;
				for(int ii=0;ii<4;ii++)
					node_tmp.centroid[ii] = atof(data_rf[i][1+ii].c_str());
				node_tmp.surface 	= atoi(data_rf[i][5].c_str());
				for(int ii=0;ii<3;ii++)
					node_tmp.box_min[ii]  = atof(data_rf[i][6+ii].c_str());
				for(int ii=0;ii<3;ii++)
					node_tmp.box_max[ii]  = atof(data_rf[i][9+ii].c_str());
				node_tmp.contact 	= atoi(data_rf[i][12].c_str());
				Graph_->SetNode(node_tmp);
				Graph_->addEmptyEdgeForNewNode(i);
			}
			else
			{
				return EXIT_SUCCESS;
			}
		}
		printer(6);
		return EXIT_SUCCESS;
	}
}

int ReadFile::ReadFileLA(
		vector<vector<string> > &data_,
		string path_)
{
	if (this->ReadFile_(path_, ',')==EXIT_FAILURE)
	{
		return EXIT_FAILURE;
	}
	else
	{
		data_ = data_rf;
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
			c=(i-3)%7;
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
						for(int iv=0;iv<3;iv++)
						{
							edge_tmp.nor[iii][iv] = atof(data_rf[i][iii*3+iv].c_str());
						}
					}
					break;
				case 2:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						for(int iv=0;iv<3;iv++)
						{
							edge_tmp.tan[iii][iv] = atof(data_rf[i][iii*3+iv].c_str());
						}
					}
					break;
				case 3:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						for(int iv=0;iv<4;iv++)
						{
							edge_tmp.loc_mid[iii][iv] = atof(data_rf[i][iii*4+iv].c_str());
						}
					}
					break;
				case 4:
					for(int iii=0;iii<num_location_intervals;iii++)
					{
						edge_tmp.loc_len[iii] = atof(data_rf[i][iii].c_str());
					}
					break;
				case 5:
					edge_tmp.counter = atoi(data_rf[i][0].c_str());
					break;
				case 6:
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

