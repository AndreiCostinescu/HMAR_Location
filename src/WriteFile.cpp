/*
 * WriteFile.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 *      Detail: Writing data to file.
 */

#include "WriteFile.h"

WriteFile::WriteFile() { }

WriteFile::~WriteFile() { }

void WriteFile::RewriteDataFileFilter(
		int curr_,
		int mem_,
		int mem2_,
		int mem3_,
		string &mem_s_,
		string &mem_s2_,
		string &mem_s3_,
		vector<string> &label_)
{
	// m - r - m
	if (!strcmp(label_.back().c_str(),mem_s2_.c_str()) 	&&
		!strcmp(label_.back().c_str(),"MOVE")	 		&&
		(curr_-mem_)<10)
	{
		for(int ii=mem_;ii<curr_+1;ii++)
		{
			label_[ii] = "MOVE";
		}
		mem_s_ = label_.back();
	}

	// r - x - r
	if (!strcmp(label_.back().c_str(),mem_s2_.c_str()) 	&&
		!strcmp(label_.back().c_str(),"RELEASE")	 	&&
		(curr_-mem_)<10)
	{
		for(int ii=mem_;ii<curr_+1;ii++)
		{
			label_[ii] = "RELEASE";
		}
		mem_s_ = label_.back();
	}

	// x - r - x
	if (!strcmp(label_.back().c_str(),mem_s2_.c_str()) 	&&
		!strcmp(mem_s_.c_str(),"RELEASE")		 	 	&&
		(curr_-mem_)<10)
	{
		for(int ii=mem_;ii<curr_+1;ii++)
		{
			label_[ii] = label_.back();
		}
		mem_s_ = label_.back();
	}
	// x - m - x
	if (!strcmp(label_.back().c_str(),mem_s2_.c_str()) 	&&
		 strcmp(mem_s_.c_str(),"RELEASE")		 	 	&&
		 strcmp(mem_s2_.c_str(),"RELEASE")		 	 	&&
		 strcmp(mem_s2_.c_str(),"MOVE"))
	{
		for(int ii=mem2_;ii<curr_+1;ii++)
		{
			label_[ii] = label_.back();
		}
		mem_s_ = label_.back();
	}

	// r - m - x
	if (!strcmp(mem_s_.c_str(),"MOVE") &&
		!strcmp(mem_s2_.c_str(),"RELEASE"))
	{
		for(int ii=mem_;ii<curr_+1;ii++)
		{
			label_[ii] = label_.back();
		}
		mem_s_ = label_.back();
	}

//	// x - m - r
//	if (!strcmp(mem_s_.c_str(),"MOVE") &&
//		!strcmp(label_.back().c_str(),"RELEASE"))
//	{
//		for(int ii=mem_;ii<curr_+1;ii++)
//		{
//			label_[ii] = label_[mem2_];
//		}
//		mem_s_ = label_[mem2_];
//	}
}

void WriteFile::RewriteDataFile(
	string path_,
	vector<vector<string> > data_,
	vector<Vector4d> points_,
	vector<int>	contact_,
	Vector4d face_,
	vector<Vector3d> label_ref_write_,
	vector<string> label_ref_name_,
	vector<string> label_list_)
{
	float face_lim = 0.20;
	int mem,mem2,mem3; mem=mem2=mem3=-1;
	string mem_s,mem_s2,mem_s3;  mem_s=mem_s2=mem_s3= "";
	vector<string> label, label_list_local;

	string tmp = path_;
	reverse(tmp.begin(),tmp.end());
	if (tmp.compare(tmp.find("/")+1,3,"200")==0) { face_lim = 0.25; }

	for(int i=0;i<points_.size();i++)
	{
		points_[i][3] = -1;
		if(contact_[i]==0)
		{
			label.push_back("RELEASE");
		}
		else
		{
			if(V4d3d(points_[i]-face_).norm()<face_lim)
			{
				points_[i][3] = 2.0;
				label.push_back("FACE");
			}
			else
			{
				bool flag = true;
				for(int ii=0;ii<label_ref_write_.size();ii++)
				{
					if((V4d3d(points_[i])-label_ref_write_[ii]).norm()<0.05)
					{
						points_[i][3] = 2.0;
						label.push_back(label_ref_name_[ii]);
						flag = false;
						break;
					}
				}
				if(flag)
				{
					label.push_back("MOVE");
				}
			}
		}
		if(strcmp(label.back().c_str(),mem_s.c_str()))
		{
			this->RewriteDataFileFilter(i,mem,mem2,mem3,mem_s,mem_s2,mem_s3,label);
			mem3 	= mem2;
			mem_s3 	= mem_s2;
			mem2 	= mem;
			mem_s2 	= mem_s;
			mem 	= i;
			mem_s 	= label.back();
		}
	}
	// x - m - r
	if (!strcmp(mem_s.c_str(),"RELEASE") &&
		!strcmp(mem_s2.c_str(),"MOVE"))
	{
		for(int i=mem2;i<mem+1;i++)
		{
			label[i] = mem_s3;
		}
	}

	remove(path_.c_str());
	ofstream write_file(path_.c_str(), ios::app);
	for(int i=0;i<points_.size();i++)
	{
		if(i==0)
		{
			label_list_local.push_back(label[i]);
		}
		else
		{
			if(strcmp(label[i].c_str(),label[i-1].c_str()))
			{
				label_list_local.push_back(label[i]);
			}
		}
		write_file 	<< data_[i][0] << ","
					<< data_[i][1] << ","
					<< data_[i][2] << ","
					<< data_[i][3] << ","
					<< data_[i][4] << ","
					<< data_[i][5] << ","
					<< data_[i][6] << ","
					<< data_[i][7] << ","
					<< label[i]   << "\n";
	}
/*
	// Visualize
	if (0)
	{
		vector<Vector4d> point_zero = points_;
		for(int ii=0;ii<point_zero.size();ii++)
		{
			if (!strcmp(label[ii].c_str(),"MOVE"))
				point_zero[ii][3] = -1;
			else if (!strcmp(label[ii].c_str(),"RELEASE"))
				point_zero[ii][3] = -1;
			else
				point_zero[ii][3] = 1;
		}
		vector<vector<unsigned char> > color_code; colorCode(color_code);
		vector<string>  goal_action, al; goal_action.resize(5);
		vector<int> 	loc_idx_zero;
		showData(
				point_zero, goal_action, al,
				loc_idx_zero, color_code, true, false, false);
	}
*/

	for(int i=0;i<label_list_.size();i++)
	{
		if(strcmp(label_list_[i].c_str(),label_list_local[i].c_str()))
		{
			cout << "*******************************************************************************" << endl;
			cout << "File with inconsistent label sequence : " << path_ << endl;
			cout << "*******************************************************************************" << endl;
			break;
		}
	}
}

//void WriteFile::WriteFileLA(
//	vector<string> line_,
//	vector<vector<string> > data_tmp,
//	string path_)
//{
//	bool flag = false;
//	if (!data_tmp.empty())
//	{
//		remove(path_.c_str());
//		ofstream write_file(path_, ios::app);
//		for(int i=0;i<data_tmp.size();i++)
//		{
//			if(!strcmp(data_tmp[i][0].c_str(),line_[0].c_str()))
//			{
//				flag = true;
//				for(int ii=0;ii<data_tmp[i].size()-1;ii++)
//				{
//					switch(ii)
//					{
//						case 1:
//							write_file << line_[ii] << ",";
//							break;
//						case 2:
//							write_file << line_[ii] << ",";
//							break;
//						case 3:
//							write_file << line_[ii] << ",";
//							break;
//						default:
//							write_file << data_tmp[i][ii] << ",";
//							break;
//					}
//					write_file << data_tmp[i][ii] << "\n";
//				}
//			}
//			else
//			{
//				for(int ii=0;ii<data_tmp[i].size();ii++)
//				{
//					if (ii==data_tmp[i].size()-1)
//						write_file << data_tmp[i][ii] << "\n";
//					else
//						write_file << data_tmp[i][ii] << ",";
//				}
//			}
//		}
//		if(!flag)
//		{
//			for(int ii=0;ii<line_.size();ii++)
//			{
//				if (ii==line_.size()-1)
//					write_file << line_[ii] << "\n";
//				else
//					write_file << line_[ii] << ",";
//			}
//		}
//	}
//}

void WriteFile::WriteFileLA(
	CGraph *Graph_,
	CKB *kb_,
	string path_)
{
	if (ifstream(path_)) { remove(path_.c_str()); }

	vector<CGraph::node_t> node_tmp = Graph_->GetNodeList();

	ofstream write_file(path_, ios::app);

	for(int i=0;i<node_tmp.size();i++)
	{
		for(int ii=0;ii<kb_->al.size();ii++)
		{
			if(!strcmp(node_tmp[i].name.c_str(),kb_->al[ii].c_str()))
			{
				write_file << ii << ",";
				break;
			}
		}
		write_file << node_tmp[i].centroid[0]	<< ","
				   << node_tmp[i].centroid[1] 	<< ","
				   << node_tmp[i].centroid[2] 	<< ","
				   << node_tmp[i].centroid[3] 	<< ","
				   << node_tmp[i].surface_flag 		<< ","
				   << node_tmp[i].cuboid_min[0] 	<< ","
				   << node_tmp[i].cuboid_min[1] 	<< ","
				   << node_tmp[i].cuboid_min[2] 	<< ","
				   << node_tmp[i].cuboid_max[0] 	<< ","
				   << node_tmp[i].cuboid_max[1] 	<< ","
				   << node_tmp[i].cuboid_max[2] 	<< ","
				   << node_tmp[i].contact;
		write_file << "\n";
	}
}

void WriteFile::WriteFileGraph(CGraph *Graph_, string path_)
{
	if (ifstream(path_)) { remove(path_.c_str()); }

	int num_location = Graph_->GetNodeList().size();
	vector<double> sec;
	vector<vector<vector<CGraph::edge_t> > > edges = Graph_->GetListOfEdges();

	ofstream write_file(path_, ios::app);
	write_file << "Locations," 			<< num_location << "\n";
	write_file << "Location Intervals," << Graph_->GetLocInt()		<< "\n";
	write_file << "Sector Intervals," 	<< Graph_->GetSecInt()		<< "\n";
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
					sec.clear();
					switch (tmp)
					{
						case 0:
							for(int iv=0;iv<Graph_->GetLocInt();iv++)
							{
								for(int v=0;v<3;v++)
								{
									write_file << float(edges[i][ii][0].nor[iv][v]);
									if (iv==Graph_->GetLocInt()-1 && v==2)
									{ write_file << "\n";  }
									else
									{ write_file << ","; }
								}
							}
							break;
						case 1:
							for(int iv=0;iv<Graph_->GetLocInt();iv++)
							{
								for(int v=0;v<3;v++)
								{
									write_file << float(edges[i][ii][0].tan[iv][v]);
									if (iv==Graph_->GetLocInt()-1 && v==2)
									{ write_file << "\n";  }
									else
									{ write_file << ","; }
								}
							}
							break;
						case 2:
							for(int iv=0;iv<Graph_->GetLocInt();iv++)
							{
								for(int v=0;v<4;v++)
								{
									write_file << float(edges[i][ii][0].loc_mid[iv][v]);
									if (iv==Graph_->GetLocInt()-1 && v==3)
									{ write_file << "\n";  }
									else
									{ write_file << ","; }
								}
							}
							break;
						case 3:
							for(int iv=0;iv<Graph_->GetLocInt();iv++)
							{
								write_file << edges[i][ii][0].loc_len[iv];
								if (iv<Graph_->GetLocInt()-1) 	{ write_file << ",";  }
								else 				{ write_file << "\n"; }
							}
							break;
						case 4:
							write_file << Graph_->GetEdgeCounter(i,ii,0) << "\n";
							break;
						case 5:
							sec = edges[i][ii][0].sector_map;
							for(int iv=0;iv<sec.size();iv++)
							{
								write_file << sec[iv];
								if (iv < sec.size()-1)	{ write_file << ",";  }
								else 					{ write_file << "\n"; }
							}
							break;
						default:
							tmp = -10;
							break;
					}
					tmp++;
				}
			}
		}
	}
}

void WriteFile::WriteFilePrediction(
	CGraph *Graph_,
	CKB *kb_,
	string path_,
	vector<string> labels_,
	vector<string> labels_predict_,
	vector<map<string,double> > goals_,
	vector<map<string,double> > windows_)
{
	if (ifstream(path_)) { remove(path_.c_str()); }

	string line;

	ofstream write_file(path_, ios::app);
	for(int i=0;i<labels_.size();i++)
	{
		line = labels_[i] + "," + labels_predict_[i] + ",";
		for(int ii=kb_->ac["GEOMETRIC"].first;ii<kb_->ac["GEOMETRIC"].second+1;ii++)
		{
			line += to_string(goals_[i][kb_->al[ii]]);
			line += ",";
		}
		for(int ii=kb_->ac["GEOMETRIC"].first;ii<kb_->ac["GEOMETRIC"].second+1;ii++)
		{
			line += to_string(windows_[i][kb_->al[ii]]);
			line += ",";
		}
		line.erase(line.size()-1);
		line += "\n";
		write_file << line;
	}
}

void WriteFile::WriteFileWindow(CGraph *Graph_, string path_)
{
	string path2 = path_;
	reverse(path2.begin(),path2.end());
	path2.erase(path2.begin(),path2.begin()+path2.find(".")+1);
	reverse(path2.begin(),path2.end());
	path2 = path2 + "2.txt";

	if (ifstream(path_)) { remove(path_.c_str()); }
	if (ifstream(path2)) { remove(path2.c_str()); }

	int s_tmp = -1;
	double max_val = 0.0;
	int num_location = Graph_->GetNodeList().size();
	vector<double> sec;
	vector<vector<vector<CGraph::edge_t> > > edges = Graph_->GetListOfEdges();

	ofstream write_file(path_, ios::app);
	ofstream write_file2(path2, ios::app);

	write_file << "Locations," 			<< num_location 		<< "\n";
	write_file << "Location Intervals," << Graph_->GetLocInt()	<< "\n";
	write_file << "Sector Intervals," 	<< Graph_->GetSecInt()		<< "\n";
	write_file2 << "Locations," 		<< num_location << "\n";
	write_file2 << "Location Intervals,"<< Graph_->GetLocInt()		<< "\n";
	write_file2 << "Sector Intervals," 	<< Graph_->GetSecInt()		<< "\n";
	for(int i=0;i<edges.size();i++)
	{
		for(int ii=0;ii<edges[i].size();ii++)
		{
			for(int iii=0;iii<edges[i][ii].size();iii++)
			{
				write_file << i << "," << ii << "," << iii << "\n";
				write_file2 << i << "," << ii << "," << iii << "\n";

				sec.clear(); sec = edges[i][ii][0].sector_map;

				for(int l=0;l<Graph_->GetLocInt();l++)
				{
					max_val = 0.0;
					for(int s=0;s<Graph_->GetSecInt()/2;s++)
					{
						max_val = max(sec[l*Graph_->GetSecInt()+s]+sec[l*Graph_->GetSecInt()+s+Graph_->GetSecInt()/2],max_val);
						s_tmp = s;
					}
					write_file << max_val;
					if (l < Graph_->GetLocInt()-1)	{ write_file << ",";  }
					else 				{ write_file << "\n"; }

					max_val = 0.0;
					for(int s=0;s<Graph_->GetSecInt()/2;s++)
					{
						max_val =
								max(
									sec[l*Graph_->GetSecInt()+(((s+(Graph_->GetSecInt()/4))+Graph_->GetSecInt())%Graph_->GetSecInt())] +
									sec[l*Graph_->GetSecInt()+(((s-(Graph_->GetSecInt()/4))+Graph_->GetSecInt())%Graph_->GetSecInt())], max_val);
					}
					write_file2 << max_val;
					if (l < Graph_->GetLocInt()-1)	{ write_file2 << ",";  }
					else 				{ write_file2 << "\n"; }
				}
			}
		}
	}
}

void WriteFile::WriteFileSurface(
	string path_,
	vector<Matrix3d> rotation_,
	vector<Vector4d> planeeq_,
	vector<Vector3d> boxmin_,
	vector<Vector3d> boxmid_,
	vector<Vector3d> boxmax_)
{
	if (ifstream(path_)) { remove(path_.c_str()); }

	ofstream write_file(path_, ios::app);
	for(int i=0;i<rotation_.size();i++)
	{
		write_file << i	<< ",";
		for(int ii=0;ii<4;ii++) write_file << planeeq_[i][ii] << ",";
		for(int ii=0;ii<3;ii++) write_file << boxmid_ [i][ii] << ",";
		for(int ii=0;ii<3;ii++) write_file << boxmin_ [i][ii] << ",";
		for(int ii=0;ii<3;ii++) write_file << boxmax_ [i][ii] << ",";
		for(int ii=0;ii<3;ii++)
			for(int iii=0;iii<3;iii++)
				write_file << rotation_[i](ii,iii) << ",";
		write_file << "\n";
	}
}

void WriteFile::WriteFile_(
	string path_,
	vector<vector<double> > data_)
{
	if (ifstream(path_)) { remove(path_.c_str()); }

	ofstream write_file(path_, ios::app);
	for(int i=0;i<data_.size();i++)
	{
		for(int ii=0;ii<data_[i].size();ii++)
		{
			if(ii<data_[i].size()-1)
				write_file << data_[i][ii] << ",";
			else
				write_file << data_[i][ii] << "\n";
		}
	}
}
