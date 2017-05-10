/*
 * ReadFile.h
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 */

#ifndef READFILE_H_
#define READFILE_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "misc.h"
#include "core.h"

class ReadFile
{
	public:
		ReadFile();
		virtual ~ReadFile();

		void ClearRF();

		bool CopyFile(
			string SRC,
			string DEST);

		bool DirectoryCheck(
			string path_);

		int ReadFile_(
			string path_,
			char delimiter);

		int ReadFileLabel(
			string path_,
			map<int,vector<string> > &label_);

		int ReadSurfaceFile(
			string path_,
			vector<Matrix3d> &rotation_,
			vector<Vector4d> &planeeq_,
			vector<Vector3d> &boxmin_,
			vector<Vector3d> &boxmid_,
			vector<Vector3d> &boxmax_);

		int ReadFileKB(
			string path_,
			kb_t &kb_);

		int ReadFileLA(
			Graph *Graph_,
			vector<string> al_,
			string path_);

		int ReadFileLA(
			vector<vector<string> > &data_,
			string path_);

		int ReadFileGraph(
			Graph *Graph_,
			string path_);

		int ReadFileName(
			string dir_name_,
			vector<int> idx_,
			map<int,map<int,pair<int,string> > > &file_list_,
			int &sub_num_);

		int ReadLabelFileName(
			string dir_name_,
			map<string, string> &label_list_);

		vector<vector<string> > GetDataRF() {return data_rf;}

	private:
		int n, nn, nnn;
		struct dirent **list0, **list1, **list2;
		vector<int> idx;
		pair<int,string> pair_tmp;
		map<int,pair<int,string> > map_tmp; // file number, action, filename

	protected:
		vector<vector<string> > data_rf;
};

#endif /* READFILE_H_ */
