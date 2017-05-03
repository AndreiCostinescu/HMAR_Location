/*
 * WriteFile.h
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 */

#ifndef WRITEFILE_H_
#define WRITEFILE_H_

#include "dataDeclaration.h"
#include "algo.h"
#include "Graph.h"
#include "misc.h"

class WriteFile
{

	public:
		WriteFile();
		virtual ~WriteFile();

		void RewriteDataFileFilter(
			int curr_,
			int mem_,
			int mem2_,
			int mem3_,
			string &mem_s_,
			string &mem_s2_,
			string &mem_s3_,
			vector<string> &label_);

		void RewriteDataFile(
			string path_,
			vector<vector<string> > data_,
			vector<point_d> &points_,
			vector<int>	contact_,
			point_d face_,
			vector<point_d> label_ref_write_, // points in label reference
			vector<string> label_ref_name_, // name of label reference
			vector<string> label_list_); // list of label for actions taken

		void WriteFileLA(Graph *Graph_, string path_);

		void WriteFileGraph(Graph *Graph_, string path_);

		void WriteFileWindow(Graph *Graph_, string path_);

		void WriteFilePrediction(
			Graph *Graph_,
			string path_,
			vector<string> labels_,
			vector<string> labels_predict_,
			vector<map<string,double> > goals_,
			vector<map<string,double> > windows_);



};

#endif /* WRITEFILE_H_ */
