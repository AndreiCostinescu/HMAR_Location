/*
 * ReadFile.h
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 */

#ifndef READFILE_H_
#define READFILE_H_

#include "dataDeclaration.h"
#include "misc.h"
#include "readWriteFile.h"

class ReadFile
{
	public:
		ReadFile();
		virtual ~ReadFile();

		int readFileName(
			string dir_name_,
			vector<int> idx_);

		void clear();

		int getNumberOfSubject() {return n;}

		map<int,map<int,pair<int,string> > > getFileList() {return file_list;}

	private:
		int n, nn, nnn, f, ff, fff, c;
		struct dirent **list0, **list1, **list2;
		string dir_s;
		pair<int,string> pair_tmp;
		map<int,map<int,pair<int,string> > > file_list; // subject, file number, action, filename
};

#endif /* READFILE_H_ */
