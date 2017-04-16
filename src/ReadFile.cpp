/*
 * ReadFile.cpp
 *
 *  Created on: Apr 16, 2017
 *      Author: chen
 */

#include "ReadFile.h"

ReadFile::ReadFile()
{
	n=nn=nnn=f=ff=fff=c=0;
	list0=list1=list2={};
	pair_tmp=make_pair(-1,"");
	file_list.clear();
}

ReadFile::~ReadFile() {}

int ReadFile::readFileName(
	string dir_name_,
	vector<int> idx_)
{
	// Reading the subject directory
	// Reading the action directory
	// Reading the individual action data file

	n = scandir(dir_name_.c_str(), &list0, folderSelect1, alphasort);
	if (n==0)	{printer(32); return EXIT_FAILURE;	}
	else		{printer(33);						}

	for(f=0;f<n;f++)
	{
		dir_s = dir_name_ + list0[f]->d_name;

		nn = scandir(dir_s.c_str(), &list1, folderSelect2, alphasort);
		if (nn==0)	{printer(25); return EXIT_FAILURE;	}
		else		{printer(26);						}

		vector<int> idx = idx_;
		map<int,pair<int,string> > map_tmp; // file number, action, filename
		c = 0;

		for(ff=0;ff<nn;ff++)
		{
			char num[4]; sprintf(num, "%03d", idx[0]);

			// takes only the action specify by idx
			if (strcmp(list1[ff]->d_name, num)) continue;

			dir_s = dir_name_ + list0[f]->d_name + "/" + list1[ff]->d_name;

			nnn = scandir(dir_s.c_str(), &list2, fileSelect, alphasort);
			for(fff=0;fff<nnn;fff++)
			{
				pair_tmp.first 	= idx[0]; //start with 001
				pair_tmp.second = dir_s + "/" + list2[fff]->d_name;
				map_tmp[c] = pair_tmp;
				c += 1;
			}

			idx.erase(idx.begin());

		}

		file_list[f] = map_tmp;

	}

	return EXIT_SUCCESS;
}

void ReadFile::clear()
{
	n=nn=nnn=f=ff=fff=c=0;
	list0=list1=list2={};
	pair_tmp=make_pair(-1,"");
	file_list.clear();
}
