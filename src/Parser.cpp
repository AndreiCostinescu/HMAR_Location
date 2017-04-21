/*
 * Parser.cpp
 *
 *  Created on: Apr 18, 2017
 *      Author: chen
 */

#include "Parser.h"

Parser::Parser()
{
	data_parser.clear();
	frames.clear();
	contact.clear();
	points.clear();
	labels.clear();
	face.x=face.y=face.z=0;
}

Parser::~Parser() { }

void Parser::ClearParser()
{
	data_parser.clear();
	frames.clear();
	contact.clear();
	points.clear();
	labels.clear();
	face.x=face.y=face.z=0;
}

void Parser::SetDataParser(vector<vector<string> > data_)
{
	this->ClearParser();
	data_parser = data_;
	frames .resize(data_parser.size());
	contact.resize(data_parser.size());
	points .resize(data_parser.size());
	labels .resize(data_parser.size());
}

int Parser::ParseDataNoLabel()
{
	for(int i=0;i<data_parser.size();i++)
	{
		if (data_parser[i].size()<8) return EXIT_FAILURE;
		frames [i]	 = atoi(data_parser[i][0].c_str());
		contact[i]   = atoi(data_parser[i][1].c_str());
		points [i].x = atof(data_parser[i][2].c_str());
		points [i].y = atof(data_parser[i][3].c_str());
		points [i].z = atof(data_parser[i][4].c_str());
		points [i].l = UNCLASSIFIED;
		if(i==200)
		{
			face.x = atof(data_parser[i][5].c_str());
			face.y = atof(data_parser[i][6].c_str());
			face.z = atof(data_parser[i][7].c_str());
			face.l = UNCLASSIFIED;
		}
	}
	return EXIT_SUCCESS;
}

int Parser::ParseData()
{
	for(int i=0;i<data_parser.size();i++)
	{
		if (data_parser[i].size()<9) return EXIT_FAILURE;
		frames [i]	 = atoi(data_parser[i][0].c_str());
		contact[i]   = atoi(data_parser[i][1].c_str());
		points [i].x = atof(data_parser[i][2].c_str());
		points [i].y = atof(data_parser[i][3].c_str());
		points [i].z = atof(data_parser[i][4].c_str());
		points [i].l = UNCLASSIFIED;
		if(i==200)
		{
			face.x = atof(data_parser[i][5].c_str());
			face.y = atof(data_parser[i][6].c_str());
			face.z = atof(data_parser[i][7].c_str());
			face.l = UNCLASSIFIED;
		}
		labels [i] = string(data_parser[i][8]);
	}
	return EXIT_SUCCESS;
}
