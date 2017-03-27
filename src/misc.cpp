/*
 * misc.cpp
 *
 *  Created on: Mar 24, 2017
 *      Author: chen
 */

#include "misc.h"

string obtainInput(
	string msg_,
	bool warning_)
{
	if (warning_) 	{cerr << msg_;}
	else			{cout << msg_;}
	string mystr;
	getline (cin, mystr);
	return mystr;
}

int printer(
	int x_)
{
	switch(x_)
	{
		case 1:
			printf("# Initialization.......................................................SUCCESS\n");
			break;
		case 2:
			printf("# Reading action labels................................................SUCCESS\n");
			break;
		case 3:
			printf(CRED "# Reading action labels.................................................FAILED\n" CNOR);
			break;
		case 4:
			printf("# Reading object specific action labels................................SUCCESS\n");
			break;
		case 5:
			printf(CRED "# Reading object specific action labels.................................FAILED\n" CNOR);
			break;
		case 6:
			printf("# Reading information about location areas.............................SUCCESS\n");
			break;
		case 7:
			printf(CYEL "# No information about location areas is found................................\n" CNOR);
			break;
		case 8:
			printf("# Reading training data................................................SUCCESS\n");
			break;
		case 9:
			printf("# Parsing training data................................................SUCCESS\n");
			break;
		case 10:
			printf("# Pre-processing data..................................................SUCCESS\n");
			break;
		case 11:
			printf("# Finding location areas...............................................SUCCESS\n");
			break;
		case 12:
			printf("# Building sector-map..................................................SUCCESS\n");
			break;
		case 13:
			printf("# Clustering of data with DBSCAN.......................................SUCCESS\n");
			break;
		case 14:
			printf("# Combining nearby clusters............................................SUCCESS\n");
			break;
		case 15:
			printf(CYEL "# Labeling clusters (location areas)..........................................\n" CNOR);
			break;
		case 16:
			printf("# Labeling clusters (location areas)...................................SUCCESS\n");
			break;
		case 17:
			printf("# Building nodes (location areas)......................................SUCCESS\n");
			break;
		case 18:
			printf("# Fitting curve........................................................SUCCESS\n");
			break;
		case 19:
			printf("# Adjusting sector map.................................................SUCCESS\n");
			break;
		case 20:
			printf("# Fitting points to sector map.........................................SUCCESS\n");
			break;
		case 21:
			printf("# Checking constraint..................................................SUCCESS\n");
			break;
		case 22:
			printf("# Fitting points to initial sector map.................................SUCCESS\n");
			break;

//		case :
//			break;

	}
	return EXIT_SUCCESS;
}

