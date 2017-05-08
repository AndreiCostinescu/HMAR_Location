//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "dataDeclaration.h"
#include "TestCase.h"

//=============================================================================
// Global
//=============================================================================

//=============================================================================
// MAIN
//=============================================================================
int main(int argc, char *argv[])
{
	printf("==============================================================================\n");
	printf("|| SYSTEM START                                                             ||\n");
	printf("==============================================================================\n");

	if(0)
	{
		ReadFile RF; WriteFile WF;
		vector<point_d> peq,bmin,bmid,bmax;
		vector<vector<double> > rot;
		RF.ReadSurfaceFile("kb/surface_.txt",rot,peq,bmin,bmid,bmax);
		WF.WriteFileSurface("kb/surface.txt",rot,peq,bmin,bmid,bmax);
	}

	TestCase *TC = new TestCase();

//	// Label
//	TC->Choose(0);

//	// Train Ind.
//	TC->Choose(1);
//	TC->Choose(2);
//	TC->Choose(3);
//	TC->Choose(4);

//	// Test
//	TC->Choose(5);
//	TC->Choose(6);
//	TC->Choose(7);
//	TC->Choose(8);

	TC->Choose(1);

//	// Train Cup
//	TC.Choose(1, TRN);
//	// Train Org
//	TC.Choose(2, TRN);
//	// Train Spg
//	TC.Choose(3, TRN);
//	// Train Knf
//	TC.Choose(4, TRN);

//	// Train Cup
//	TC.Choose(1, TST);
//	// Train Org
//	TC.Choose(2, TST);
//	// Train Spg
//	TC.Choose(3, TST);
//	// Train Knf
//	TC.Choose(4, TST);


//	TC.Choose(3, TST);


//	TC.Choose(5, DPL);

	delete TC;

	printf("==============================================================================\n");
	printf("|| SYSTEM END                                                               ||\n");
	printf("==============================================================================\n");

	return 0;
}




