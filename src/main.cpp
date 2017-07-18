//=============================================================================
// Name        : main.cpp
// Author      :
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//=============================================================================

#include "memory"
#include "Preprocessing.h"
#include "TestCase.h"
#include "festival.h"

//=============================================================================
// Global
//=============================================================================

//=============================================================================
// MAIN
//=============================================================================
int main(
		int argc,
		char *argv[])
{
	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM START                                                             ||\n");
	printf(
			"==============================================================================\n");

	if (0)
	{
		// Preprocessing
		std::string kb_path
		{ "Data2/kb/" };
		std::vector<string> objs
		{ "CUP", "APP", "SPG" };

		std::map<std::string, std::vector<int> > idxs;
		idxs["CUP"] =
		{	1,2};
		idxs["APP"] =
		{	3,4};
		idxs["SPG"] =
		{	5,6};

		Preprocessing PP;
		PP.Transitions(kb_path, objs, idxs);
		PP.Surfaces("Data2/kb/surface.txt", "Data2/kb/surface_.txt");

		//			if (obj == "CUP") { idxs = {1}; }
		//			if (obj == "ORG") { idxs = {2}; }
		//			if (obj == "SPG") { idxs = {3,4,5,6,7,8}; }
		//			if (obj == "KNF") { idxs = {9}; }
	}

//std::shared_ptr<TestCase> TC(new TestCase());
//	auto TC = std::make_shared<TestCase>();
//	TC->Choose(100);
//	TC->Choose(200);
//	TC->Choose(300);
//	TC->Choose(1000);
//	TC->Choose(2000);
//	TC->Choose(3000);

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

//	TC->Choose(9);
//	TC->Choose(10);
//	TC->Choose(11);
//	TC->Choose(12);
//
//	TC->Choose(13);
//	TC->Choose(14);
//	TC->Choose(15);
//	TC->Choose(16);

//	TC->Choose(9);
//	TC->Choose(13);

////	TC->Choose(10);
//	TC->Choose(14);
//
////	TC->Choose(11);
//	TC->Choose(15);
//
////	TC->Choose(12);
//	TC->Choose(16);

//	TC->Choose(17);
//	TC->Choose(18);
//	TC->Choose(19);
//	TC->Choose(20);

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

//	delete TC;

//	{
//		std::map<string, std::vector3d> offset_list;
//		std::vector<std::vector3d> translation;
//		std::vector<std::vector<Matrix3d> > rotation;
//		unique_ptr<ReadFile> RF(new ReadFile());
//		RF->ReadDataset(" ",translation,rotation,offset_list);
//	}

//    EST_Wave wave;
//    int heap_size = 210000;  // default scheme heap size
//    int load_init_files = 1; // we want the festival init files loaded
//
//    festival_initialize(load_init_files,heap_size);
//
//    // Say simple file
//    festival_say_file("/etc/motd");
//
//    festival_eval_command("(voice_ked_diphone)");
//    // Say some text;
//    festival_say_text("hello world");
//
//    // Convert to a waveform
//    festival_text_to_wave("hello world",wave);
//    //wave.save("/tmp/wave.wav","riff");
//
//    // festival_say_file puts the system in async mode so we better
//    // wait for the spooler to reach the last waveform before exiting
//    // This isn't necessary if only festival_say_text is being used (and
//    // your own wave playing stuff)
//    festival_wait_for_spooler();

//	int heap_size = 210000;  // default scheme heap size
//	int load_init_files = 1; // we want the festival init files loaded
//	festival_initialize(load_init_files, heap_size);
//	for (int i = 0; i < 100; i++)
//	{
////		sleep(5);
//		festival_eval_command("(voice_ked_diphone)");
//		festival_say_text("i think you are going to drink");
//	}

	printf(
			"==============================================================================\n");
	printf(
			"|| SYSTEM END                                                               ||\n");
	printf(
			"==============================================================================\n");

	return 0;
}

