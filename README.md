# HMAR_Location
----

[01] Initial Commit
- 19/01/2017
- system that clusters locations and creates the sector map connecting the location areas

[02] Updated Commit
- 16/02/2017
- system using saved data is running

[03] Correction Commit
- 17/02/2017
- all array data type changed to vector
- corrrected error in processing boundary and caclculating the average

[04] Data File Commit
- 20/02/2017
- added reading and writing data to file
- changed the way data is stored :  
  Scenes - [scene] - [object - [loc,mov,max,min,const], surface.txt]

[05] Class Commit
- 22/02/2017
- rewritten all data to depend on class graph
- added Verbose to show relevant ouput only

[06] Single Edge Commit
- 23/02/2017
- added function to show data with sectormap
- corrected error in angle calculation
- added VERBOSE to show the values independently for motion and nonmotion predictions
