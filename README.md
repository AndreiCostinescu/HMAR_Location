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

[07] True Label Commit
- 24/02/2017
- added function to give label name based on highest prediction rate

[08] Sector direction check Commit
- 27/02/2017
- added a check for +/- as the equation for calculating angle does not give +/-

[09]
- 06/03/2017
- reworked sectormap to check for variation along curve
- curve is updated based on averaging with previous ones

[09a]
- 06/03/2017
- nested all functions in main
- updated the prediction functions

[09b]
- 07/03/2017
- minor updates
- changed read file to read alphabetically
- minor modification to the prediction functions

[09c]
- 07/03/2017
- rectify change between pc

[10]
- 07/03/2017
- started data recording with 3 scenarios:  
  * cup direct drink
  * cup rest drink
  * sponge clean

[11]
- 08/03/2017
- updates for boundary condition.

[11a]
-08/03/2017
- evaluations for a few scenarios.
