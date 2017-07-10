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
- 08/03/2017
- evaluations for a few scenarios.

[12]
- 09/03/2017
- updated function and variable names.
- added function to fill up the holes in sectormap by allowing multiple loc_pred during adjustment.

[13]
- 10/03/2017
- added a new location.
- updated to show movement (sliding) for both in and outside the location areas.

[14]
- 13/03/2017
- restructured again the code
- clustering is done separately before learning the sector maps
- given a 0.9999 confidence for inside prediction to allow it to be comapred with the rest.
- INTERESTINGLY : the confidence drops at the beginning before increasing again.
- this is probably due to the fact that the location area is clustered as a sphere dome instead of a point.

[15]
- 14/03/2017
- added 3 new libraries
- moved everything from main into util
- main functions are split into 3 modules
- added a check to prevent prediction from showing 0% confidence by taking the last known prediction

[16]
- 15/03/2017
- reworked the colorcode function
- allow the curve check to look for more sectors instead of just 1

[17] MAJOR CHANGE
- 27/03/2017
- files are now processed individually instead of doing batch processing.
- LA are defined as locations where a change in contact is valid.
- Normal vector for each edge is only calculated using the first sector and transformed along the entire edge.
- Reworked the prediction model, it now takes in 5 criterias to determine the actions.
- a list of action label is defined.
- surface information is not used...

[18] MAJOR CHANGE
- 28/03/2017
- added new data
- added the option of deleting locations

[19]
- 29/03/2017
- added new data
- added some constraint to prevent the front and back part of sectormap from changing drastically
-still need to address the issue of curved ends#####

[20]
- 04/04/2017
- new data 
- reworked how to deal with begin and end situation of sectormap
- for case of curve we ignore if the curve goes backwards

[21]
- 06/04/2017
- new data 
- added printout analysis
- new ideas on analysis based on d2l and curve together with slide

[22]
- 16/04/2017
- revert back to before d2l
- added evaluation, prediction, read datafile classes
- evaluation is to evaluate during prediction
- prediction is to gather the data and save it to be outputed

[23]
- 21/04/2017
- added results
- added new data recordings
- encased everything in class except algo and core

[24]
- 03/05/2017
- evaluated result and added matlab plots
- final adjustment on code to handle cases exceeding the sectormap
- added decision tree for parsing the data

[25]
- 05/05/2017
- first draft version
- code and system is working as of time of writing with results

[26]
- 08/05/2017
- changed the way the tables are modelled, using OBB representation.
- added the surface boundary with new kinect data.

[27]
- 10/05/2017
- using the LA as reference.
- adaptable LA.
- added the EIGEN library

[28]
- 22/05/2017
- used pointer referencing between classes.
- added "containers" to keep the data that are relevant for processing.
- removed most inheritance, it is making things unflexible.

[29]
- 26/06/2017
- added object state
- added new recording data
- reorganized the folders

[29a]
- 26/06/2017
- added a build folder

[29b]
- 26/06/2017
- modified testing to allow for integration with online deployment

[30]
- 10/07/2017
- updated the code and changed how cdata class works.

