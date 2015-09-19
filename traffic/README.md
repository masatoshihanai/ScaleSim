###Format of rd.sim.csv###
The rd.sim.csv is a road network file.
One line represents one logical process (one cross point (CP) + out-going roads from it).
{Road A at CP X}; {Road B at CP X}; {Road C at CP X}; {Road D at CP X};..
{Road E at CP Y}; {Road F at CP Y}; {Road G at CP Y};...
{Road F at CP Z}; ...
...
##Roads format##
R, {road id}, {source cross point}, {distination cross point}, {speed limit (km/hour)}, {length (m)}, {# of lanes} 
ex)
  R,0,0,1,10,100,2;R,1,0,1,10,200,1;R,3,0,3,10,200,100

###Format of trip.csv###
TP, {id}, 0, {Deptime (sec)}, {beginning cross point}, {2nd cross point}, {3rd cross point}, {4th cross point},,, {end corss point}
ex) 
  TP,0,0,1,0,1,2,3,4,5
  TP,1,0,10,2,1,2,3,4

###Format of trafficgraph###
This is the format for METIS
You can check the format by
$ graphchk trafficgraph

And then

$ gpmetis trafficgraph {# of Partition}
  ##NOTICE##
  In this format, the ids start from "1".
  So, the converter convert id from "x" to "x - 1" in trafficgraph.part.* 
  ----
  {# of vertexes} {# of edges}
  {neighber id of cross point 1} {neighber id of cross point 1} {neighber id of cross point 1},,
  {neighber id of cross point 2} {neighber id of cross point 2},,
  ,,   
  ----

###Format of trafficgraph.part.* ###
--
{partition id of cross point0}
{partition id of cross point1}
,
,
,
--

###Format of what if query ###
There are three types in what if query
- 1, Change State
- 2, Add Event
- 3, Delete Event 
  ## 1, State Change (SC) ##
  SC, {lp id}, {time}; {lp format};..;..;...
  ex)
    SC,10,555;R,0,10,1,10,100,2;R,1,10,1,10,200,1;R,3,10,3,10,200,100

  ## 2, Add Event (AE) ##
  Same format as trip.csv
  AE, {event id}, 0, {Deptime (sec)}, {beginning cross point}, {2nd cross point}, {3rd cross point}, {4th cross point},,, {end corss point}
  ex)
    AE,0,0,1,0,1,2,3,4,5

  ## 3, Event Delete (DE) ##
  DE, {lp id}, {time}, {event id}
  Or you can directly use result data.
  RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
  ex)
    DE,999,10,555 
    RE,2,2,94,3,135

###Result formatt ###
RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
ex)
  RE,0,2,83,3,124
  RE,1,3,92,4,133
  RE,2,2,94,3,135
  RE,0,3,124,4,165
  RE,1,4,133,5,174
  RE,2,3,135,4,176


