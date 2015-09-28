# Input Format
## Format of Road Network (rd.sim.csv)

The rd.sim.csv is a road network file, including cross points and roads.
One line represents one logical process, including one cross point (CP) & out-going roads.
The line number represents cross point ID.
Out-going roads are separated by `;`.
```
{Road A at CP 0}; {Road B at CP 0}; {Road C at CP 0}; {Road D at CP 0};..
{Road E at CP 1}; {Road F at CP 1}; {Road G at CP 1};...
{Road F at CP 2}; ...
.
.
.
```
Each road has a format as follows.
Values are separated by `,`.
```
R, {road id}, {source}, {destination}, {speed limit (km/hour)}, {length (m)}, {# of lanes}
```
Example of a road network
```
R,0,0,1,10,100,2;R,1,0,1,10,200,1;R,3,0,3,10,200,100
```
## Format of Vehicles' Trip (trip.csv)
Vehicles' trip data includes ***ID***, ***Departure Time***, and ***Full Tracks*** to final destination cross point.
The ***ID*** has to be universally unique in simulation.
ID duplication causes fatal results' violation.
Values are separated by `,`.
```
TP, {id}, 0, {Departure time (sec)}, {beginning cp}, {2nd cp}, {3rd cp}, {4th cp},,, {end cp}
```
Example of vehicles' trips
```
TP,0,0,1,0,1,2,3,4,5
TP,1,0,10,2,1,2,3,4
```
### Format of trafficgraph
This is the general graph structure file generating partition file.
You can generate partition files with METIS ( http://glaros.dtc.umn.edu/gkhome/metis/metis/overview ).
In the first line, the numbers of vertexes & edges are defined.
Following lines show the neighber ID of cross point *N* ( *N* = line number ).   
```
{# of vertexes} {# of edges}
{neighber id of cross point 1} {neighber id of cross point 1} {neighber id of cross point 1},,
{neighber id of cross point 2} {neighber id of cross point 2},,
,,   
```
Check the format by
```
$ graphchk trafficgraph
```
And then
```
$ gpmetis trafficgraph {# of Partition}
```
You can see a new file `trafficgraph.part.{# of partition}`.
### Format of trafficgraph.part.*
```
{partition id of cross point1}
{partition id of cross point2}
,
,
,
```
#### !! NOTICE !!
In this format, IDs start from `1` (not from `0`).
So, the partition initializer converts ID from `x` to `x - 1`.

### Format of what if query
There are three types in what if query
- 1, State Change
- 2, Add Event
- 3, Delete Event

#### 1, State Change (SC)
State change input file includes ***LP ID***, ***Time***, ***New Logical Process***.
The first 3 values (`SC`, `{lp id}`, `{time}`) are separated by `,`.
The 3 values and `{new logical process}` are separated by `;`.
```
SC, {lp id}, {time}; {new logical process (a sequence of out-going roads)}
```
Example of State Change.
```
SC,10,555;R,0,10,1,10,100,2;R,1,10,1,10,200,1;R,3,10,3,10,200,100
```

#### 2, Add Event (AE)
Add event file has a same format as trip.csv, including ***Event ID***, ***Departure Time***, and ***Full Tracks***.
```
AE, {event id}, 0, {Deptime (sec)}, {beginning cross point}, {2nd cross point}, {3rd cross point}, {4th cross point},,, {end corss point}
```
Example of Add Event.
```
AE,0,0,1,0,1,2,3,4,5
```

#### 3, Delete Event (DE)
There are 2 ways to define deleting events.
The format includes ***LP ID***, ***Time*** and ***Event ID***
```
DE, {lp id}, {time}, {event id}
```
Or you can directly use result data.
```
RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
```
Example of Delete Event.
```
DE,999,10,555
RE,2,2,94,3,135
```

### Result format
The result format includes ***Event ID***, ***Source ID***, ***Send Time***, ***Destination ID***, and ***Receive Time***.
```
RE, {event id}, {source id}, {send time}, {destination id}, {receive time}
```
Example of Results.
```
RE,0,2,83,3,124
RE,1,3,92,4,133
RE,2,2,94,3,135
RE,0,3,124,4,165
RE,1,4,133,5,174
RE,2,3,135,4,176
```
