Copyright (c) 2012, Carnegie Mellon University

Developed with the sponsorship of the Defense Advanced Research Agency

Permission is hereby granted, free of charge, to any person obtaining a copy of this data, including any software or models in source or binary form, specifications, algorithms, and documentation (collectively "the Data”), to deal in the Data without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Data, and to permit persons to whom the Data is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Data.

THE DATA IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS, SPONSORS, DEVELOPERS, CONTRIBUTORS, OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DATA OR THE USE OR OTHER DEALINGS IN THE DATA

Michael Dawson-Haggerty 5/29/2012
CMU Robotics Institute
For Dr. David Bourne / iFAB

README.txt: 
A summary of software used to run the iFAB welding node developed at Carnegie Mellon. The station software accepts JSON structures as an input, and outputs welded space frames. 

This document is ordered by importance to a user of the station. 

Contributors:
Michael Dawson-Haggerty
Mabaran Rajaraman
Zachary Rubinstein


Station Hardware:
1. ABB IRB 2600 Robot / IRC5 Controller
    a) Mounted on palletized base. 
    b) Running SERVER and LOGGER.
    c) Connected via ethernet to the computer running the main loop.
    d) 11 digital IO lines configured as a group output signal.
        i) This allows an integer of maximum size 2^11 to be sent to the Labjack DAQ. 
        ii) We use this signal for synchronizing robot positions with other sensors.
2. Labjack UE9 DAQ
    a) Connected to power supply and other devices.
    b) Connected to computer running main loop via USB or Ethernet.
3. ShowWX+ Laser projector 
    a) Mounted in protective enclosure on robot.
    b) Connected to computer video signal, Labjack DAQ, and power supply. 
    c) Labjack DAQ opens the servo controlled splatter protection door.
4. Micro-Epsilon ILD 1402-200 Laser Displacement Sensor
    a) Mounted in protective enclosure on robot. 
    b) Connected to Labjack DAQ and power supply. 
5. Tregaskiss 500A Air-Cooled Tough-Gun GMAW Torch
    a) Mounted on robot. 
6. Miller Auto-Axcess 300DI Welder
    a) Mounted on palletized base
7. Lifter with fixturing
    a) ABB Flexlifter 600
    b) I - beams mounted to accept welding table or complete vehicle frame
8. Table
    a) 8’ x 6.5’ slatted modular welding table, steel top. 


Station Software Requirements:
1. Python 2.7
    a) We recommend downloading Python(X,Y), a scientific install which includes Numpy, Scipy, and matplotlib
1. Numpy
2. Pyglet
3. Labjackpython
4. Matplotlib


lib\glserver.py:
Runs a loop, waiting for messages to come in over the network containing 2D image information, and display them when they do arrive. It opens a fullscreen window on the last monitor on a computer. It requires the projector to be configured as a monitor in windows. Start glserver.py before the main loop, by running:
python glserver.py 
Leave it running in the background, and then the main loop can be started. 


main\main.py: 
The main manufacturing loop is contained within this file. main.py loads an iFAB welding format zip file. A typical call to manufacture a structure defined in ‘octagon.zip’ would look like:
python main.py octagon.zip -w -z

This call would open octagon.zip, load the .STL model files from it, load the JSON serialized data structure, project fixture/part locations for a worker (based on the .STL models and the transformation matrices in the JSON structure), do an inspection (the -z specifies inspection type, in this case zig-zag), and then weld (the -w means no prompt between inspection and welding)
Physical placement of a member is guided with laser projection system, and then scanned resulting in a point cloud. That very accurate data is used to align the weld, so that parts which are misaligned on an absolute scale, but correct on a relative scale can succeed. We can also use this data to calculate if the relative positions are within tolerance before welding. 


data\tool.object.[welder, projector, displacement]
These three files contain a data structure which contains the tool calibrations for the three devices (welder, projector, and displacement sensor). It is serialized using the python-specific ‘pickle’ module. To examine the data, you can run (in python):
>>> import pickle
>>> t = pickle.load(open('tool.object.displacement', 'rb'))
>>> t
[[34.348, 160.756, 73.993], [-0.708, 0.007, -0.706, -0.007]]
The first three numbers contain the XYZ offset of the tool center point (in mm), and the next 4 numbers consist of the quaternion orientation of the tool. These three calibrations are all generated in different ways, but luckily they only have to be done when one of the devices shifts (usually from either disassembly or an impact). 
 ‘tool.object.welder’ is generated using the multi-point method included with the ABB robot. 
 To generate the weld tool calibration, on the robot teach pendant (in manual mode):
1. Press the red ‘ABB’ logo in the top right of the screen
2. Program Data
3. tooldata
4. New > OK
5. Edit > Define
6. Click on ‘Point 1’, jog the robot so that the tip of the welder is lined up with a pointed stick
7. Press ‘modify position’
8. Jog the robot so that the tip of the welder is lined up with the pointed stick, but from a different angle.
9. Press ‘modify position’
10. After defining several points, the robot will calculate a tool object. 


demos\calibrateProjectorQuat.py:
Used to generate tool objects for the laser projector and the displacement sensor (‘tool.object.displacement’, ‘tool.object.projector’)
To run, start SERVER on the robot, and glserver.py on the projection computer and run:
python calibrateProjectorQuat.py -r -p

1. Jog the robot angularly by setting the axis you want to rotate around (‘p’) and the step size 
    (‘stepa’, then start moving by typing ‘step’) until the projector/sensor is pointed roughly down. 
2. Jog it linearly (set linear step size with ‘stepl’ and axis again with ‘p’) as low as you can go. 
3. Put a piece of white paper under the laser crosshairs or dot, and establish the axis directions on the paper by jogging in different directions. 
4. Mark the dot on the paper. 
5. Jog the robot ONLY vertically as high as you can get the robot (you can use the ‘gov’ command). 
6. Mark where the dot has moved to, and then measure the distance in X and Y. 
7. Run the ‘xyh’ command, which asks for the movement in X, Y, and the vertical jog magnitude. 
    It will calculate the magnitude of the angle, and the vector which to rotate around. 
    It will update the current quaternion to attempt to be perpendicular to the surface. 
8. Run a negative ‘gov’ to see if the dot moves or not. If it moves very little 
    (sub-mm is what you should be expecting), you can save the tool. If it moves more that, you should run another cycle, 
    and measure the distances more carefully. 
9. You have now jogged to the correct orientation of the tool. You can save this value by calculating the tool object (‘tool’) and then saving it (‘dtool’). 
10. Establish a point on the table (use the multi-point method on the weld tool, and then jog it to a point on the table)
11. Point the projector/sensor straight down. Then, jog it in XY until the established point and the laser line up. 
    The difference between the robot XY and the established point XY is the tool offset. 
12a. To calculate the Z offset with the laser displacement sensor, in another terminal start up distance-polling.py, and jog the robot down in Z until the sensor reading is zero mm. 
12b. To calculate the Z offset for the projector, we project a checkerboard pattern, and go to multiple Z levels, and then track the corner movement with Z height. We haven’t automated this process, although we started (check out dev\projectormap\autocalibrate.py for more details)


ABB-RAPID\ifab1_BACKUP_2012-05-29:
This is a modern backup of the controller configuration. SERVER and LOGGER are defined as separate tasks. 


workpieces\*:
Contains the JSON files we used to manufacture the HMMWV as well as some test code, and a possible version 2.0 of the file format. 


NON-USER ACCESSED FILES:
lib\ifabModel.py: loads .STL files, calculates outlines, transforms, runs inspections
lib\ifabPCL.py: does template matching optimizatons 
lib\projector.py: sends messages to glserver, contains transfer classes
lib\jackDistance.py: interfaces with labjack for distance readings / door openings, as well as 
    syncronization and forward kinematics for the sensor data
lib\robot.py: contains robot motion interface classes, and logger interface classes 
demos\distance-polling.py: runs the displacement sensor manually
dev\cutting\stock-cut.py: very basic stock cutting planner which divides list of rough cut pices between the inventory in a way to reduce scrap
dev\cutCalc.py: Takes a list of .STL files for space frames, and calculates the end profile for automated plasma cutting. 
dev\projectormap\autocalibrate.py: unfinished prototype to use vision to automatically generate tool calibrations for the projector and displacement sensor. 
dev\pyCalib\pycad.txt: We’ve written a lot of functions to access mesh geometry, 
    and this was an attempt to clean up some of it so we could eventually not include geometry functions in our library files.
    The functions included here are cleaned up and tweaked for speed.