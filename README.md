# Welcome to the UVIC ROBOTICS Rover page

A breakdown of the ROS Files will be in the wiki. The wiki should be updated so that people can get a clear understanding of what we are doing and what still needs to be done. 

## What this repository is supposed to be

This will be the home for all ROS related files that control our rover. This includes microcontroller code, configuration files for sensors, custom classes, and any packages that we create.

## Current Activities

The current things that need to get done for this project will:

- Decide on ROS Version
- Create basic framework for what needs to be published and subscribed
- Start building custom classes for each type of sensor
- Figure out how to handle more compute nodes
- Figure out gazebo
- Start simulating in gazebo
- Implement on Rover

## Who do I talk to?
To get things approved for use on the rover talk to Greg, He is the software lead and must approve any/all files.

## Revision control
To make sure we keep track of everything thats going on the rover we are implementing a basic revision control in 3 groups.

### REVISION 1
This is unapproved by the software lead, this is for WIP Items. Things that are in this category might not even compile. To get a rev 1 file to rev 2 it needs to be well commented, have a wiki page, clearly indicate sources (for math or datasheets), and must fit with the current setup ie. must not conflict with any rev 2 or rev 3 designs. Meeting all of these critera, you can contact the software lead and only he can approve something to rev 2 or higher.

### REVISION 2
This is approved by our software lead for use on the rover. However things in revision 2 have not been testing with the sensors on the rover. So think of this revision as good to go, but untested with hardware.

### REVISION 3
This is after it has passed the hardware integration tests. So it is confirmed and working. Only things in revision 3 will be on the final rover.

