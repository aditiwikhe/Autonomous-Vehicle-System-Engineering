# Autonomous-Vehicle-System-Engineering

## HW1 - Distressed Flashing

Under control of the computer, cause the vehicle to flash the SOS pattern on its turn indicator lights. SOS in morse code is dot-dot-dot-dash-dash-dash-dot-dot-dot. Map dot to the left turn indicator and dash to the right, and make the vehicle flash: left-left-left-right-right-right-left-left-left followed by a wait, then repeat. 

## HW2 - Braking

Cause the vehicle to brake after detecting a person (answers in teams of no more than five). You may use any detector. 

## HW3 - Controlling distance from pedestrain

Cause the vehicle to follow a person at a fixed distance. The person should walk, and may move fairly fast. If the person moves toward the vehicle, the vehicle should reverse to keep the distance constant. You may measure the distance to the person either using LIDAR, or using the size of a 2D detector box in the image. You should use a PID controller. 

## HW4 - GPS Scribbling

Use the codebase for GPS and waypoints which Hang Cui has kindly let us use ( gzipped tarfile) and your skill and judgement to cause the vehicle to follow a figure 8 path on the outdoor track. Read this code carefully, as it won't do what you want; you'll need to modify. 

## HW5 - Using Lidar to position the vehicle.

We will provide two large cardboard boxes, visible to LIDAR. These will be placed some distance apart on the outdoor track. From any reasonable start point, you should cause the vehicle to: find the boxes in the LIDAR; drive between the boxes and in a circle around one box; then stop between the boxes. Doing so will require using a simple planner, because this should work for any (reasonable) pair of boxes and any (reasonable) start position of the vehicle. 
