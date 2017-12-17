
### How to Compile and run

Each folder have a project with a cmakelist and source files. 

#To compile:

cd build/
cmake ..
make

##FinalProject_Marker1
This contains the vision code for detecting and tracking three points with marker1. Just running the file will automatically run it for one of the sequences. To run it for a specific sequence either change the code or call it with the path to the other sequence like: 

#Hard Sequence
./FinalProject_Marker1 ../marker_color_hard/marker_color_hard_%02d.png

#Easy sequence
./FinalProject_Marker1 ../marker_color/marker_color_%02d.png

##FinalProject_Marker3
Works the same way as marker1. Supports two input arguments when running. First is for the sequence, second is for the marker image to track. Both have a default set so the program can also be ran just by "./FinalProject_Marker3".

#Hard sequence
./FinalProject_Marker3 ../marker_corny_hard/marker_corny_hard_%02d.png ../marker3.png

#Easy sequence
./FinalProject_Marker3 ../marker_corny/marker_corny_%02d.png ../marker3.png


##Robotics
------- OLLIVER

##Combined Intergration
------- Same as in robotics, just uses the vision part to track instead of the sequence of motions. 
