===============================================================================================
Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining 
Version: 1.0
===============================================================================================

Copyright (C) 2013 Liang Zhao, Shoudong Huang and Gamini Dissanayake
University of Technology, Sydney, Australia

MATLAB and C/C++ sourse code for Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining. 

Authors:  Liang Zhao         -- liang.zhao@imperial.ac.uk 
          Shoudong Huang     -- Shoudong.Huang@uts.edu.au
	  Gamini Dissanayake -- Gamini.Dissanayake@uts.edu.au

          Centre for Autonomous Systems
          Faculty of Engineering and Information Technology
          University of Technology, Sydney
          NSW 2007, Australia
-----------------------------------------------------------------------------------------------
License
-----------------------------------------------------------------------------------------------

Linear SLAM by Liang Zhao, Shoudong Huang, Gamini Dissanayake is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.

1) You can freely use and modify this code.

2) If you want to distribute code based on this one, it has to be done under the Creative Commons Attribution-NonCommercial-ShareAlike 3.0 Unported License.

If you use this code for academic work, please reference:

      Liang Zhao, Shoudong Huang and Gamini Dissanayake,
      Linear SLAM: A Linear Solution to the Pose Feature and Pose Graph SLAM based on Submap Joining,
      Submitted to IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2013.

3) For commercial uses, please contact the authors.

-----------------------------------------------------------------------------------------------
Quick start
-----------------------------------------------------------------------------------------------

MATLAB Code

Run Main.m

C/C++ Code

In Linux

Install cmake and suitesparse
    - sudo apt-get install cmake
    - sudo apt-get install libsuitesparse-dev

Install Linear SLAM
    - cd LinearSLAM_C/linux
    - mkdir build
    - cd build
    - cmake ..
    - make
    - sudo make install
 
Run
    - LinearSLAM -path VicPark_200_local_maps -num 200 -meth DC -type 2DPF -p pose.txt -f feature.txt

If one wants to know all the commands, type 
    - LinearSLAM -help

In Windows

Open solution "LinearSLAM.sln" using Microsoft Visual Studio.
Complie and run.
We recommend to use Microsoft Visual Studio 2010 or higher version.

The code are accompanied by some simulation and experimental datasets, for both pose feature and pose graph, 2D and 3D.

2D Pose Feature Datasets
    VicPark 200 local maps
    VicPark 6898 local maps
    DLR 200 local maps
    DLR 3298 local maps
    8240 data 50 local maps
    35188 data 700 local maps

3D Pose Feature Datasets
    Simu 3D 870 Loop


2D Pose Graph Datasets
    Intel
    manhattanOlson3500
    city10000

3D Pose Graph Datasets
    parking garage
    sphere2500

-----------------------------------------------------------------------------------------------
Support
-----------------------------------------------------------------------------------------------

Linear SLAM

Version: 1.0

The code is not optimized in this version. Any questions, comments, suggestions, discussions and bug reports are welcomed. The authors will appreciate every suggestions and any efforts on optimizing the code. 

Please mail to liang.zhao@imperial.ac.uk


