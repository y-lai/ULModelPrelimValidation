# ULModelPrelimValidation

This is the example repository of source code used to complete the research. This research is to be published in EMBC 2021 and can be found [here](https://www.researchgate.net/publication/354311343_Preliminary_Validation_of_Upper_Limb_Musculoskeletal_Model_using_Static_Optimization).

A pre-requisite for this package is installing OpenSim with MATLAB bindings enabled. Instructions can be found [on the OpenSim github repository](https://github.com/opensim-org/opensim-core).

The package also requires the native MATLAB ROS Toolbox to be installed as an add-on in the MATLAB installation.

Start by looking at the example script in releaseExampleScript.m.

Example code for visualising the results can also be found in releaseAnalysis.m. Note that releaseExampleScript.m needs to be run before running releaseAnalysis.m.

NOTE: computing moment arms using the [generalized forces method](https://simtk-confluence.stanford.edu:8443/download/attachments/2624184/HowToComputeMuscleMomentArm.pdf?version=1&modificationDate=1341887981184&api=v2) is not implemented. (Unavailable MATLAB bindings from OpenSIM for certain functionalities from SimTK)

The *Utilities* folder consists of various functions used in runOpenSim plus a few functions to pre-process the rosbag files for EMG & force torque data, and processing the .csv files from NaturalPoint OptiTrack motion capture software.

For pre-processing data for OpenSim:

* extractTRCmarkerfile.m
  * convertFrames.m - user specific function to convert frames of reference such that it fits the osim model's frame of reference
* filterEMGPlusForce.m

Functions for OpenSim functionalities:

* removeUnnecessaryForces.m - user specific function to remove unnecessary forces if needed. Change the optional fields in runOpenSim.m to disable functionality.
* calcMomentArms.m
* calcForcewDynamics.m

Functions for plotting figures for EMG:

* plotMultichannel.m - function to plot multi-channel data in a single figure. Loosely based on [Hio-Been Han's implementation](https://github.com/Hio-Been/plot_multichan)

Functions obtained elsewhere:

* filtfilthd.m - obtained from [this MATLAB central exchange page](https://au.mathworks.com/matlabcentral/fileexchange/17061-filtfilthd) - credits go to Malcolm Lidierth
* loadFilterCropArray.m - obtained from [Advanced Examples in OpenSim for Custom Static Optimization](https://simtk-confluence.stanford.edu/display/OpenSim/Custom+Static+Optimization+in+MATLAB) - credits go to the OpenSim team
* osimToTableStruct.m - obtained from OpenSim when installing MATLAB bindings (a requirement for this package to work)
