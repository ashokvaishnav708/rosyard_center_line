The functionality and working for the centerline estimation is defined in the file 'centerline_estimation_py' in the 'script' folder. The scripts folder will contain all the .py extension files. 

Any file with .cpp extenion must be stored in the 'src' directory. 

The executable script is referenced is the cmake file 'CMakeLists.txt' defined in the same directory. It has to be changed in case of referencing a different file for execution.  

The directory 'rosyard_common/config' stores all the .yaml files such as configuration files. Currently, ROS is uses a cheat driver which is enabled via a flag defined in /node_parameters/ file in this directory. 

'src/rosyard_common/msg' stores various .h extension files in which numerous datatypes have been defined which are used throughout the project. 

In the directory rosyard_common/src/rosyard_common_scripts/ there are files where a few
functions have already been defined which can be referred while using the pointcloud. One of
the functions that is used, is to convert the cone color to string from type RGB
(common_methods.py).

While executing we need to use "catkin -j8" command if any changes have been commited in any file/script.
