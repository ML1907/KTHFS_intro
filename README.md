# KTHFS_intro
recruitment exercises solution

DOWNLOADING INSTRUCTIONS:
*This git repository has to be downloaded inside the SRC folder of your catkin_ws
*Packages' dependencies are specified inside the package.xml files. Re-build the catkin_ws after having downloaded the repository.

RUN THE FILES:
*ex1:
	-source the enviroment: source devel/setup.bash
	-open 3 terminals (Ubuntu 18.04, ros melodic):
		--roscore
		--rosrun broadcast publisher_node.py
		--rosrun receiver subscriber_node.py

*ex2: (Ros is not needed. Ex2 is there just for convenience)
	-cd in the folder Ex2 and run: python FunctionVisualiser.py 
