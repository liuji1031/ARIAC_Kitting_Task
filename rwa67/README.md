# ENPM663_GroupOne

# This is the branch to clone for rwa67

# Brief description about folder contents

1. Config:- consists of
    > sensors.yaml: that lists all sensors spawned in simulation to be used for detecting
                    objects in the environment
    
    > trials folder: contains yaml files that setup the competition with various
                     arrangments of parts in the environment to be shipped

2. include:- contains header files for services that change gripper, perform placing 
             tasks, shipping and submitting orders, and defining basic parametrs used 
             for handling poses in the simulation

3. launch:- contains launch folder to run the competition ones the ariac environment is
            spawned

4. meshes:- contains the 3D model stl files visualizing all objects in Gazebo

5. nodes:- holds python executables (user interface) responsible for starting competition
           and starting the pick up services

6. rwa67:- contains python files that are the domain interfaces (including the class 
           templates) for the action tree, competition interface, pick up service, and pose computations used when running the competition.

7. src:- contain the implementation source codes for the services defined in the include
         folder

8. To build and run this package, CMakeLists.txt and package.xml are included listing all
   the dependencies needed to run package successfully.

