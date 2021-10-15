cmake_minimum_required(VERSION 2.8.3)
project(pal_carbon_collector)

find_package(catkin REQUIRED
    COMPONENTS
        pal_statistics_msgs
)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################

catkin_package()


#############
## Install ##
#############

install(PROGRAMS
    scripts/carbon_collector_node.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir config launch)
    install(DIRECTORY ${dir}/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)

    find_package(rostest REQUIRED)

    add_rostest(test/carbon_collector.test)

endif()
