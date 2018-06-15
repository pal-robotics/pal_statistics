cmake_minimum_required(VERSION 2.8.3)
project(pal_carbon_collector)

find_package(catkin REQUIRED
    COMPONENTS
        pal_statistics_msgs
        rospy
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
endforeach(dir)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pal_carbon_collector.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
