cmake_minimum_required(VERSION 2.8.3)
project(pal_statistics_msgs)

find_package(catkin REQUIRED
    COMPONENTS
        message_generation
        roscpp
        std_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
    FILES
        Statistic.msg
        Statistics.msg
        StatisticsNames.msg
        StatisticsValues.msg
)

generate_messages(
    DEPENDENCIES
        std_msgs
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
    CATKIN_DEPENDS
        message_runtime
        std_msgs
)

