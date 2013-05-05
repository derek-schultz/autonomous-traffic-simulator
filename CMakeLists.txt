INCLUDE_DIRECTORIES(${ROSS_SOURCE_DIR})

SET(ats_srcs 
    ats.c autonomous_traffic.c traffic_light.c)

ADD_EXECUTABLE(ats ${ats_srcs})

TARGET_LINK_LIBRARIES(ats ROSS m)
