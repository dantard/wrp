cmake_minimum_required(VERSION 2.4.6)

project(wrp)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 NEW)
	cmake_policy(SET CMP0015 NEW)
endif(COMMAND cmake_policy)

#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/build/lib)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wextra -Wall -Wundef -Wstrict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -Werror-implicit-function-declaration")
SET(CMAKE_CXX_FLAGS "-std=c++0x")

add_library(radiotap
   core/layer_1/radiotap/radiotap.c
)

add_library(layer_1
   core/layer_1/include/layer_1.h
   core/layer_1/include/bridge.h
   core/layer_1/include/frames.h
   core/layer_1/include/types.h
   core/layer_1/layer_1.c
   core/layer_1/bridge.c
)
target_link_libraries(layer_1 rt pthread config)

add_library(layer_2
   core/layer_2/include/layer_2.h
   core/layer_2/layer_2.cpp
)
target_link_libraries(layer_2 rt pthread)

add_library(queues
   core/queues/include/QElement.h
   core/queues/include/Queue.h
   core/queues/include/queues.h
   core/queues/queues.cpp
)
target_link_libraries(queues rt pthread utils)

add_library(graphs
    core/graphs/include/graphs_common.h
    core/graphs/include/dfs.h
    core/graphs/dijkstra.cpp
    core/graphs/dfs.cpp
)

add_library(utils
    utils/timespec_utils.c
    utils/timespec_utils.h
    utils/bits.h
)

add_library(tuntap
    tun/include/tun.h
    tun/tun.cpp
    #tun/tin.c

)

add_executable(test_rx core/main.cpp)
target_link_libraries (tuntap pthread)
target_link_libraries (test_rx queues pthread graphs utils layer_1 layer_2 radiotap)

add_executable(test_tx core/main_tx.cpp)
target_link_libraries (tuntap pthread)
target_link_libraries (test_tx queues pthread graphs utils layer_1 layer_2 radiotap)

