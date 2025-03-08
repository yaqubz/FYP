# README: run 'compile_run_exe.bat' in the launch directory to compile and run. need to update the .exe filename in the batch file.

cmake_minimum_required(VERSION 3.0.0)

# Define a variable for the executable name ---- INSERT OUTPUT .EXE NAME HERE
set(TARGET_NAME nlink_unpack_COMx_udp2)    

project(${TARGET_NAME} VERSION 0.1.0) # project name is not necessarily the executable name, but set same here for consistency
include(CTest)
enable_testing()

add_executable(
  ${TARGET_NAME}
  main_udpmedianv2.c  # ---- INSERT SOURCE CODE HERE
  src/nlink_utils.c
  src/nlink_linktrack_nodeframe0.c
  src/nlink_linktrack_nodeframe1.c
  src/nlink_linktrack_nodeframe2.c
  src/nlink_linktrack_nodeframe3.c
  src/nlink_linktrack_aoa_nodeframe0.c
  src/nlink_linktrack_tagframe0.c
  src/nlink_linktrack_anchorframe0.c
  src/nlink_tofsense_frame0.c
  src/nlink_tofsensem_frame0.c)

include_directories(
  "include/"
  "src/"
)

target_link_libraries(${TARGET_NAME} ws2_32.lib)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)


# # Use forward slashes or double backslashes for paths
# # include_directories("C:/Users/Gabriel/OneDrive - Nanyang Technological University/Desktop/nlink_unpack-master/include")
# include_directories(  # 7 Jan TBC is this required
#   "include/"
#   "src/"
# )

# # target_link_libraries(nlink_unpack3 "C:/Users/Gabriel/OneDrive - Nanyang Technological University/Desktop/nlink_unpack-master/lib/json-c.lib")
# # target_link_libraries(nlink_unpack2 "C:/Users/Gabriel/OneDrive - Nanyang Technological University/Desktop/nlink_unpack-master/lib/ws2_32.lib")

# target_link_libraries(nlink_unpack2_1 ws2_32.lib)   # 7 Jan TBC is this required
# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
