include_directories(${CMAKE_SOURCE_DIR}/controllers)

add_subdirectory(footbot_task_partitioning)
# Compile the neural network controllers only if GALIB was found
if(GALIB_FOUND)
  add_subdirectory(footbot_nn)
endif(GALIB_FOUND)
