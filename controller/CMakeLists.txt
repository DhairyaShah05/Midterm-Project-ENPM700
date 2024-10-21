# Create a library called "controller" (in Linux, this library is created
# with the name of either libcontroller.a or controller.so).
add_library(controller
  # list of cpp source files:
  controller.cpp
  PIDController.cpp
  Vehicle.cpp
  AckermannController.cpp
)
# Indicate what directories should be added to the include file search
# path when using this library.
target_include_directories(controller PUBLIC
  # list of directories:
  ${CMAKE_SOURCE_DIR}/include   
)
# Link necessary libraries (if any)
target_link_libraries(controller PUBLIC ${COMMON_LIBS})
# Link options
target_link_options(controller PUBLIC --static)
