# Create a library called "myLib2" (in Linux, this library is created
# with the name of either libmyLib2.a or myLib2.so).
add_library (controller
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
  ${COMMON_INCLUDE_DIRS}
  )

target_link_libraries(controller PUBLIC ${COMMON_LIBS})

target_link_options(controller PUBLIC --static)
