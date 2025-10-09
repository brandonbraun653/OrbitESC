# Trace Module CMakeLists.txt
# Generalized trace module for embedded and simulator environments

# Create trace library
add_library(OrbitTrace STATIC
  trace_registry.cpp
  trace_interface.cpp
  trace_examples.cpp
)

# Add embedded-specific sources
if(NOT SIMULATOR)
  target_sources(OrbitTrace PRIVATE
    trace_embedded.cpp
  )
else()
  target_sources(OrbitTrace PRIVATE
    trace_simulator.cpp
  )
endif()

# Set target properties
set_target_properties(OrbitTrace PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)

# Include directories
target_include_directories(OrbitTrace PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
)

# Link libraries
target_link_libraries(OrbitTrace PUBLIC
  Aurora
  Chimera
)

# Add nanopb dependency for embedded builds
if(NOT SIMULATOR)
  target_link_libraries(OrbitTrace PUBLIC
    nanopb
  )
endif()

# Add TCP server dependency for simulator builds
if(SIMULATOR)
  target_link_libraries(OrbitTrace PUBLIC
    OrbitSimulator
  )
endif()

# Compiler definitions
target_compile_definitions(OrbitTrace PRIVATE
  ORBIT_TRACE_MODULE
)

# Install target
install(TARGETS OrbitTrace
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(FILES
  trace_types.hpp
  trace_registry.hpp
  trace_interface.hpp
  trace_examples.hpp
  DESTINATION include/trace
)

# Install embedded headers
if(NOT SIMULATOR)
  install(FILES
    trace_embedded.hpp
    DESTINATION include/trace
  )
else()
  install(FILES
    trace_simulator.hpp
    DESTINATION include/trace
  )
endif()
