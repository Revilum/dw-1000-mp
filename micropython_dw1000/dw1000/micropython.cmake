# MicroPython DW1000 module CMake configuration

# Create interface library for DW1000 module
add_library(usermod_dw1000 INTERFACE)

# Add our source files
target_sources(usermod_dw1000 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/moddw1000.c
    ${CMAKE_CURRENT_LIST_DIR}/dw1000_hal.c
)

# Add the DW1000 driver source files
target_sources(usermod_dw1000 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/../../dw1000-driver/deca_device.c
    ${CMAKE_CURRENT_LIST_DIR}/../../dw1000-driver/deca_params_init.c
    ${CMAKE_CURRENT_LIST_DIR}/../../dw1000-driver/deca_range_tables.c
)

# Add include directories
target_include_directories(usermod_dw1000 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/../../dw1000-driver
)

# Add compile definitions if needed
target_compile_definitions(usermod_dw1000 INTERFACE
    # Add any necessary definitions here
)

# Add compile options
target_compile_options(usermod_dw1000 INTERFACE
    -Wno-unused-function
    -Wno-unused-variable
)

# Link to the usermod target
target_link_libraries(usermod INTERFACE usermod_dw1000)