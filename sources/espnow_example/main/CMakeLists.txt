file(GLOB SRCS "ros.c" "app_espnow.c")  # Add this line to gather all source files
idf_component_register(SRCS ${SRCS}  # Update this line to use the gathered source files
                    INCLUDE_DIRS "."
                    )