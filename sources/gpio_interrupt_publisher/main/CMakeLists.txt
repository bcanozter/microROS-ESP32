file(GLOB SRCS "main.c" "*.c" "*.h")  # Add this line to gather all source files
idf_component_register(SRCS ${SRCS}  # Update this line to use the gathered source files
                    INCLUDE_DIRS ""
                    )