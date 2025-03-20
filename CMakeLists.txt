file(GLOB_RECURSE SRCS_LIST *.c *.cpp)

idf_component_register( SRCS ${SRCS_LIST}
                        INCLUDE_DIRS src
                        REQUIRES main
                        )