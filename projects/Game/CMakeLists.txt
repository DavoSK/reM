add_library(Game SHARED
	src/main.cpp
	src/C_Vehicle.cpp
)

set_target_properties(
    Game 
    PROPERTIES
        OUTPUT_NAME "Game"
        SUFFIX ".asi"
)

include_directories(
	src
	../LS3DF/src
	../plugin/src
)

target_link_libraries(Game LS3DF plugin)
