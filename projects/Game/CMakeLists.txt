add_library(Game SHARED
	src/main.cpp
	src/C_Vehicle.cpp
)

include_directories(
	src
	../LS3DF/src
	../plugin/src
)

target_link_libraries(Game LS3DF plugin)
