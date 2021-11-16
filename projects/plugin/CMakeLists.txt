add_library(plugin
	src/reversiblehooks/ReversibleHooks.cpp
    src/reversiblehooks/SSimpleReversibleHook.cpp
    src/reversiblehooks/SVirtualReversibleHook.cpp
)

include_directories(
	src
    src/reversiblehooks
)