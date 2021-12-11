add_executable(sim
	sim.cpp
	cpu.cpp
	cu_maps.cpp
	elfload.cpp
)

target_include_directories(sim PUBLIC "${PROJECT_SOURCE_DIR}")
target_link_libraries(sim)