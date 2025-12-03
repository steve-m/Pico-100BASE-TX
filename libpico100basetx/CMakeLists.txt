# Note we are using INTERFACE so that the library can be configured per-app
# with compile-time defines

add_library(libpico100basetx INTERFACE)

target_sources(libpico100basetx INTERFACE
	${CMAKE_CURRENT_LIST_DIR}/pico100basetx.c
	${CMAKE_CURRENT_LIST_DIR}/pico100basetx.h
	)

target_include_directories(libpico100basetx INTERFACE ${CMAKE_CURRENT_LIST_DIR})
target_link_libraries(libpico100basetx INTERFACE
	pico_base_headers
	pico_util
	pico_multicore
	hardware_pio
	hardware_dma
	)

pico_generate_pio_header(libpico100basetx ${CMAKE_CURRENT_LIST_DIR}/mlt3_out.pio)
pico_generate_pio_header(libpico100basetx ${CMAKE_CURRENT_LIST_DIR}/pin_out.pio)
