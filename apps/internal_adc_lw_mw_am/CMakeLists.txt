add_executable(internal_adc_lw_mw_am
	internal_adc_lw_mw_am.c
)

target_compile_options(internal_adc_lw_mw_am PRIVATE -Wall)

target_link_libraries(internal_adc_lw_mw_am
	pico_stdlib
	hardware_adc
	hardware_dma
	pico_util
	libpico100basetx
)

# enable usb output, disable uart output
pico_enable_stdio_usb(internal_adc_lw_mw_am 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(internal_adc_lw_mw_am)
