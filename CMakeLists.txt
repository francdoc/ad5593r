add_executable(ad5593r_i2c 
        ad5593r_i2c.c
        )

target_link_libraries(ad5593r_i2c
        pico_stdlib              # for core functionality
        hardware_i2c             # for i2c
        pico_cyw43_arch_none     # we need Wifi to access the GPIO, but we don't need anything else
        )

# create map/bin/hex file etc.
pico_add_extra_outputs(ad5593r_i2c)

# add url via pico_set_program_url
example_auto_set_url(ad5593r_i2c)

pico_enable_stdio_usb(ad5593r_i2c 1)
pico_enable_stdio_uart(ad5593r_i2c 0)