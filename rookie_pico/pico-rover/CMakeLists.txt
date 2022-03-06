# set header paths
set(HEADERS include/definitions.h)

add_executable(rover_peri
        ${HEADERS}      # include headers defined above
        src/main.c
        )

# pull in common dependencies and additional uart hardware support
target_link_libraries(rover_peri 
        pico_stdlib 
        hardware_uart
        hardware_i2c
        hardware_pwm
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(rover_peri 1)
pico_enable_stdio_uart(rover_peri 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(rover_peri)

