# set header paths
# set(HEADERS include/definitions.h)
include_directories(include)

add_executable(rover_peri
        # ${HEADERS}      # include headers defined above
        include/main.h
        include/comms.h
        include/definitions.h
        include/motors.h
        include/config.h
        src/main.c
        src/comms.c
        src/motors.c
        src/config.c
        )

# pull in common dependencies and additional uart hardware support
target_link_libraries(rover_peri 
        pico_stdlib
        pico_multicore
        pico_sync 
        hardware_uart
        hardware_i2c
        hardware_pwm
        hardware_gpio
        )

# enable usb output, disable uart output
pico_enable_stdio_usb(rover_peri 1)
pico_enable_stdio_uart(rover_peri 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(rover_peri)

