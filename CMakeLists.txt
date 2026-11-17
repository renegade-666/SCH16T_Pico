if (TARGET tinyusb_device)
    add_executable(sch16t_basic
            main.cpp SCH16T_Pico.cpp
            )

    # pull in common dependencies
    target_link_libraries(sch16t_basic pico_stdlib hardware_spi hardware_gpio pico_float)

    # enable usb output, disable uart output
    pico_enable_stdio_usb(sch16t_basic 1)
    pico_enable_stdio_uart(sch16t_basic 0)

    # create map/bin/hex/uf2 file etc.
    pico_add_extra_outputs(sch16t_basic)

elseif(PICO_ON_DEVICE)
    message("Skipping hello_usb because TinyUSB submodule is not initialized in the SDK")
endif()
