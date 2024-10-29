add_executable(${NAME}
        main.c
        pico_init.c
        )

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	pico_stdlib
    FreeRTOS-Kernel-Heap4
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
	)
	
target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )
    
pico_btstack_make_gatt_header(${NAME} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/GATT_Service/server_gattfile.gatt")
     
target_compile_definitions(${NAME} PRIVATE
    configNUM_CORES=2
    ENABLE_BLE
)


# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, disable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 0)
