idf_component_register(
    SRCS 
        "xbee_serial_esp32.c"
        "xbee_platform_esp32.c"
    INCLUDE_DIRS 
        "../../include"
    PRIV_REQUIRES
        "esp_timer"
        "esp_driver_usb_serial_jtag"
    
#    PRIV_DEFINES XBEE_PLATFORM_HEADER=\"platform_config.h\"

)
target_compile_definitions(${COMPONENT_LIB} PRIVATE XBEE_PLATFORM_HEADER="\"${CMAKE_CURRENT_LIST_DIR}/../../include/platform_config.h\"")
