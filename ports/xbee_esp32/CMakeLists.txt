# file(GLOB XBEE_SRCS "../../src/xbee/*.c")
# list(FILTER XBEE_SRCS EXCLUDE REGEX "xbee_ipv4.c")  # skip ipv4.c
# list(FILTER XBEE_SRCS EXCLUDE REGEX "xbee_sms.c")

idf_component_register(
    SRCS 
#        ${XBEE_SRCS}
        "xbee_serial_esp32.c"
        "xbee_platform_esp32.c"
        "../../src/xbee/xbee_device.c"
        "../../src/xbee/xbee_atcmd.c"
        "../../src/util/swapbytes.c"
    INCLUDE_DIRS 
        "../../include"
    PRIV_REQUIRES
        "esp_timer"
        "esp_driver_usb_serial_jtag"
        "driver"
        "freertos"
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE XBEE_PLATFORM_HEADER="\"${CMAKE_CURRENT_LIST_DIR}/../../include/platform_config.h\"")
