
idf_component_register(
    SRCS 
        "xbee_serial_esp32.c"
        "xbee_platform_esp32.c"
        "../../src/xbee/xbee_device.c"
        "../../src/xbee/xbee_atcmd.c"
        "../../src/util/swapbytes.c"
        "../../src/wpan/wpan_types.c"
        "../../src/xbee/xbee_scan.c"
        "../../samples/common/_atinter.c"
        "../../src/util/hexdump.c"
        "../../src/util/hexstrtobyte.c"
    INCLUDE_DIRS 
        "../../include"
        "../../samples/common/"
        "./"
    PRIV_REQUIRES
        "esp_timer"
        "esp_driver_usb_serial_jtag"
        "driver"
        "freertos"
)
target_compile_definitions(${COMPONENT_LIB} PRIVATE XBEE_PLATFORM_HEADER="\"${CMAKE_CURRENT_LIST_DIR}/../../include/platform_config.h\"")
