
idf_component_register(
    SRCS 
        "xbee_serial_esp32.c"
        "xbee_platform_esp32.c"
        # Zigbee interactive
        "../../src/xbee/xbee_device.c"
        "../../src/xbee/xbee_atcmd.c"
        "../../src/util/swapbytes.c"
        "../../src/wpan/wpan_types.c"
        "../../src/xbee/xbee_scan.c"
        "../../samples/common/_atinter.c"
        "../../src/util/hexdump.c"
        "../../src/util/hexstrtobyte.c"
        # Zigbee walker
        "../../src/xbee/xbee_wpan.c"
        "../../src/wpan/wpan_aps.c"
        "../../samples/common/_zigbee_walker.c"
        "../../samples/common/zigbee_walker.c"
        "../../src/zigbee/zigbee_zcl.c"
        "../../src/zigbee/zcl_types.c"
        "../../src/zigbee/zigbee_zdo.c"
        "../../src/util/swapcpy.c"
        "../../src/zigbee/zcl_client.c"
        "../../src/xbee/xbee_time.c"

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
