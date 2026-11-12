# ============================================================
# 仓库根目录的 CMakeLists.txt
# 路径：CMakeLists.txt
#
# 2026-11-12 修改记录：
#   为 owlet_usb_switch.c 增加条件编译，兼容不同 Shield 下的配置。
# ============================================================
target_sources_ifdef(CONFIG_OWLET_BLE_LED app PRIVATE src/owlet_leds.c)
target_sources(app PRIVATE src/owlet_rgb_toggle.c)
target_sources_ifdef(CONFIG_OWLET_USB_CHARGER_SWITCH app PRIVATE src/owlet_usb_switch.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CONFIG_OWLET_BLE_LED)
  message(STATUS "Owlet: LED indicator driver enabled")
endif()
