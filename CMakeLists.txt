# ============================================================
# 仓库根目录的 CMakeLists.txt
# 路径：CMakeLists.txt
#
# 2026-11-14 修改记录：
#   移除 owlet_rgb_save.c 的编译。
# ============================================================
target_sources_ifdef(CONFIG_OWLET_BLE_LED app PRIVATE src/owlet_leds.c)
target_sources_ifdef(CONFIG_OWLET_USB_CHARGER_SWITCH app PRIVATE src/owlet_usb_switch.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CONFIG_OWLET_BLE_LED)
  message(STATUS "Owlet: LED indicator driver enabled")
endif()
