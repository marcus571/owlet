# ============================================================
# 仓库根目录的 CMakeLists.txt
# 路径：CMakeLists.txt
#
# 2026-11-11 修改记录：
#   新增 include/ 目录到头文件搜索路径，用于 owlet_charger_event.h。
# ============================================================
target_sources_ifdef(CONFIG_OWLET_BLE_LED app PRIVATE src/owlet_leds.c)
target_sources(app PRIVATE src/owlet_rgb_toggle.c)
target_sources(app PRIVATE src/owlet_usb_switch.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

if(CONFIG_OWLET_BLE_LED)
  message(STATUS "Owlet: LED indicator driver enabled")
endif()
