# ============================================================
# 仓库根目录 CMakeLists.txt
# 路径：CMakeLists.txt
# 共享驱动编译 + 暴露 include/ 头文件路径（驱动↔UI 事件接口）。
# ============================================================
target_sources_ifdef(CONFIG_OWLET_BLE_LED app PRIVATE src/owlet_leds.c)
target_sources_ifdef(CONFIG_OWLET_USB_CHARGER_SWITCH app PRIVATE src/owlet_usb_switch.c)

target_include_directories(app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
