# ============================================================
# 仓库根目录的 CMakeLists.txt
# 路径：CMakeLists.txt
#
# 2026-11-06 修改记录：
#   新增 src/owlet_rgb_toggle.c 的编译。
# ============================================================
target_sources_ifdef(CONFIG_OWLET_BLE_LED app PRIVATE src/owlet_leds.c)
target_sources(app PRIVATE src/owlet_rgb_toggle.c)

if(CONFIG_OWLET_BLE_LED)
  message(STATUS "Owlet: LED indicator driver enabled")
endif()
