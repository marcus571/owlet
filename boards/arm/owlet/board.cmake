# SPDX-License-Identifier: MIT

board_runner_args(nrfjprog "--nrf-family=NRF52")
board_runner_args(jlink "--device=nrf52" "--speed=4000")
board_runner_args(pyocd "--target=nrf52840" "--frequency=4000000")

# 2026-10-27 关键修复：为 UF2 runner 指定应用程序基地址
# 这确保生成的 UF2 文件将固件写入到 0x26000，而不是默认的 0x0。
board_runner_args(uf2 "--base-address=0x26000")

include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/nrfutil.board.cmake)
include(${ZEPHYR_BASE}/boards/common/uf2.board.cmake)
