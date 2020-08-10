# SPDX-License-Identifier: Apache-2.0

set_ifndef(XC16_TOOLCHAIN_PATH "$ENV{XC16_TOOLCHAIN_PATH}")
set(       XC16_TOOLCHAIN_PATH ${XC16_TOOLCHAIN_PATH} CACHE PATH "xc16 install directory")
assert(    XC16_TOOLCHAIN_PATH "XC16_TOOLCHAIN_PATH is not set")

if(NOT EXISTS ${XC16_TOOLCHAIN_PATH})
  message(FATAL_ERROR "Nothing found at XC16_TOOLCHAIN_PATH: '${XC16_TOOLCHAIN_PATH}'")
endif()

set(TOOLCHAIN_HOME ${XC16_TOOLCHAIN_PATH})

set(COMPILER xc16)
set(LINKER ld)
set(BINTOOLS gnu)

set(CROSS_COMPILE_TARGET xc16)

list(APPEND TOOLCHAIN_C_FLAGS
  -imacros${ZEPHYR_BASE}/include/toolchain/xc16_missing_defs.h
  )

set(CROSS_COMPILE ${TOOLCHAIN_HOME}/bin/${CROSS_COMPILE_TARGET}-)
set(TOOLCHAIN_HAS_NEWLIB OFF CACHE BOOL "True if toolchain supports newlib")

message(STATUS "Found toolchain: xc16 (${XC16_TOOLCHAIN_PATH})")
