# SPDX-License-Identifier: Apache-2.0

set_ifndef(C++ g++)

# Configures CMake for using GCC, this script is re-used by several
# GCC-based toolchains

find_program(CMAKE_C_COMPILER ${CROSS_COMPILE}${CC} PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)
if(${CMAKE_C_COMPILER} STREQUAL CMAKE_C_COMPILER-NOTFOUND)
  message(FATAL_ERROR "C compiler ${CROSS_COMPILE}${CC} not found - Please check your toolchain installation")
endif()

if(CONFIG_CPLUSPLUS)
  set(cplusplus_compiler ${CROSS_COMPILE}${C++})
else()
  if(EXISTS ${CROSS_COMPILE}${C++})
    set(cplusplus_compiler ${CROSS_COMPILE}${C++})
  else()
    # When the toolchain doesn't support C++, and we aren't building
    # with C++ support just set it to something so CMake doesn't
    # crash, it won't actually be called
    set(cplusplus_compiler ${CMAKE_C_COMPILER})
  endif()
endif()
find_program(CMAKE_CXX_COMPILER ${cplusplus_compiler} PATHS ${TOOLCHAIN_HOME} NO_DEFAULT_PATH)

set(NOSTDINC "")

# Note that NOSYSDEF_CFLAG may be an empty string, and
# set_ifndef() does not work with empty string.
if(NOT DEFINED NOSYSDEF_CFLAG)
  set(NOSYSDEF_CFLAG -undef)
endif()

list(APPEND NOSTDINC "${TOOLCHAIN_HOME}/include/lega-c")
list(APPEND NOSTDINC "${TOOLCHAIN_HOME}/support/generic/h")

list(APPEND TOOLCHAIN_C_FLAGS -mcpu=GENERIC-16DSP-CH)
list(APPEND TOOLCHAIN_LD_FLAGS
	-mcpu=GENERIC-16DSP-CH
	--handles
	--no-isr
	--no-ivt
	--no-smart-io)

# For CMake to be able to test if a compiler flag is supported by the
# toolchain we need to give CMake the necessary flags to compile and
# link a dummy C file.
#
# CMake checks compiler flags with check_c_compiler_flag() (Which we
# wrap with target_cc_option() in extentions.cmake)
foreach(isystem_include_dir ${NOSTDINC})
  list(APPEND isystem_include_flags -isystem "\"${isystem_include_dir}\"")
endforeach()

# The CMAKE_REQUIRED_FLAGS variable is used by check_c_compiler_flag()
# (and other commands which end up calling check_c_source_compiles())
# to add additional compiler flags used during checking. These flags
# are unused during "real" builds of Zephyr source files linked into
# the final executable.
#
# Appending onto any existing values lets users specify
# toolchain-specific flags at generation time.
list(APPEND CMAKE_REQUIRED_FLAGS
  -nostartfiles
  -nostdlib
  ${isystem_include_flags}
  -Wl,--unresolved-symbols=ignore-in-object-files
  -Wl,--entry=0 # Set an entry point to avoid a warning
  )
string(REPLACE ";" " " CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS}")

# Load toolchain_cc-family macros
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_freestanding.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_security_fortify.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_security_canaries.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_optimizations.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_cpp.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_asm.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_baremetal.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_warnings.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_imacros.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_base.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_coverage.cmake)
include(${ZEPHYR_BASE}/cmake/compiler/${COMPILER}/target_sanitizers.cmake)
