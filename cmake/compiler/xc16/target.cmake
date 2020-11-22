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
list(APPEND NOSTDINC "${TOOLCHAIN_HOME}/support/generic/inc")
list(APPEND NOSTDINC "${TOOLCHAIN_HOME}/support/dsPIC33C/h")
list(APPEND NOSTDINC "${TOOLCHAIN_HOME}/support/dsPIC33C/inc")

list(APPEND TOOLCHAIN_C_FLAGS -mcpu=${CONFIG_SOC_PART_NUMBER} -save-temps)
list(APPEND TOOLCHAIN_LD_FLAGS
	-Wl,--handles
	-Wl,--no-isr
	-Wl,--no-ivt
	-Wl,--no-smart-io
	-lm)
set(LIBC_LIBRARY_DIR "\"${TOOLCHAIN_HOME}\"/lib")

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
  -L${TOOLCHAIN_HOME}/lib
  -llega-pic30 # Include library to provide __reset
  )
string(REPLACE ";" " " CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS}")
