# No special flags are needed for xc16.
include(${ZEPHYR_BASE}/cmake/compiler/gcc/compiler_flags.cmake)

# Unset unsupported warning
set_compiler_property(PROPERTY warning_extended)
