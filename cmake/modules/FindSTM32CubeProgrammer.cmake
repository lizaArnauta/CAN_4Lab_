# - Find STM32CubeProgrammer CLI
# This module defines:
#
#   STM32CubeProgrammer_CLI_EXECUTABLE  - path to STM32_Programmer_CLI executable
#   STM32CubeProgrammer_CLI_FOUND       - whether the tool was found
#
# Usage:
#   find_package(STM32CubeProgrammer REQUIRED)
#
# Example:
#   add_custom_target(flash
#       COMMAND ${STM32CubeProgrammer_CLI_EXECUTABLE} -c port=usb1 -w firmware.bin 0x08000000 -rst
#   )

find_program(STM32CubeProgrammer_CLI_EXECUTABLE
    NAMES STM32_Programmer_CLI STM32_Programmer_CLI.exe
    PATHS
        "$ENV{ProgramFiles}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "$ENV{ProgramFiles\(x86\)}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "$ENV{HOME}/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin"
        "/usr/local/STMicroelectronics/STM32CubeProgrammer/bin"
        "/usr/bin"
        "/usr/local/bin"
        "/opt/STMicroelectronics/STM32CubeProgrammer/bin"
    DOC "Path to STM32_Programmer_CLI executable"
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(STM32CubeProgrammer
    REQUIRED_VARS STM32CubeProgrammer_CLI_EXECUTABLE
    FAIL_MESSAGE "STM32CubeProgrammer CLI not found. Please install STM32CubeProgrammer."
)

mark_as_advanced(STM32CubeProgrammer_CLI_EXECUTABLE)