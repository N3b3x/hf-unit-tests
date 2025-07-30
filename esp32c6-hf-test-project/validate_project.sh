#!/bin/bash

# ESP32-C6 HardFOC Test Project Validation Script
# This script validates the project setup and dependencies

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[✅]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[⚠️ ]${NC} $1"
}

print_error() {
    echo -e "${RED}[❌]${NC} $1"
}

VALIDATION_PASSED=true

validate_item() {
    local description="$1"
    local test_command="$2"
    
    printf "%-50s" "$description"
    
    if eval "$test_command" &>/dev/null; then
        echo -e "${GREEN}✅${NC}"
        return 0
    else
        echo -e "${RED}❌${NC}"
        VALIDATION_PASSED=false
        return 1
    fi
}

validate_file() {
    local description="$1"
    local file_path="$2"
    
    validate_item "$description" "[[ -f '$file_path' ]]"
}

validate_dir() {
    local description="$1"
    local dir_path="$2"
    
    validate_item "$description" "[[ -d '$dir_path' ]]"
}

validate_command() {
    local description="$1"
    local command="$2"
    
    validate_item "$description" "command -v $command"
}

print_info "ESP32-C6 HardFOC Test Project Validation"
print_info "========================================"
echo ""

# Environment validation
print_info "Environment Validation:"
validate_command "ESP-IDF Python" "python"
validate_command "ESP-IDF idf.py" "idf.py"
validate_item "ESP-IDF Path set" "[[ -n '$IDF_PATH' ]]"
validate_item "ESP-IDF Tools installed" "[[ -d '$IDF_PATH/tools' ]]"

echo ""

# Project structure validation
print_info "Project Structure Validation:"
validate_file "Main CMakeLists.txt" "CMakeLists.txt"
validate_file "SDK Config defaults" "sdkconfig.defaults"
validate_file "Partition table" "partitions.csv"
validate_file "Build script" "build_and_flash.sh"
validate_item "Build script executable" "[[ -x 'build_and_flash.sh' ]]"

echo ""

# Main component validation
print_info "Main Component Validation:"
validate_dir "Main directory" "main"
validate_file "Main CMakeLists.txt" "main/CMakeLists.txt"
validate_file "Main application" "main/main.cpp"
validate_file "GPIO config implementation" "main/hf_gpio_config.cpp"
validate_file "Communication tests" "main/test_communication.cpp"
validate_file "Hardware tests" "main/test_hardware.cpp"

echo ""

# Headers validation
print_info "Header Files Validation:"
validate_dir "Include directory" "main/include"
validate_file "GPIO config header" "main/include/hf_gpio_config.hpp"
validate_file "Communication test header" "main/include/test_communication.hpp"
validate_file "Hardware test header" "main/include/test_hardware.hpp"

echo ""

# HardFOC component validation
print_info "HardFOC Component Validation:"
validate_dir "Components directory" "components"
validate_dir "HF wrapper component" "components/hf_internal_interface_wrap"
validate_file "HF wrapper CMakeLists.txt" "components/hf_internal_interface_wrap/CMakeLists.txt"

echo ""

# HardFOC library validation
print_info "HardFOC Library Validation:"
HF_PATH="../tests-internal-interface-wrap/hf-internal-interface-wrap"
validate_dir "HF library root" "$HF_PATH"
validate_dir "HF include directory" "$HF_PATH/inc"
validate_dir "HF base headers" "$HF_PATH/inc/base"
validate_dir "HF ESP32 headers" "$HF_PATH/inc/mcu/esp32"
validate_dir "HF ESP32 utils" "$HF_PATH/inc/mcu/esp32/utils"
validate_dir "HF source directory" "$HF_PATH/src"
validate_dir "HF ESP32 sources" "$HF_PATH/src/mcu/esp32"

echo ""

# Key header files validation
print_info "Key Header Files Validation:"
validate_file "McuSelect.h" "$HF_PATH/inc/utils/McuSelect.h"
validate_file "HardwareTypes.h" "$HF_PATH/inc/base/HardwareTypes.h"
validate_file "EspTypes.h" "$HF_PATH/inc/mcu/esp32/utils/EspTypes.h"
validate_file "BaseAdc.h" "$HF_PATH/inc/base/BaseAdc.h"
validate_file "BaseGpio.h" "$HF_PATH/inc/base/BaseGpio.h"
validate_file "BaseSpi.h" "$HF_PATH/inc/base/BaseSpi.h"
validate_file "BaseI2c.h" "$HF_PATH/inc/base/BaseI2c.h"

echo ""

# ESP32 implementation validation
print_info "ESP32 Implementation Validation:"
validate_file "EspAdc.h" "$HF_PATH/inc/mcu/esp32/EspAdc.h"
validate_file "EspAdc.cpp" "$HF_PATH/src/mcu/esp32/EspAdc.cpp"
validate_file "EspGpio.h" "$HF_PATH/inc/mcu/esp32/EspGpio.h"
validate_file "EspGpio.cpp" "$HF_PATH/src/mcu/esp32/EspGpio.cpp"
validate_file "EspSpi.h" "$HF_PATH/inc/mcu/esp32/EspSpi.h"
validate_file "EspSpi.cpp" "$HF_PATH/src/mcu/esp32/EspSpi.cpp"
validate_file "EspI2c.h" "$HF_PATH/inc/mcu/esp32/EspI2c.h"
validate_file "EspI2c.cpp" "$HF_PATH/src/mcu/esp32/EspI2c.cpp"

echo ""

# VS Code configuration validation
print_info "VS Code Configuration Validation:"
validate_dir "VS Code directory" ".vscode"
validate_file "VS Code tasks" ".vscode/tasks.json"
validate_file "VS Code C++ properties" ".vscode/c_cpp_properties.json"

echo ""

# Build system validation
print_info "Build System Test:"
if validate_item "Can set ESP32-C6 target" "idf.py set-target esp32c6"; then
    print_success "ESP32-C6 target can be set successfully"
else
    print_error "Failed to set ESP32-C6 target"
fi

echo ""

# Final validation result
print_info "Validation Summary:"
print_info "=================="

if [ "$VALIDATION_PASSED" = true ]; then
    print_success "All validations PASSED! ✨"
    print_success "The project is ready to build and flash."
    echo ""
    print_info "Next steps:"
    print_info "1. Connect your ESP32-C6 development board"
    print_info "2. Run: ./build_and_flash.sh"
    print_info "3. Or use VS Code tasks: Ctrl+Shift+P -> Tasks: Run Task"
    echo ""
    exit 0
else
    print_error "Some validations FAILED! ⚠️"
    print_error "Please fix the issues above before proceeding."
    echo ""
    print_info "Common fixes:"
    print_info "- Ensure ESP-IDF is properly installed and sourced"
    print_info "- Check that hf-internal-interface-wrap is in the correct location"
    print_info "- Verify all required files are present"
    echo ""
    exit 1
fi
