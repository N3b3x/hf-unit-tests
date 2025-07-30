#!/bin/bash

# ESP32-C6 HardFOC Test Project Build and Flash Script
# This script automates the build, flash, and monitor process

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Default values
PORT=""
BAUDRATE=115200
BUILD_ONLY=false
CLEAN_BUILD=false
MONITOR_ONLY=false
FLASH_ONLY=false

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to show usage
show_usage() {
    echo "ESP32-C6 HardFOC Test Project Build Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -p, --port PORT       Serial port (e.g., /dev/ttyUSB0, COM3)"
    echo "  -b, --baudrate RATE   Baudrate for flashing (default: 115200)"
    echo "  -c, --clean          Clean build (fullclean before build)"
    echo "  --build-only         Build only, don't flash or monitor"
    echo "  --flash-only         Flash only, don't monitor"
    echo "  --monitor-only       Monitor only, don't build or flash"
    echo "  -h, --help           Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0                          # Build, flash, and monitor (auto-detect port)"
    echo "  $0 -p /dev/ttyUSB0         # Use specific port"
    echo "  $0 -c                      # Clean build"
    echo "  $0 --build-only            # Build only"
    echo "  $0 --monitor-only -p COM3  # Monitor only"
}

# Function to detect ESP32-C6 port automatically
detect_port() {
    print_info "Detecting ESP32-C6 port..."
    
    # Common port patterns for different OS
    if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        # Linux
        for port in /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/*ESP32*; do
            if [[ -e "$port" ]]; then
                echo "$port"
                return 0
            fi
        done
    elif [[ "$OSTYPE" == "darwin"* ]]; then
        # macOS
        for port in /dev/cu.usbserial* /dev/cu.SLAB* /dev/cu.wchusbserial*; do
            if [[ -e "$port" ]]; then
                echo "$port"
                return 0
            fi
        done
    elif [[ "$OSTYPE" == "msys" ]] || [[ "$OSTYPE" == "cygwin" ]]; then
        # Windows (Git Bash, Cygwin)
        for port in /dev/ttyS* COM*; do
            if [[ -e "$port" ]]; then
                echo "$port"
                return 0
            fi
        done
    fi
    
    return 1
}

# Function to check ESP-IDF environment
check_esp_idf() {
    if [[ -z "$IDF_PATH" ]]; then
        print_error "ESP-IDF environment not set. Please run: . \$IDF_PATH/export.sh"
        exit 1
    fi
    
    if ! command -v idf.py &> /dev/null; then
        print_error "idf.py not found. Please ensure ESP-IDF is properly installed."
        exit 1
    fi
    
    print_success "ESP-IDF environment detected: $IDF_PATH"
}

# Function to check if we're in the correct directory
check_project_dir() {
    if [[ ! -f "CMakeLists.txt" ]] || [[ ! -d "main" ]]; then
        print_error "Please run this script from the esp32c6-hf-test-project directory"
        exit 1
    fi
    
    if [[ ! -d "../tests-internal-interface-wrap/hf-internal-interface-wrap" ]]; then
        print_error "hf-internal-interface-wrap not found at expected location"
        print_error "Expected: ../tests-internal-interface-wrap/hf-internal-interface-wrap"
        exit 1
    fi
    
    print_success "Project directory and dependencies verified"
}

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -b|--baudrate)
            BAUDRATE="$2"
            shift 2
            ;;
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        --build-only)
            BUILD_ONLY=true
            shift
            ;;
        --flash-only)
            FLASH_ONLY=true
            shift
            ;;
        --monitor-only)
            MONITOR_ONLY=true
            shift
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            print_error "Unknown option: $1"
            show_usage
            exit 1
            ;;
    esac
done

# Main script execution
print_info "ESP32-C6 HardFOC Test Project Build Script"
print_info "==========================================="

# Check environment and project
check_esp_idf
check_project_dir

# Handle monitor-only mode
if [[ "$MONITOR_ONLY" == true ]]; then
    if [[ -z "$PORT" ]]; then
        PORT=$(detect_port)
        if [[ $? -ne 0 ]]; then
            print_error "Could not auto-detect port. Please specify with -p option."
            exit 1
        fi
        print_info "Auto-detected port: $PORT"
    fi
    
    print_info "Starting monitor on $PORT..."
    idf.py -p "$PORT" monitor
    exit 0
fi

# Set target to ESP32-C6
print_info "Setting target to ESP32-C6..."
idf.py set-target esp32c6

# Clean build if requested
if [[ "$CLEAN_BUILD" == true ]]; then
    print_info "Performing clean build..."
    idf.py fullclean
fi

# Build the project
if [[ "$FLASH_ONLY" != true ]]; then
    print_info "Building project..."
    start_time=$(date +%s)
    
    if idf.py build; then
        end_time=$(date +%s)
        build_time=$((end_time - start_time))
        print_success "Build completed in ${build_time} seconds"
    else
        print_error "Build failed!"
        exit 1
    fi
fi

# Exit if build-only mode
if [[ "$BUILD_ONLY" == true ]]; then
    print_success "Build-only mode completed"
    exit 0
fi

# Auto-detect port if not specified
if [[ -z "$PORT" ]]; then
    PORT=$(detect_port)
    if [[ $? -ne 0 ]]; then
        print_warning "Could not auto-detect port. Common ports:"
        print_warning "  Linux: /dev/ttyUSB0, /dev/ttyACM0"
        print_warning "  macOS: /dev/cu.usbserial-*"
        print_warning "  Windows: COM3, COM4, etc."
        read -p "Please enter the port manually: " PORT
        
        if [[ -z "$PORT" ]]; then
            print_error "Port not specified. Exiting."
            exit 1
        fi
    else
        print_info "Auto-detected port: $PORT"
    fi
fi

# Flash the project
print_info "Flashing to $PORT at ${BAUDRATE} baud..."
if idf.py -p "$PORT" -b "$BAUDRATE" flash; then
    print_success "Flash completed successfully"
else
    print_error "Flash failed!"
    print_warning "Try a lower baud rate or check connections"
    exit 1
fi

# Start monitoring unless flash-only mode
if [[ "$FLASH_ONLY" != true ]]; then
    print_info "Starting monitor (Press Ctrl+] to exit)..."
    print_info "You should see the test suite running..."
    echo ""
    
    # Give a moment for the device to reset
    sleep 2
    
    idf.py -p "$PORT" monitor
fi

print_success "Script completed successfully!"
