#!/bin/bash

# Script to set up an Ubuntu system with a PREEMPT_RT kernel
# This script follows the guidelines provided in the documentation.

# Define colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to handle errors and exit
handle_error() {
    echo -e "${RED}Error on line $1${NC}"
    exit 1
}

# Trap errors and pass the line number to the handle_error function
trap 'handle_error $LINENO' ERR

# Function to check the last command status and exit if failed
check_status() {
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error encountered, exiting...${NC}"
        exit 1
    fi
}

# Install required tools and packages
install_tools() {
    echo -e "${YELLOW}Installing required packages...${NC}"
    sudo apt-get update || { echo -e "${RED}Failed to update package list${NC}"; exit 1; }
    sudo apt-get install -y build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison
    check_status
    echo -e "${GREEN}Required packages installed successfully.${NC}"
}

# Check the currently installed kernel version
check_kernel_version() {
    echo -e "${YELLOW}Checking the current kernel version...${NC}"
    CURRENT_KERNEL=$(uname -r)
    check_status
    echo -e "${GREEN}Current kernel version: $CURRENT_KERNEL${NC}"
}

# Function to download and verify files based on pattern matching
download_and_verify_files() {
    local base_url=$1
    local pattern=$2

    echo -e "${YELLOW}Searching for files matching pattern: $pattern${NC}"
    
    # List the files and download the ones that match the pattern
    wget -r -l1 -nd -np -A "$pattern" -e robots=off --accept-regex "$pattern" "$base_url" -P "$TEMP_DIR"
    check_status
    
    echo -e "${GREEN}Files matching pattern '$pattern' downloaded successfully.${NC}"
}

# Function to download the kernel and patch files
download_kernel_and_patch() {
    # Define patterns for the files we need
    KERNEL_PATTERN="linux-${KERNEL_VERSION}*.tar.xz*"
    PATCH_PATTERN="patch-${PATCH_VERSION}*.patch.xz*"

    # Download the kernel and patch files
    download_and_verify_files "$KERNEL_BASE_URL" "$KERNEL_PATTERN"
    download_and_verify_files "$PATCH_BASE_URL" "$PATCH_PATTERN"

    # Decompress the downloaded files
    echo -e "${YELLOW}Decompressing downloaded files...${NC}"
    for file in "$TEMP_DIR"/*.xz; do
        if [ -f "${file%.xz}" ]; then
            echo -e "${YELLOW}File ${file%.xz} already decompressed, skipping...${NC}"
        else
            unxz "$file"
            check_status
            echo -e "${GREEN}Decompressed $file successfully.${NC}"
        fi
    done
    echo -e "${GREEN}All downloaded files decompressed successfully.${NC}"

    # Verification of the downloaded files
    echo -e "${YELLOW}Verifying downloaded files...${NC}"

    # Import public keys for verification
    echo -e "${YELLOW}Importing public keys from kernel developers...${NC}"
    gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org
    check_status
    echo -e "${GREEN}Public keys imported successfully.${NC}"

    # Function to attempt importing a key from multiple keyservers
    import_patch_author_key() {
        local key="zanussi"
        local keyservers=(
            "hkp://keyserver.ubuntu.com"
            "hkp://pgp.mit.edu"
            "hkp://keys.openpgp.org"
        )
        
        for server in "${keyservers[@]}"; do
            echo -e "${YELLOW}Trying keyserver: $server${NC}"
            gpg2 --keyserver "$server" --search-keys "$key" && return 0
            echo -e "${RED}Failed to retrieve key from $server${NC}"
        done
        
        echo -e "${RED}All keyserver attempts failed. Please check your network or try again later.${NC}"
        exit 1
    }

    echo -e "${YELLOW}Searching and importing the key for the patch author...${NC}"
    import_patch_author_key
    check_status
    echo -e "${GREEN}Patch author key imported successfully.${NC}"

    # Verify the kernel source signature
    for sign_file in "$TEMP_DIR"/*.sign; do
        if [[ "$sign_file" == *"linux-"* ]]; then
            echo -e "${YELLOW}Verifying the kernel source signature for $sign_file...${NC}"
            gpg2 --verify "$sign_file"
            check_status
            echo -e "${GREEN}Kernel source signature verified successfully.${NC}"
        elif [[ "$sign_file" == *"patch-"* ]]; then
            echo -e "${YELLOW}Verifying the patch signature for $sign_file...${NC}"
            gpg2 --verify "$sign_file"
            check_status
            echo -e "${GREEN}Patch signature verified successfully.${NC}"
        fi
    done

    echo -e "${GREEN}Kernel sources and patch have been successfully downloaded, verified, and prepared.${NC}"
}

# 
compile_kernel() {
    # Display the current state for user satisfaction
    echo -e "${YELLOW}Current state:${NC}"
    cat "$STATE_FILE"

    # Compilation and Patch Application

    # Extract the tar archive
    echo -e "${YELLOW}Extracting the kernel source tar archive...${NC}"
    tar xf "linux-$KERNEL_VERSION.tar"
    check_status
    echo -e "${GREEN}Kernel source extracted successfully.${NC}"

    # Navigate into the extracted kernel source directory
    cd "linux-$KERNEL_VERSION" || { echo -e "${RED}Failed to enter kernel source directory${NC}"; exit 1; }

    # Apply the real-time patch
    echo -e "${YELLOW}Applying the real-time patch...${NC}"
    xzcat "../patch-$PATCH_VERSION.patch.xz" | patch -p1
    check_status
    echo -e "${GREEN}Real-time patch applied successfully.${NC}"

    # Configure the kernel
    echo -e "${YELLOW}Configuring the kernel...${NC}"
    make oldconfig
    check_status
    echo -e "${GREEN}Kernel configured successfully.${NC}"

    # Build the kernel
    echo -e "${YELLOW}Building the kernel... This may take some time.${NC}"

    # Clean up previous builds if any
    make clean
    check_status

    # Installing missing dependencies
    sudo apt-get install -y libncurses-dev libssl-dev flex bison
    check_status

    # Compile the kernel. Set V=1 for verbose output in make command
    make -j "$(getconf _NPROCESSORS_ONLN)" deb-pkg
    check_status
    echo -e "${GREEN}Kernel compiled successfully.${NC}"

    # Compile the modules
    echo -e "${YELLOW}Compiling the kernel modules...${NC}"
    make -j "$(getconf _NPROCESSORS_ONLN)" modules
    check_status
    echo -e "${GREEN}Kernel modules compiled successfully.${NC}"

    # Install the modules
    echo -e "${YELLOW}Installing the kernel modules...${NC}"
    sudo make modules_install
    check_status
    echo -e "${GREEN}Kernel modules installed successfully.${NC}"

    # Install the kernel
    echo -e "${YELLOW}Installing the kernel...${NC}"
    sudo make install
    check_status
    echo -e "${GREEN}Kernel installed successfully.${NC}"
}

