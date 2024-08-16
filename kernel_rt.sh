#!/bin/bash

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

# Function to display a message in yellow (informational)
display_info() {
    echo -e "${YELLOW}$1${NC}"
}

# Function to display a success message in green
display_success() {
    echo -e "${GREEN}$1${NC}"
}

# Function to display an error message in red
display_error() {
    echo -e "${RED}$1${NC}"
}

install_packages() {
    display_info "Installing required packages..."
    sudo apt-get update || { display_error "Failed to update package list"; exit 1; }
    sudo apt-get install -y build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison
    check_status
    display_success "Required packages installed successfully."
}

check_kernel_version() {
    display_info "Checking the current kernel version..."
    CURRENT_KERNEL=$(uname -r)
    check_status
    display_success "Current kernel version: $CURRENT_KERNEL"
}

create_temp_directory() {
    TEMP_DIR="${HOME}/rt_kernel_build"
    display_info "Creating a temporary directory for kernel build: $TEMP_DIR"
    mkdir -p "$TEMP_DIR"
    check_status
    cd "$TEMP_DIR" || { display_error "Failed to navigate to $TEMP_DIR"; exit 1; }
    display_success "Temporary directory created and navigated to: $TEMP_DIR"
}

download_kernel_and_patch() {
    # Provide current kernel version details
    display_info "Checking the current kernel version..."
    CURRENT_KERNEL=$(uname -r)
    check_status
    display_success "Current kernel version: $CURRENT_KERNEL"

    # Provide example links for reference
    echo -e "${YELLOW}Please use the following links as a reference:${NC}"
    echo -e "${YELLOW}PREEMPT_RT patch versions: https://wiki.linuxfoundation.org/realtime/preempt_rt_versions${NC}"
    echo -e "${YELLOW}Kernel base URL example: https://mirrors.edge.kernel.org/pub/linux/kernel/v6.x/${NC}"
    echo -e "${YELLOW}Patch base URL example: https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.6/${NC}"

    # Prompt the user for the kernel version they want to use
    read -p "Enter the PREEMPT_RT kernel version you want to use (e.g., 6.6.7): " KERNEL_VERSION
    check_status
    PATCH_VERSION="${KERNEL_VERSION}-rt"

    # Prompt the user for the Kernel Base URL
    read -p "Enter the Kernel Base URL (e.g., https://mirrors.edge.kernel.org/pub/linux/kernel/v6.x/): " KERNEL_BASE_URL
    check_status

    # Prompt the user for the Patch Base URL
    read -p "Enter the Patch Base URL (e.g., https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.6/): " PATCH_BASE_URL
    check_status

    display_info "Downloading kernel and patch files for version $KERNEL_VERSION..."

    # Define patterns for the files we need
    KERNEL_PATTERN="linux-${KERNEL_VERSION}*.tar.xz"
    KERNEL_SIGN_PATTERN="linux-${KERNEL_VERSION}*.tar.sign"
    PATCH_PATTERN="patch-${PATCH_VERSION}*.patch.xz"
    PATCH_SIGN_PATTERN="patch-${PATCH_VERSION}*.patch.sign"

    # Download the kernel source and its signature
    wget "${KERNEL_BASE_URL}${KERNEL_PATTERN}" -O "linux-${KERNEL_VERSION}.tar.xz"
    check_status
    wget "${KERNEL_BASE_URL}${KERNEL_SIGN_PATTERN}" -O "linux-${KERNEL_VERSION}.tar.sign"
    check_status

    # Download the patch and its signature
    wget "${PATCH_BASE_URL}${PATCH_PATTERN}" -O "patch-${PATCH_VERSION}.patch.xz"
    check_status
    wget "${PATCH_BASE_URL}${PATCH_SIGN_PATTERN}" -O "patch-${PATCH_VERSION}.patch.sign"
    check_status

    display_success "Kernel and patch files for version $KERNEL_VERSION downloaded successfully."
}

decompress_files() {
    display_info "Decompressing downloaded files..."

    # Decompress the kernel source file
    if [ -f "linux-${KERNEL_VERSION}.tar.xz" ]; then
        display_info "Decompressing linux-${KERNEL_VERSION}.tar.xz..."
        xz -d "linux-${KERNEL_VERSION}.tar.xz"
        check_status
        display_success "Decompressed linux-${KERNEL_VERSION}.tar.xz successfully."
    else
        display_error "Kernel source file linux-${KERNEL_VERSION}.tar.xz not found!"
        exit 1
    fi

    # Decompress the patch file
    if [ -f "patch-${PATCH_VERSION}.patch.xz" ]; then
        display_info "Decompressing patch-${PATCH_VERSION}.patch.xz..."
        xz -d "patch-${PATCH_VERSION}.patch.xz"
        check_status
        display_success "Decompressed patch-${PATCH_VERSION}.patch.xz successfully."
    else
        display_error "Patch file patch-${PATCH_VERSION}.patch.xz not found!"
        exit 1
    fi

    display_success "All files decompressed successfully."
}

verify_downloaded_files() {
    display_info "Verifying the integrity of the downloaded files..."

    # Import public keys from kernel developers
    display_info "Importing public keys from kernel developers..."
    gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org
    check_status
    display_success "Public keys imported successfully."

    # Search for and import the patch author's key
    display_info "Searching for and importing the patch author's key..."
    
    # Function to try multiple keyservers
    import_patch_author_key() {
        local key="zanussi"
        local keyservers=(
            "hkp://keyserver.ubuntu.com"
            "hkp://pgp.mit.edu"
            "hkp://keys.openpgp.org"
        )
        
        for server in "${keyservers[@]}"; do
            display_info "Trying keyserver: $server"
            gpg2 --keyserver "$server" --search-keys "$key" && return 0
            display_error "Failed to retrieve key from $server"
        done
        
        display_error "All keyserver attempts failed. Please check your network or try again later."
        exit 1
    }

    # Call the function to import the key
    import_patch_author_key
    check_status
    display_success "Patch author key imported successfully."

    # Verify the kernel source signature
    display_info "Verifying the kernel source signature..."
    gpg2 --verify "linux-${KERNEL_VERSION}.tar.sign"
    check_status
    display_success "Kernel source signature verified successfully."

    # Verify the patch signature
    display_info "Verifying the patch signature..."
    gpg2 --verify "patch-${PATCH_VERSION}.patch.sign"
    check_status
    display_success "Patch signature verified successfully."

    display_success "All files verified successfully."
}

extract_and_patch_kernel() {
    display_info "Extracting the kernel source tar archive..."

    # Extract the tar archive
    tar xf "linux-${KERNEL_VERSION}.tar"
    check_status
    display_success "Kernel source extracted successfully."

    # Navigate into the extracted kernel source directory
    cd "linux-${KERNEL_VERSION}" || { display_error "Failed to enter kernel source directory"; exit 1; }

    # Apply the real-time patch
    display_info "Applying the real-time patch..."
    xzcat "../patch-${PATCH_VERSION}.patch.xz" | patch -p1
    check_status
    display_success "Real-time patch applied successfully."
}

configure_kernel() {
    display_info "Configuring the kernel..."

    # Run make oldconfig
    make oldconfig
    check_status
    display_success "Kernel configured successfully."

    display_info "Please ensure to select 'Fully Preemptible Kernel (RT)' during the configuration."
}

compile_kernel() {
    display_info "Building the kernel... This may take some time."

    # Clean up previous builds if any
    make clean
    check_status

    # Compile the kernel and build the Debian packages
    make -j "$(getconf _NPROCESSORS_ONLN)" deb-pkg
    check_status
    display_success "Kernel compiled and Debian packages created successfully."
}

install_compiled_kernel() {
    display_info "Installing the compiled kernel packages..."

    # Install the headers and image packages (excluding -dbg)
    sudo apt install ../linux-headers-${KERNEL_VERSION}-rt*_*.deb ../linux-image-${KERNEL_VERSION}-rt*_*.deb
    check_status
    display_success "Kernel and headers installed successfully."
}
    