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
echo -e "${YELLOW}Installing required packages...${NC}"
sudo apt-get update || { echo -e "${RED}Failed to update package list${NC}"; exit 1; }
sudo apt-get install -y build-essential bc ca-certificates gnupg2 libssl-dev wget gawk flex bison
check_status
echo -e "${GREEN}Required packages installed successfully.${NC}"

# Check the currently installed kernel version
echo -e "${YELLOW}Checking the current kernel version...${NC}"
CURRENT_KERNEL=$(uname -r)
check_status
echo -e "${GREEN}Current kernel version: $CURRENT_KERNEL${NC}"

# Provide link to the PREEMPT_RT version library
RT_KERNEL_URL="https://wiki.linuxfoundation.org/realtime/preempt_rt_versions"
echo -e "${YELLOW}To choose an appropriate real-time kernel version, visit the following link:${NC}"
echo -e "${YELLOW}$RT_KERNEL_URL${NC}"
echo -e "${YELLOW}Please choose a kernel version close to your current version ($CURRENT_KERNEL).${NC}"

# Prompt the user to input the kernel version they want to use
read -p "Enter the PREEMPT_RT kernel version you want to use (e.g., 4.14.139): " KERNEL_VERSION
check_status
PATCH_VERSION="${KERNEL_VERSION}-rt"

# Prompt the user to input the kernel and patch base URLs
read -p "Enter the Kernel Base URL (e.g., https://www.kernel.org/pub/linux/kernel/v4.x/): " KERNEL_BASE_URL
check_status
read -p "Enter the Patch Base URL (e.g., https://cdn.kernel.org/pub/linux/kernel/projects/rt/4.14/): " PATCH_BASE_URL
check_status

# Create a temporary directory for building the real-time kernel
TEMP_DIR="${HOME}/rt_kernel_build"
echo -e "${YELLOW}Creating a temporary directory for kernel build: $TEMP_DIR${NC}"
mkdir -p "$TEMP_DIR"
check_status
cd "$TEMP_DIR" || { echo -e "${RED}Failed to navigate to $TEMP_DIR${NC}"; exit 1; }
echo -e "${GREEN}Temporary directory created and navigated to: $TEMP_DIR${NC}"

# Save the current state (in case we need to revert)
STATE_FILE="${TEMP_DIR}/state.txt"
echo -e "${YELLOW}Saving initial state...${NC}"
echo "Initial kernel version: $CURRENT_KERNEL" > "$STATE_FILE"
echo "Selected PREEMPT_RT kernel version: $KERNEL_VERSION" >> "$STATE_FILE"
echo "Kernel Base URL: $KERNEL_BASE_URL" >> "$STATE_FILE"
echo "Patch Base URL: $PATCH_BASE_URL" >> "$STATE_FILE"
echo "Temporary directory: $TEMP_DIR" >> "$STATE_FILE"
check_status
echo -e "${GREEN}Initial state saved successfully.${NC}"

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

# Define patterns for the files we need
KERNEL_PATTERN="linux-${KERNEL_VERSION}*.tar.xz*"
PATCH_PATTERN="patch-${PATCH_VERSION}*.patch.xz*"

# Download the kernel and patch files
download_and_verify_files "$KERNEL_BASE_URL" "$KERNEL_PATTERN"
download_and_verify_files "$PATCH_BASE_URL" "$PATCH_PATTERN"

# Decompress the downloaded files
echo -e "${YELLOW}Decompressing downloaded files...${NC}"
for file in *.xz; do
    if [ -f "${file%.xz}" ]; then
        echo -e "${YELLOW}File ${file%.xz} already decompressed, skipping...${NC}"
    else
        echo -e "${YELLOW}Decompressing $file...${NC}"
        xz -d "$file"
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
for sign_file in *.sign; do
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
# sudo apt-get build-dep linux
sudo apt-get install libncurses-dev libssl-dev flex bison

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

# Update the GRUB bootloader to include the new kernel
echo -e "${YELLOW}Updating GRUB bootloader...${NC}"
sudo update-grub
check_status
echo -e "${GREEN}GRUB updated successfully.${NC}"

# Display the final state for user satisfaction
echo -e "${YELLOW}Final state:${NC}"
cat "$STATE_FILE"

# Setup user privileges for real-time scheduling

echo -e "${YELLOW}Setting up user privileges for real-time scheduling...${NC}"

# Create the realtime group
sudo groupadd realtime
check_status
echo -e "${GREEN}Realtime group created successfully.${NC}"

# Add the current user to the realtime group
sudo usermod -aG realtime "$(whoami)"
check_status
echo -e "${GREEN}User added to the realtime group successfully.${NC}"

# Ensure /etc/security/limits.conf contains the necessary configuration
echo -e "${YELLOW}Configuring /etc/security/limits.conf for real-time scheduling...${NC}"
sudo bash -c 'cat >> /etc/security/limits.conf <<EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF'
check_status
echo -e "${GREEN}/etc/security/limits.conf configured successfully.${NC}"

# Setup GRUB to always boot the real-time kernel

echo -e "${YELLOW}Configuring GRUB to boot the real-time kernel by default...${NC}"

# Find the menu entry for the newly installed kernel
GRUB_ENTRY=$(awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg | grep -E "Advanced options for Ubuntu>Ubuntu, with Linux ${KERNEL_VERSION}-rt" | head -n 1)

if [ -z "$GRUB_ENTRY" ]; then
    echo -e "${RED}Error: Could not find the GRUB entry for the real-time kernel.${NC}"
    exit 1
fi

echo -e "${GREEN}Found GRUB entry: $GRUB_ENTRY${NC}"

# Set the GRUB default entry
sudo sed -i "s/^GRUB_DEFAULT=.*/GRUB_DEFAULT=\"$GRUB_ENTRY\"/" /etc/default/grub
check_status
echo -e "${GREEN}GRUB default entry set successfully.${NC}"

# Update GRUB
sudo update-grub
check_status
echo -e "${GREEN}GRUB updated successfully.${NC}"

# Final instructions to the user
echo -e "${YELLOW}Please note:${NC} You will need to log out and log back in (or reboot) for the changes to the realtime group to take effect. The system will automatically boot into the real-time kernel on the next restart."
