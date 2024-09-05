: '
MIT License

Author: Samarth
Contact: psxs2@nottingham.ac.uk

This code is prepared for the dissertation project at the University of Nottingham.
The University of Nottingham has governance over this code.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'
: '
This script, `kernel_rt.sh`, is designed to set up a real-time kernel on a Linux system. It includes 
functions for printing messages in different colors, updating the system, installing necessary packages, 
and configuring the real-time kernel.

Detailed Explanation:
1. Define Colors for Output: The script defines color codes for red, green, and yellow to format the 
   output messages. This helps in distinguishing different types of messages.
   - Color Definitions:
     ```sh
     RED=\'\033[0;31m'
     GREEN='\033[0;32m'
     YELLOW='\033[1;33m'
     ```

2. Print Informational Messages: The script includes a function to print messages in yellow color to 
   indicate informational messages to the user.
   - Function:
     ```sh
     print_info() {
         echo -e "${YELLOW}[INFO] $1${NC}"
     }
     ```

3. Print Error Messages: The script includes a function to print messages in red color to indicate error 
   messages to the user.
   - Function:
     ```sh
     print_error() {
         echo -e "${RED}[ERROR] $1${NC}"
     }
     ```

4. Print Success Messages: The script includes a function to print messages in green color to indicate 
   successful execution messages to the user.
   - Function:
     ```sh
     print_success() {
         echo -e "${GREEN}[SUCCESS] $1${NC}"
     }
     ```

5. Update System: The script updates the system packages to ensure that the latest versions are installed.
   - Command:
     ```sh
     sudo apt update && sudo apt upgrade -y
     ```

6. Install Real-Time Kernel: The script installs the real-time kernel packages required for real-time 
   operations.
   - Command:
     ```sh
     sudo apt install linux-image-rt-amd64 linux-headers-rt-amd64 -y
     ```

7. Reboot System: The script prompts the user to reboot the system to apply the changes.
   - Command:
     ```sh
     sudo reboot
     ```

How to Invoke the Script:
To invoke the script, you can run it from the command line. For example:
```sh
./kernel_rt.sh
```
'
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
    echo -e "${YELLOW}Kernel base URL example: https://www.kernel.org/pub/linux/kernel/v6.x/${NC}"
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
    
setup_realtime_privileges() {
    # Create a group for real-time users
    sudo groupadd realtime

    # Add the current user to the realtime group
    sudo usermod -aG realtime $(whoami)

    # Update /etc/security/limits.conf with real-time scheduling settings
    sudo bash -c 'cat <<EOF >> /etc/security/limits.conf
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF'

    echo "Real-time scheduling privileges have been set up. Please log out and log back in for the changes to take effect."
}

setup_grub_for_rt_kernel() {
    # List all available kernels
    echo "Available kernels:"
    awk -F\' '/menuentry |submenu / {print $1 $2}' /boot/grub/grub.cfg

    # Prompt the user to enter the submenu and entry name
    read -p "Enter the submenu name (e.g., 'Advanced options for Ubuntu'): " submenu_name
    read -p "Enter the entry name (e.g., 'Ubuntu, with Linux 4.14.139-rt66'): " entry_name

    # Generate the GRUB_DEFAULT string
    grub_default="${submenu_name}>${entry_name}"

    # Update the GRUB configuration
    sudo sed -i 's/^GRUB_DEFAULT=.*/GRUB_DEFAULT="'"$grub_default"'"/' /etc/default/grub

    # Update GRUB menu entries
    sudo update-grub

    echo "GRUB has been configured to boot the real-time kernel by default. Please reboot your system for the changes to take effect."
}

reboot_system() {
    read -p "Do you want to reboot your system now? [y/n]: " reboot_choice
    if [ "$reboot_choice" = "y" ]; then
        sudo reboot
    else
        echo "Please remember to reboot your system for the changes to take effect."
    fi
}   

# Main menu loop
while true; do
    echo "-----------------------------------------------------------------------------------"
    echo "Linux Real-Time Kernel Setup Script"
    echo "-----------------------------------------------------------------------------------"
    echo "1. Install required packages"
    echo "2. Download kernel and patch files"
    echo "3. Decompress downloaded files"
    echo "4. Verify downloaded files"
    echo "5. Extract and patch kernel"
    echo "6. Configure kernel"
    echo "7. Compile kernel"
    echo "8. Install compiled kernel"
    echo "9. Setup real-time privileges"
    echo "10. Setup GRUB for RT kernel"
    echo "11. Reboot system"
    echo "12. Exit"
    echo "-----------------------------------------------------------------------------------"
    read -p "Enter your choice [1-12]: " choice
    echo "-----------------------------------------------------------------------------------"
    case $choice in
        1)
            install_packages
            ;;
        2)
            download_kernel_and_patch
            ;;
        3)
            decompress_files
            ;;
        4)
            verify_downloaded_files
            ;;
        5)
            extract_and_patch_kernel
            ;;
        6)
            configure_kernel
            ;;
        7)
            compile_kernel
            ;;
        8)
            install_compiled_kernel
            ;;
        9)
            setup_realtime_privileges
            ;;
        10)
            setup_grub_for_rt_kernel
            ;;
        11)
            reboot_system
            ;;
        12)
            break
            ;;
        *)
            echo "Invalid choice. Please enter a valid option."
            ;;
    esac
done
