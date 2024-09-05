#!/bin/bash

# Define color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check the status of the last command
check_status() {
    local cmd="$1"
    local func="$2"
    if [ $? -ne 0 ]; then
        echo -e "${RED}Error occurred in function ${func} while executing: ${cmd}. Exiting.${NC}"
        exit 1
    else
        echo -e "${GREEN}Command executed successfully in function ${func}: ${cmd}.${NC}"
    fi
}

# Function to set up user privileges for real-time scheduling
setup_realtime_privileges() {
    echo -e "${YELLOW}Setting up user privileges for real-time scheduling...${NC}"

    # Check if the user is already in the realtime group
    if groups "$(whoami)" | grep -q '\brealtime\b'; then
        echo -e "${YELLOW}User is already in the realtime group.${NC}"
        # Ask if the user wants to delete the realtime group and re-add the user
        read -p "Do you want to delete the realtime group and re-add the user? [y/n]: " choice
        case $choice in
            [Yy]*)
                # Delete the realtime group
                cmd="sudo groupdel realtime"
                $cmd
                check_status "$cmd" "setup_realtime_privileges"

                # Make a new realtime group
                cmd="sudo groupadd realtime"
                $cmd
                check_status "$cmd" "setup_realtime_privileges"
                ;;
            *)
                echo -e "${YELLOW}Skipping deletion of the realtime group.${NC}"
                ;;
        esac
    else
        # Create the realtime group
        cmd="sudo groupadd realtime"
        $cmd
        check_status "$cmd" "setup_realtime_privileges"
    fi

    # Add the current user to the realtime group
    cmd="sudo usermod -aG realtime $(whoami)"
    $cmd
    check_status "$cmd" "setup_realtime_privileges"

    # Ensure /etc/security/limits.conf contains the necessary configuration
    echo -e "${YELLOW}Configuring /etc/security/limits.conf for real-time scheduling...${NC}"
    cmd="sudo bash -c 'cat >> /etc/security/limits.conf <<EOF
@realtime soft rtprio 99
@realtime soft priority 99
@realtime soft memlock 102400
@realtime hard rtprio 99
@realtime hard priority 99
@realtime hard memlock 102400
EOF'"
    eval "$cmd"
    check_status "$cmd" "setup_realtime_privileges"
}


# Function to download and verify kernel and patch files
download_and_verify_files() {
    local base_url=$1
    local pattern=$2

    echo -e "${YELLOW}Searching for files matching pattern: $pattern${NC}"

    # List the files and download the ones that match the pattern
    cmd="wget -r -l1 -nd -np -A \"$pattern\" -e robots=off --accept-regex \"$pattern\" \"$base_url\" -P \"$TEMP_DIR\""
    eval "$cmd"
    check_status "$cmd" "download_and_verify_files"
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
            cmd="unxz \"$file\""
            eval "$cmd"
            check_status "$cmd" "download_kernel_and_patch"
        fi
    done

    # Verification of the downloaded files
    echo -e "${YELLOW}Verifying downloaded files...${NC}"

    # Import public keys for verification
    echo -e "${YELLOW}Importing public keys from kernel developers...${NC}"
    cmd="gpg2 --locate-keys torvalds@kernel.org gregkh@kernel.org"
    eval "$cmd"
    check_status "$cmd" "download_kernel_and_patch"

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
            cmd="gpg2 --keyserver \"$server\" --search-keys \"$key\""
            eval "$cmd"
            if [ $? -eq 0 ]; then
                check_status "$cmd" "import_patch_author_key"
                return 0
            else
                echo -e "${RED}Failed to retrieve key from $server${NC}"
            fi
        done
        
        echo -e "${RED}All keyserver attempts failed. Please check your network or try again later.${NC}"
        exit 1
    }

    echo -e "${YELLOW}Searching and importing the key for the patch author...${NC}"
    import_patch_author_key

    # Verify the kernel source signature
    for sign_file in "$TEMP_DIR"/*.sign; do
        if [[ "$sign_file" == *"linux-"* ]]; then
            echo -e "${YELLOW}Verifying the kernel source signature for $sign_file...${NC}"
            cmd="gpg2 --verify \"$sign_file\""
            eval "$cmd"
            check_status "$cmd" "download_kernel_and_patch"
        elif [[ "$sign_file" == *"patch-"* ]]; then
            echo -e "${YELLOW}Verifying the patch signature for $sign_file...${NC}"
            cmd="gpg2 --verify \"$sign_file\""
            eval "$cmd"
            check_status "$cmd" "download_kernel_and_patch"
        fi
    done
}

# Function to compile the kernel
compile_kernel() {
    # Display the current state for user satisfaction
    echo -e "${YELLOW}Current state:${NC}"
    cmd="cat \"$STATE_FILE\""
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Compilation and Patch Application

    # Extract the tar archive
    echo -e "${YELLOW}Extracting the kernel source tar archive...${NC}"
    cmd="tar xf \"linux-$KERNEL_VERSION.tar\""
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Navigate into the extracted kernel source directory
    cmd="cd \"linux-$KERNEL_VERSION\""
    eval "$cmd" || { echo -e "${RED}Failed to enter kernel source directory${NC}"; exit 1; }

    # Apply the real-time patch
    echo -e "${YELLOW}Applying the real-time patch...${NC}"
    cmd="xzcat \"../patch-$PATCH_VERSION.patch.xz\" | patch -p1"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Configure the kernel
    echo -e "${YELLOW}Configuring the kernel...${NC}"
    cmd="make oldconfig"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Build the kernel
    echo -e "${YELLOW}Building the kernel... This may take some time.${NC}"

    # Clean up previous builds if any
    cmd="make clean"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Installing missing dependencies
    cmd="sudo apt-get install -y libncurses-dev libssl-dev flex bison"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Compile the kernel. Set V=1 for verbose output in make command
    cmd="make -j \"$(getconf _NPROCESSORS_ONLN)\" deb-pkg"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Compile the modules
    echo -e "${YELLOW}Compiling the kernel modules...${NC}"
    cmd="make -j \"$(getconf _NPROCESSORS_ONLN)\" modules"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Install the modules
    echo -e "${YELLOW}Installing the kernel modules...${NC}"
    cmd="sudo make modules_install"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"

    # Install the kernel
    echo -e "${YELLOW}Installing the kernel...${NC}"
    cmd="sudo make install"
    eval "$cmd"
    check_status "$cmd" "compile_kernel"
}

# Function to update the GRUB bootloader
# Function to update the GRUB bootloader
update_grub_bootloader() {
    echo -e "${YELLOW}Updating GRUB bootloader...${NC}"
    cmd="sudo update-grub"
    eval "$cmd"
    check_status "$cmd" "update_grub_bootloader"

    # Display the final state for user satisfaction
    echo -e "${YELLOW}Final state:${NC}"
    cmd="cat \"$STATE_FILE\""
    eval "$cmd"
    check_status "$cmd" "update_grub_bootloader"

    # Setup GRUB to always boot the real-time kernel
    echo -e "${YELLOW}Configuring GRUB to boot the real-time kernel by default...${NC}"

    # Find the menu entry for the newly installed kernel
    cmd="awk -F\' '/menuentry |submenu / {print \$1 \$2}' /boot/grub/grub.cfg | grep -E \"Advanced options for Ubuntu>Ubuntu, with Linux ${KERNEL_VERSION}-rt\" | head -n 1"
    GRUB_ENTRY=$(eval "$cmd")
    check_status "$cmd" "update_grub_bootloader"

    if [ -z "$GRUB_ENTRY" ]; then
        echo -e "${RED}Error: Could not find the GRUB entry for the real-time kernel.${NC}"
        exit 1
    fi

    echo -e "${GREEN}Found GRUB entry: $GRUB_ENTRY${NC}"

    # Set the GRUB default entry
    cmd="sudo sed -i \"s/^GRUB_DEFAULT=.*/GRUB_DEFAULT=\\\"$GRUB_ENTRY\\\"/\" /etc/default/grub"
    eval "$cmd"
    check_status "$cmd" "update_grub_bootloader"

    # Update GRUB
    cmd="sudo update-grub"
    eval "$cmd"
    check_status "$cmd" "update_grub_bootloader"

    # Final instructions to the user
    echo -e "${YELLOW}Please note:${NC} You will need to log out and log back in (or reboot) for the changes to the realtime group to take effect. The system will automatically boot into the real-time kernel on the next restart."
}

# Function to display the menu
display_menu() {
    echo -e "${YELLOW}Kernel Real-Time Setup Menu${NC}"
    echo "1. Setup user privileges for real-time scheduling"
    echo "2. Download and verify kernel and patch files"
    echo "3. Compile the kernel"
    echo "4. Update GRUB bootloader"
    echo "5. Exit"
}

# Function to handle user input
handle_user_input() {
    local choice
    read -p "Enter your choice [1-5]: " choice
    case $choice in
        1)
            setup_realtime_privileges
            ;;
        2)
            download_kernel_and_patch
            ;;
        3)
            compile_kernel
            ;;
        4)
            update_grub_bootloader
            ;;
        5)
            echo -e "${GREEN}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid choice, please try again.${NC}"
            ;;
    esac
}

# Main loop to display the menu and handle user input
main_loop() {
    while true; do
        display_menu
        handle_user_input
    done
}

# Call the main loop
main_loop