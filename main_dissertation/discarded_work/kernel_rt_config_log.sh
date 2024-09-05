# ---------------------------- Main script ----------------------------

# Building a real-time Linux kernel

# We create a directory in our home dir with
mkdir ~/kernel
# and switch into it with
cd ~/kernel

# We can go with a browser to https://mirrors.edge.kernel.org/pub/linux/kernel/v5.x/ and see if the version is there.
# You can download it from the site and move it manually from /Downloads to the /kernel folder, or download it using wget by right clicking the link using "copy link location".
wget https://mirrors.edge.kernel.org/pub/linux/kernel/v6.x/linux-6.6.44.tar.gz
# unpack it with
tar -xvf linux-6.6.44.tar.gz

# Downloading the rt_preempt patch matching the Kernel version
# We can go to https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.6/ to find the rt_preempt patch for our kernel version.
# You can download it manually from the website and move it to the /kernel folder, or download it using wget by right-clicking the link and selecting "copy link location".
wget https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.6/patch-6.6.44-rt39.patch.gz

# Unpack the patch with
gunzip patch-6.6.44-rt39.patch.gz

# We switch into the kernel directory with
cd linux-6.6.44

# We apply the patch with
patch -p1 < ../patch-6.6.44-rt39.patch

# We simply want to use the config of our Ubuntu installation, so we get the Ubuntu config with
cp /boot/config-$(uname -r) .config

# Open Software & Updates. in the Ubuntu Software menu tick the ‘Source code’ box

#We need some tools to build kernel, install them with
sudo apt-get build-dep linux
sudo apt-get install -y build-essential libncurses-dev bison flex libssl-dev libelf-dev

# To enable all Ubuntu configurations, we simply use
yes '' | make oldconfig

# Then we need to enable rt_preempt in the kernel. We call
make menuconfig

# and set the following

# # Enable CONFIG_PREEMPT_RT
#  -> General Setup
#   -> Preemption Model (Fully Preemptible Kernel (Real-Time))
#    (X) Fully Preemptible Kernel (Real-Time)

# # Enable CONFIG_HIGH_RES_TIMERS
#  -> General setup
#   -> Timers subsystem
#    [*] High Resolution Timer Support

# # Enable CONFIG_NO_HZ_FULL
#  -> General setup
#   -> Timers subsystem
#    -> Timer tick handling (Full dynticks system (tickless))
#     (X) Full dynticks system (tickless)

# # Set CONFIG_HZ_1000 (note: this is no longer in the General Setup menu, go back twice)
#  -> Processor type and features
#   -> Timer frequency (1000 HZ)
#    (X) 1000 HZ

# # Set CPU_FREQ_DEFAULT_GOV_PERFORMANCE [=y]
#  ->  Power management and ACPI options
#   -> CPU Frequency scaling
#    -> CPU Frequency scaling (CPU_FREQ [=y])
#     -> Default CPUFreq governor (<choice> [=y])
#      (X) performance

# Save and exit menuconfig. 
# Now we’re going to build the kernel which will take quite some time. (10-30min on a modern cpu)

# Initialize a Git repository ( if you face error: creating source package requires git repository)
git init
git add .
git commit -m "Initial commit of kernel source"

# ---------------------------- Note ----------------------------
# READ THIS BEFORE YOU BUILD THE KERNEL
# Typically, compiling and installing a custom kernel does not require changes to your BIOS settings.
# However, there are a few BIOS settings that can affect the performance and behavior of your system,
# especially when dealing with real-time kernels:

# 1. Virtualization: Ensure that virtualization support (VT-x for Intel or AMD-V for AMD) is enabled
#    if you plan to use virtual machines or certain types of debugging tools.

# 2. Secure Boot: If Secure Boot is enabled, it might prevent the installation of unsigned kernels.
#    You may need to disable Secure Boot or sign your kernel.

# 3. Hyper-Threading: For real-time performance, some users disable Hyper-Threading to reduce latency.

# 4. Power Management: Ensure that power management settings are optimized for performance rather than
#    power saving. This can include disabling C-states and other power-saving features that might introduce latency.

# 5. IOMMU: Ensure that IOMMU (Input-Output Memory Management Unit) is enabled if you need it for device
#    passthrough in virtual machines.

# Before making any changes to your BIOS, ensure you understand the implications and have a way to revert
# the changes if needed. BIOS settings can usually be accessed by pressing a key (such as F2, F10, DEL, or ESC)
# during the boot process.

# If you continue to experience issues with compiling or installing the kernel, it is more likely related to
# software configuration or dependencies rather than BIOS settings. Focus on resolving any software-related errors first.

# ---------------------------- Note ----------------------------
# Compile the kernel and create Debian packages
make -j$(nproc) deb-pkg

# After the build is finished check the debian packages
ls ../*deb
../linux-headers-5.4.78-rt41_5.4.78-rt44-1_amd64.deb  ../linux-image-5.4.78-rt44-dbg_5.4.78-rt44-1_amd64.deb
../linux-image-5.4.78-rt41_5.4.78-rt44-1_amd64.deb    ../linux-libc-dev_5.4.78-rt44-1_amd64.deb


# ---------------------------- Main script ----------------------------

# ---------------------------- Code being derived from ( not to be a part of the main script ) ---------------------------- 


# ---------------------------- Code being derived from ( not to be a part of the main script ) ----------------------------