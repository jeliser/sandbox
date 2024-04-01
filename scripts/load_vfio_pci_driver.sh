#!/bin/bash

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
  echo "This script must be run as root" >&2
  exit 1
fi

# Replace these variables with your actual PCI ID and vendor:device ID
PCI_ID="0000:01:00.0"
VENDOR_DEVICE_ID="10de 2684"

if [ $( lspci -k -s $PCI_ID | grep "driver in use" | grep vfio-pci | wc -l ) -eq 1 ]; then
  echo "GPU already bound to vfio-pci."
  exit 0
fi

# Unload NVIDIA drivers
echo "Unloading NVIDIA drivers..."
modprobe -r nvidia_drm nvidia_modeset nvidia_uvm nvidia
if [ $? -ne 0 ]; then
  echo "Failed to unload NVIDIA drivers."
  exit 1
fi

# Load vfio-pci
echo "Loading vfio-pci module..."
modprobe vfio-pci
if [ $? -ne 0 ]; then
  echo "Failed to load vfio-pci module."
  exit 1
fi

# Bind the GPU to vfio-pci
echo "$VENDOR_DEVICE_ID" > /sys/bus/pci/drivers/vfio-pci/new_id
echo "$PCI_ID" > /sys/bus/pci/devices/$PCI_ID/driver/unbind
echo "$PCI_ID" > /sys/bus/pci/drivers/vfio-pci/bind

if [ $? -ne 0 ]; then
  echo "Failed to bind the GPU to vfio-pci."
  exit 1
else
  echo "GPU successfully bound to vfio-pci."
fi

