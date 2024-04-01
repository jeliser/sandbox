#!/bin/bash

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
  echo "This script must be run as root" >&2
  exit 1
fi

# Replace these variables with your actual PCI ID and vendor:device ID
PCI_ID="0000:01:00.0"
VENDOR_DEVICE_ID="10de 2684"

if [ $( lspci -k -s $PCI_ID | grep "driver in use" | grep nvidia | wc -l ) -eq 1 ]; then
  echo "GPU already bound to nvidia."
  exit 0
fi

# Unbind the GPU from vfio-pci
echo "Unbinding the GPU from vfio-pci..."
echo "$PCI_ID" > /sys/bus/pci/drivers/vfio-pci/unbind
echo "$VENDOR_DEVICE_ID" > /sys/bus/pci/drivers/vfio-pci/remove_id

# Check if the GPU was successfully unbound
if [ ! -e "/sys/bus/pci/drivers/vfio-pci/$PCI_ID" ]; then
  echo "GPU successfully unbound from vfio-pci."
else
  echo "Failed to unbind the GPU from vfio-pci."
  exit 1
fi

# Reload NVIDIA drivers
echo "Reloading NVIDIA drivers..."
modprobe nvidia nvidia_modeset nvidia_uvm nvidia_drm

if [ $? -ne 0 ]; then
  echo "Failed to reload NVIDIA drivers."
  exit 1
else
  echo "NVIDIA drivers successfully reloaded."
fi

