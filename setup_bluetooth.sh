#!/bin/bash
# Bluetooth MAVLink Setup for Mission Planner

echo "Setting up Bluetooth Serial for MAVLink..."

# Enable Bluetooth
sudo systemctl enable bluetooth
sudo systemctl start bluetooth

# Make Bluetooth discoverable
sudo bluetoothctl <<EOF
power on
discoverable on
pairable on
agent on
default-agent
EOF

echo " Bluetooth is now discoverable as: $(hostname)"
echo " Connect from Mission Planner using Bluetooth"
echo ""
echo "On Windows Mission Planner:"
echo "  1. Pair with this Raspberry Pi via Bluetooth"
echo "  2. Note the COM port (e.g., COM5)"
echo "  3. Connect to that COM port at 57600 baud"
