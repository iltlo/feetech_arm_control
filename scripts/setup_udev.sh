#!/bin/bash

# Define the udev rules file
UDEV_RULES_FILE="99-feetech-arms.rules"
UDEV_RULES_PATH="/etc/udev/rules.d/$UDEV_RULES_FILE"

echo "🔧 Setting up Feetech Arm udev rules..."

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "❌ Please run this script as root (use sudo)."
    exit 1
fi

# Copy the udev rules file
if [ -f "$UDEV_RULES_FILE" ]; then
    echo "📂 Copying $UDEV_RULES_FILE to $UDEV_RULES_PATH..."
    cp "$UDEV_RULES_FILE" "$UDEV_RULES_PATH"
else
    echo "❌ Error: $UDEV_RULES_FILE not found in the current directory!"
    exit 1
fi

# Set correct permissions
chmod 644 "$UDEV_RULES_PATH"

# Reload udev rules
echo "🔄 Reloading udev rules..."
udevadm control --reload-rules && udevadm trigger

echo "✅ Udev rules installed successfully!"
echo "🔌 Please unplug and replug your Feetech Arm(s) for the changes to take effect."