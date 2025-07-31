#!/bin/bash
set -e

# Find setup.cfg in the current directory
SETUP_CFG="./setup.cfg"
REQ_FILE="./requirements.txt"

if [[ ! -f "$SETUP_CFG" ]]; then
    echo "❌ setup.cfg not found in current directory."
    exit 1
fi

# Extract the Python executable path from setup.cfg
PYTHON_EXEC=$(awk '
    /^\[build_scripts\]/ { in_section=1; next }
    /^\[/ { in_section=0 }
    in_section && /^executable *=/ {
        sub(/^executable *= */, "", $0)
        print $0
        exit
    }
' "$SETUP_CFG")

if [[ -z "$PYTHON_EXEC" || ! -x "$PYTHON_EXEC" ]]; then
    echo "❌ Invalid or missing Python executable in setup.cfg: '$PYTHON_EXEC'"
    exit 1
fi

if [[ ! -f "$REQ_FILE" ]]; then
    echo "⚠️ requirements.txt not found. Nothing to install."
    exit 0
fi

# Install dependencies using the specified Python executable
echo "📦 Installing dependencies with: $PYTHON_EXEC"
"$PYTHON_EXEC" -m pip install --upgrade pip
"$PYTHON_EXEC" -m pip uninstall cv_bridge # just in case so it can pull from sudo apt install ros-humble-cv-bridge
"$PYTHON_EXEC" -m pip install deepface
"$PYTHON_EXEC" -m pip install tf-keras
"$PYTHON_EXEC" -m pip install --no-cache-dir --upgrade --ignore-installed opencv-python
"$PYTHON_EXEC" -m pip install numpy==1.26.4

echo "✅ Installation complete."