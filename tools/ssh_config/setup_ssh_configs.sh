#!/bin/bash

# This script sets up SSH configuration for tunnel forwarding so you can
# build and deploy code without having the code locally.
# It creates or appends to the local SSH config file (~/.ssh/config)
# and connects to the build server to create or append the remote SSH config.
# It prompts the user for their SSH username, the name of the local identity file,
# and the name of the remote identity file.

# Instructions:
# 1. Run this script locally on your machine.
# 2. Make the script executable: chmod +x setup_ssh_configs.sh
# 3. Run the script: ./setup_ssh_configs.sh

# Note: Ensure that the local and remote SSH config template files are 
# in the same directory as this script.

# TODO: Use jinja for better templating
# TODO: Use python to also support windows devices.

# Get the script's directory
script_dir=$(dirname "$(realpath "$0")")

# Prompt for user input
read -p "Enter your username for SSH connection: " username
read -p "Enter the name of the local identity file (e.g., id_971_ed25519): " local_identity_file
read -p "Enter the name of the remote identity file (e.g., id_971_ed25519): " remote_identity_file

local_ssh_config_template="$script_dir/local_config_template.txt"
remote_ssh_config_template="$script_dir/remote_config_template.txt"


local_ssh_config_content=$(cat "$local_ssh_config_template" | sed -e "s/{{username}}/$username/g" -e "s/{{identity_file}}/$local_identity_file/g")
local_ssh_config="$HOME/.ssh/config"

# Check if the local SSH config file already exists
if [ -f "$local_ssh_config" ]; then
    # Append
    echo "$local_ssh_config_content" >> "$local_ssh_config"
else
    echo "$local_ssh_config_content" > "$local_ssh_config"
    chmod 600 "$local_ssh_config"
fi

remote_ssh_config_content=$(cat "$remote_ssh_config_template" | sed -e "s/{{username}}/$username/g" -e "s/{{identity_file}}/$remote_identity_file/g")
remote_ssh_config="~/.ssh/config"

ssh "frc971" "echo '$remote_ssh_config_content' >> $remote_ssh_config"

echo "SSH configuration for tunnel forwarding has been set up locally and on the build server."
