#!/bin/bash

# Create .zsh_history file if it doesn't exist
if [ ! -f ".tmp/.zsh_history" ]; then
    mkdir -p ".tmp"
    touch ".tmp/.zsh_history"
fi

xhost +local:docker