#!/bin/bash
cd "$(dirname "$(readlink -f "$0")")"
tmuxp load config/launch_magic_box.yaml