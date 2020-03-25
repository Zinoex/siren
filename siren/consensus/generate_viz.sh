#!/usr/bin/env bash

# Assuming sismic is in the current python scope be that the global site-packages or a virtual environment

# Usage: ./generate_viz.sh [-p] [file_name]
# - p: run PlantUML on the generated file
# - file_name: name of the PlantUML file (excluding file extension). Default: consensus.

while getopts "p" opt; do
  case $opt in
    p ) plantuml=true;;
    * ) echo "Invalid option"; exit 1;;
  esac

  shift $((OPTIND -1))
done

file_name=${1:-consensus}

sismic-plantuml consensus.yaml > "$file_name.txt"

if [ $plantuml ]; then
  plantuml "$file_name.txt";
fi