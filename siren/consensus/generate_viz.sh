#!/usr/bin/env bash

# Assuming sismic is in the current python scope be that the global site-packages or a virtual environment
# Takes 1 optional parameter of the name of the PlantUML file (excluding file extension). If no name is specified, the name defaults to consensus.txt.

sismic-plantuml consensus.yaml > ${1:-consensus}.txt