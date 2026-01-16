# Motion Planning Experiment

This project implements a new motion planning algorithm extension from the family of [Multi Level Motion Planner](https://ompl.kavrakilab.org/multiLevelPlanning.html) using [OMPL (Open Motion Planning Library)](https://ompl.kavrakilab.org/index.html). It includes a framework to run experiments with multiple algorithms on different scenes, collect performance data, and visualize the results.

This work is based on the [Multilevel motion planning: A fiber bundle formulation](https://journals.sagepub.com/doi/full/10.1177/02783649231209337) article, written by [Andreas Orthey](https://scholar.google.com/citations?user=bQKreEMAAAAJ), Sohaib Akbar, and [Prof. Marc Toussaint](https://scholar.google.com/citations?user=t2X4Mg8AAAAJ&hl=iw).

## Table of Contents

- [Description](#description)
- [How to use](#how-to-use)

## Description

The project provides a framework for testing and comparing multi level motion planning algorithms such as QRRT, QRRTStar, QMP, and QMPStar versus BiQRRT, on a familty group of 2D kinematic chain robots. 

It includes a Python script to automate the execution of these algorithms on different scenes with varying time limits and collect performance metrics, and provides several custom made input scenes.

## How to use

The project is written with a heavily dependency on OMPL. For verifying versions alignment, the best way for using this repo is by building a docker image.

### Prerequisites

- Docker

### Building the Docker Image
Run the following from source.

1. Clone the repository:
    ```sh
    git clone https://github.com/tomsabala/MLMP.git
    cd MLMP
    ```

2. Build the Docker image and run container (this might take up to 8-10 minutes):
    ```sh
    ./build_image.sh
    ./run_container.sh
    ./shell_container.sh
    ```
### Compile project
Make sure you are located in the mlmp directory:

    cd ~/../home/mlmp/mlmp
    ./compile

** If this from some reason takes more than a couple of minutes, please kill the process and retry.

### Usage

The program accepts the following command-line arguments:

    --input-file: Specifies the input file for the scene (mandatory).
    --time-limit: Specifies the time limit for the algorithm (default: 1.0).
    --verbose: Enables verbose output (default: false).
    --algo: Specifies the algorithm to use (default: BiQRRT, allowed values: RRTStar, PRMStar, QRRT, BiQRRT, QMP, QMPStar).

* Python:
    
    From the `mlmp/tools` directory, run:
    
    `python3 main.py --input-file scene4.json --time-limit 5 --verbose 1 --algo BiQRRT`

* C++:
    
    From the `mlmp` directory, run:

    `./mlmp --input-file ./inputs/experiment/scene4.json --time-limit 5 --verbose 1 --algo BiQRRT`


** note the difference in the path requirement of the scene file.

### Experiment

From `mlmp/tools` directory, you can run:

    python3 experiment.py

Note this is might take some time to evaluate all results.

[git link](https://github.com/tomsabala/MLMP)

Enjoy :) 
