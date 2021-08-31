# DARP: Divide Areas Algorithm for Optimal Multi-Robot Coverage Path Planning

## Motivation

This project deals with the path planning problem of a team of mobile robots, in order to cover an area of interest, with prior-defined obstacles.

DARP algorithm divides the terrain into a number of equal areas each corresponding to a specific robot, so as to guarantee complete coverage, non-backtracking solution, minimum coverage path, while at the same time does not need any preparatory stage.

## Requirements

This project was created using:

* Python version >= 3.6.14
* OpenCV version >= 4.5.2.54
* Pygame version >= 2.0.1
* Scipy version >= 1.7.1

## Installation

```
git clone https://github.com/alice-st/DARP-Python.git
cd DARP-Python
./Dependencies.sh DARP
source DARP/bin/activate
python3 darpinPoly.py
```
