# UWB Wrapper - README

**Last Updated:** 14 Feb 2025

## Overview

This package (specifically `UWB_ReadUDP.py`) is meant to be run with another .exe (compiled from `nlink_unpack-master/main_udp.c`) running it the background, which sends the position of ALL nodes (aka tags) in a single message via UDP Port 5000.

The functions in `UWB_ReadUDP.py` are designed to be imported into ANY code, and allows the user to obtain the UWB positions of any tag ID at any time.

- `UWB_ReadUDP2.py` is adapted for use with a USB WiFi antenna, to read data from a secondar WiFi network

The remaining codes are just there for testing purposes (e.g. UWB_SendUDP.py simulates drones flying in circles, and sends their coordinates via UDP for UWB_ReadUDP.py to pick up). 