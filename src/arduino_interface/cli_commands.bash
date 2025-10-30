#!/usr/bin/env bash
arduino-cli compile --fqbn arduino:avr:leonardo arduino_interface.ino
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:leonardo arduino_interface.ino
