# Idef_OS_X - A python suite to allow control of a quadruped (specifically Idef'X).

This is a small, light weight and very extandable suite of python tools to let you (but mostly me for now) control a quadruped robot (Idef'X for now). It was build to run on a raspberry pi.

## Key features

- Ability to initialize both a "zero" position as well as a "resting" position for the motors (gyems rmd x8 for now).
- HTTP socket to receive order from anywhere on the net to let you connect any type of controller (local navigation stack, remote controller, cmd-line based interface, ...)
- Ability to log any quantity in hdf5 format interactively while not dropping anything in the event of an error or emergency stop.

