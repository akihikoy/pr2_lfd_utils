#! /usr/bin/env python

class JoyKind:
    Invalid = -1
    PS3 = 1 # Playstation 3 controller
    RS = 2  # Radioshack controller
    default = PS3

def strToJoyKind(str_joy_kind):
    if   str_joy_kind=='default': return JoyKind.default
    elif str_joy_kind=='PS3': return JoyKind.PS3
    elif str_joy_kind=='RS':  return JoyKind.RS
    else:
        print "Invalid joystick kind: "+str(str_joy_kind)
        return JoyKind.Invalid
