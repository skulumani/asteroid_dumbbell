#!/usr/bin/env python3

from myexceptions import someFunction

try:
    someFunction()
except UserWarning as ex:
    print("Exception {} caught: {}".format(type(ex), ex))
