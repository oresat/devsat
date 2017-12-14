#!/usr/bin/env python

# Example
# import Utils as u
#  s = "opened port {}\n".format(self.serial_port,)
#  u.info(s)


import sys

if sys.version_info < (3, 0):
    def info(s):
        print "\r\n*** INFO: ",s
        return

    def warning(s):
        print "\r\n*** WARNING: ",s
        return

    def error(s):
        print "\r\n*** ERROR: ",s
        return
else:
    def info(s):
        print("\r\n*** INFO: ",s)
        return

    def warning(s):
        print("\r\n*** WARNING: ",s)
        return

    def error(s):
        print("\r\n*** ERROR: ",s)
        return

