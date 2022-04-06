#!/usr/bin/python3

import roslib.message
import sys

sys.stdout.write(roslib.message.get_service_class(sys.argv[1])._md5sum)
