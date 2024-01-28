#! /usr/bin/env python3
import sys
import subprocess
import time

time.sleep(5)

subprocess.run(["runcoppelia", sys.argv[1], "-s"])