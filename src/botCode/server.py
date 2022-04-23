#!/usr/bin/env python3

import robomodules
import os
from messages import MsgType

ADDRESS = os.environ.get("LOCAL_ADDRESS","localhost")
# ADDRESS = os.environ.get("LOCAL_ADDRESS","172.20.10.3")
PORT = os.environ.get("LOCAL_PORT", 11295)

def main():
    server = robomodules.Server(ADDRESS, PORT, MsgType)
    server.run()

if __name__ == "__main__":
    main()
