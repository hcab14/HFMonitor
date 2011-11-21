#!/bin/bash
# $Id$
at now <<EOF
cd /home/chm/C++/HFMonitor
sudo ./hfmon_server config_server.xml > log_server.txt 2>&1
EOF
