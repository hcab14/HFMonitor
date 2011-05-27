#!/bin/bash
# $Id$
at now <<EOF
cd /home/linutop/HFMonitor
./hfmon_clientTCPFFT 127.0.0.1 18000 config_FFTProcessor.xml > log_client.txt 2>&1
EOF
