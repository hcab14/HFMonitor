#!/bin/bash
# $Id$
at now <<EOF
cd /media/dfa46bc2-9377-47b4-8d25-3cc2039fc2ff/.rw/home/linutop
~/HFMonitor/hfmon_clientTCPFFT 127.0.0.1 18000 ~/HFMonitor/config_FFTProcessor.xml > ~/HFMonitor/log_client.txt 2>&1
EOF
