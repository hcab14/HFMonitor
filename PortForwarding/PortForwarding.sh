#!/bin/sh
# $Id$
#
flock --timeout 3 $0 ssh -F $HOME/.ssh/PortForwardingGIFDS -g -N cpcfcmd >/dev/null 2>/dev/null

