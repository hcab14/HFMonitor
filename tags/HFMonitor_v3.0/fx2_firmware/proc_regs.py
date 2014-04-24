#!/usr/bin/env python2.7

import sys

lines = [line.strip() for line in open(sys.argv[1])]

print "[XRAM]"
for i in range(len(lines)):
    ls=lines[i].split()
    if (len(ls)>1 and ls[0] == "__xdata"):
        v=ls[5].replace(";", "").split("[")
        if len(v) == 1:
            print "%s:%s" % (ls[2], v[0])
        else:
            n=int(v[1].replace("]", ""))
            for i in range(n):
                addr=int(ls[2], 16)+i
                print "0x%04X:%s[%d]" % (addr, v[0], i)

print "[IRAM]"
for i in range(len(lines)):
    ls=lines[i].split()
    if (len(ls)>1 and ls[0] == "__sfr"):
        v=ls[3].replace(";", "");
        print "%s:%s" % (ls[2], v)


