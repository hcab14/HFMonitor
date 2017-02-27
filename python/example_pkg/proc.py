#!/usr/binenv python

a = 0
def process_data_per_interval(d):
    """this function is called for each epoch"""
    global a;
    a += 1;
    print a;
    print d;
    for k, v in d.iteritems():
        for x in v:
            print k, x[0], x[1]
