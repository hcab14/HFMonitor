#!/bin/env python

import datetime
import cPickle as pickle

#
# global variables
#
data = {}

#
# restore state
#
try:
    with open("save.p", "rb") as f:
        data = pickle.load(f)
except EnvironmentError, err:
    print err

    
def fill_data(d):
    """ update data """
    global data
    # find latest time
    t  = max([x[0] for k in d.keys() for x in d[k]])
    # data older than t-dt will not be kept
    dt = datetime.timedelta(hours=1)
    # add d to the saved data
    for k, v in d.iteritems():
        if k in data.keys():
            data[k].extend(v)
        else:
            data[k]=v
        # remove data older than t-dt
        data[k] = [x for x in data[k] if x[0] > t-dt]


def process_data_per_interval(d):
    """this function is called for each epoch"""
    print d;
    fill_data(d);
    with open("save.p", "wb") as f:
        pickle.dump(data, f, pickle.HIGHEST_PROTOCOL)
