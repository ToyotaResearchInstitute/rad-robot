#!/usr/bin/env python
import argparse
import itertools
import operator

from lmp_reader import LMPReader

parser = argparse.ArgumentParser()
parser.add_argument("logfile")
args = parser.parse_args()
d = LMPReader(args.logfile)
print(d)
print("Size", len(d))
print(d[0].keys())
for i, v in enumerate(d):
    assert isinstance(v, dict), "Element {:d}: {}".format(i, type(v))
print("All elements OK")


def grouper(iterable, n, fillvalue=None):
    '''
    Collect data into fixed-length chunks or blocks
    https://docs.python.org/3/library/itertools.html#recipes
    '''
    # grouper('ABCDEFG', 3, 'x') --> ABC DEF Gxx"
    args = [iter(iterable)] * n
    return itertools.zip_longest(*args, fillvalue=fillvalue)


interval_length = 3
get_time = operator.itemgetter(b'timestamp')
# times = list(map(lambda v: v["video0"]["t"], g))
# Groups of interval_length entries: Simply uses an iterator
for i, g in enumerate(itertools.islice(grouper(d, interval_length), 2)):
    print("Group", i, type(g), len(g))
    times = list(map(get_time, g))
    print(times)
    # Find the time differences
    times_a, times_b = itertools.tee(times)
    next(times_b)
    dtimes = list(map(operator.sub, times_b, times_a))
    print(dtimes)

# Use the consecutive method: Uses a slice object
for i, index_entry in enumerate(itertools.islice(range(0, len(d), interval_length), 2)):
    interval = slice(index_entry, index_entry + interval_length)
    print("Interval", i, interval)
    g = d[interval]
    times = list(map(get_time, g))

    print(times)
    # Find the time differences
    times_a, times_b = itertools.tee(times)
    next(times_b)
    dtimes = list(map(operator.sub, times_b, times_a))
    print(dtimes)

print("Other times")
times = list(map(get_time, d[:3]))
print(times)
print("Other times2")
times = list(map(get_time, d[-3:]))
print(times)