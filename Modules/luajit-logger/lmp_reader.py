#!/usr/bin/env python3
'''
Provide Python access to LMP log files.
'''

import binascii
import bisect
import io
from itertools import repeat
import operator
import os
import struct
import subprocess
import sys

# Ensure that we have a msgpack module
try:
    from msgpack import loads
except ImportError:
    from umsgpack import loads

# Access the LCM log format information
# (magic, cnt, t_us, sz_ch, sz_data)
LCM_STRUCT_FMT = ">IQQII"
LCM_MAGIC = 0xeda1da01
LCM_HDR_SZ = struct.calcsize(LCM_STRUCT_FMT)
assert LCM_HDR_SZ == 28, "Bad LCM header calculation"
# (entry_offset, entry_size, entry_timestamp)
IDX_STRUCT_FMT = ">QQQ"
IDX_STRUCT_SZ = struct.calcsize(IDX_STRUCT_FMT)
assert IDX_STRUCT_SZ == 24, "Bad Index entry calculation"
# Log size limited to just below 4GB, per FAT32 file limits
# https://en.wikipedia.org/wiki/File_Allocation_Table#FAT32
MAX_LOG_SZ = 2**32 - 1

# Access the output of the parsed entries
GET_CHANNEL = operator.itemgetter(0)
GET_PAYLOAD = operator.itemgetter(1)


def parse_lcm_hdr(lcm_hdr):
    lcm_magic, count, t_us, sz_ch, sz_data = struct.unpack(LCM_STRUCT_FMT, lcm_hdr)
    assert lcm_magic == LCM_MAGIC, "Bad LCM magic [{}]".format(binascii.hexlify(lcm_hdr))
    # Check the channel name length
    assert sz_ch < 256, "Bad channel size {:d} | data size {:d} | {}".format(
        sz_ch, sz_data, binascii.hexlify(lcm_hdr))
    return count, t_us, sz_ch, sz_data


class LMPReader():
    '''
    Access LMP log files
    '''

    def __init__(self,
                 fname_log,
                 *kargs,
                 fname_idx=None,
                 channel='harvest',
                 use_mmap=False,
                 use_pread=False,
                 **kwargs):
        '''
        For training, we require the log file and the index file
        '''
        # Ensure we have files
        assert os.path.isfile(fname_log), "Log is not a file"
        assert fname_log.endswith('.lmp'), "Wrong log extension"
        self.fname_log = fname_log
        # Check the log size
        log_sz = os.path.getsize(fname_log)
        if log_sz > MAX_LOG_SZ:
            sys.stderr.write("Log is too big\n")
        self.log_sz = log_sz
        # Keep this open for the iterator
        self.f_log = open(self.fname_log, 'rb')
        self.fd_log = self.f_log.fileno()
        # Assume default idx file
        if not fname_idx:
            fname_idx = fname_log.replace(".lmp", ".idx")
            if not os.path.isfile(fname_idx):
                sys.stderr.write("Generating index... [{}]\n".format(fname_idx))
                self.index(fname_idx)
        else:
            assert os.path.isfile(fname_idx), "Bad index specified"
            assert fname_idx.endswith('.idx'), "Wrong index extension"
        # 8 bytes of offset and 8 bytes of timestamp per entry
        idx_sz = os.path.getsize(fname_idx)
        assert idx_sz % IDX_STRUCT_SZ == 0, "Index is corrupted"
        self.idx_sz = idx_sz

        # Set the number of entries
        self.n_entries = idx_sz // IDX_STRUCT_SZ
        # Set the log and index
        if use_mmap:
            raise NotImplementedError("Memory mapping of the log file is not supported, yet")

        # Save the index in memory, as it is small
        with open(fname_idx, 'rb') as f_idx:
            idx_bytes = f_idx.read()
        assert len(idx_bytes) == idx_sz, "Could not read all index data"

        # Grab the index values
        it_idx = struct.iter_unpack(IDX_STRUCT_FMT, idx_bytes)
        self.offsets, self.sizes, self.timestamps = tuple(map(list, zip(*it_idx)))

        # Form easily accessed data of offsets, sizes and timestamps
        assert len(self.offsets) == self.n_entries
        assert len(self.timestamps) == self.n_entries

        # TODO: Ensure positive offset differences
        # Find the diff, for easier reads
        # offsets_a, offsets_b = tee(self.offsets)
        # next(offsets_b)
        # self.sizes = list(map(operator.sub, offsets_b, offsets_a))
        # Add the length of the last entry
        # TODO: This does not work for a corrupted log. Check the last index entry
        # self.sizes.append(self.log_sz - next(offsets_a))
        # Check the channel
        self.channel = channel
        # Select the decoder
        self.decode = loads
        # TODO: Use a single file handle
        '''
        Use the UNIX pread function to read at a particular offset.
        Running a seek followed by a read is not thread-safe
        NOTE: Multiple reads on the same file may be unspecified
        https://stackoverflow.com/questions/5155695/pread-threadsafe-or-not
        http://pubs.opengroup.org/onlinepubs/009695399/functions/pread.html
        '''
        self.use_pread = use_pread

    def read_raw_entry(self, offset_entry, sz_entry):
        # Open the file in a context manager for automatic cleanup
        with open(self.fname_log, 'rb') as f_log:
            f_log.seek(offset_entry, io.SEEK_SET)
            raw_entry = f_log.read(sz_entry)
        return raw_entry

    def get_slice1(self, idx_slice):
        '''
        Get a continuous range: Return an iterable
        '''
        # Ensure that our slice is within range
        assert idx_slice.step is None, "No support for step sizes, yet"
        idx_start = idx_slice.start if idx_slice.start else 0
        idx_stop = idx_slice.stop if idx_slice.stop else self.n_entries
        assert idx_stop <= self.n_entries
        offset_entry0 = self.offsets[idx_start]
        sizes_entries = self.sizes[idx_start:idx_stop]
        with open(self.fname_log, 'rb') as f_log:
            # Seek to the start of the range
            f_log.seek(offset_entry0, io.SEEK_SET)
            # Read consecutive entries
            # NOTE: Must return a list, since mapping via a file operation
            raw_entries = list(map(f_log.read, sizes_entries))
            # NOTE: Not checking EOF
        return raw_entries

    def parse_raw_entry(self, raw_entry):
        '''
        Read the log at an offset and output the channel and payload
        '''
        assert len(raw_entry) >= LCM_HDR_SZ, "Bad header {:d}".format(len(raw_entry))
        lcm_hdr = raw_entry[:LCM_HDR_SZ]
        # Check the header
        _, _, sz_ch, sz_data = parse_lcm_hdr(lcm_hdr)
        # Channel check for verification
        channel = raw_entry[LCM_HDR_SZ:LCM_HDR_SZ + sz_ch].decode()
        assert channel == self.channel, "Channel does not match [{}] != [{}]".format(
            channel, self.channel)
        # Form and load the payload
        payload = raw_entry[LCM_HDR_SZ + sz_ch:LCM_HDR_SZ + sz_ch + sz_data]
        return channel, payload

    def index(self, fname_idx):
        with open(fname_idx, 'ab') as f_idx:
            while True:
                offset = self.f_log.tell()
                hdr = self.f_log.read(LCM_HDR_SZ)
                if len(hdr) != LCM_HDR_SZ:
                    break
                _, t_us, sz_ch, sz_msg = parse_lcm_hdr(hdr)
                sz_data = sz_ch + sz_msg
                self.f_log.seek(sz_data, io.SEEK_CUR)
                idx_entry = struct.pack(IDX_STRUCT_FMT, offset, LCM_HDR_SZ + sz_data, t_us)
                f_idx.write(idx_entry)

    def get_slice(self, idx_slice):
        '''
        Use pread to get a slice
        '''
        it_raw = map(os.pread, repeat(self.fd_log), self.sizes[idx_slice], self.offsets[idx_slice])
        it_parsed = map(self.parse_raw_entry, it_raw)
        it_decoded = map(self.decode, map(GET_PAYLOAD, it_parsed))
        return list(it_decoded)

    def time_to_idx(self, time_entry):
        '''Use microseconds'''
        if time_entry is None:
            return None
        return bisect.bisect(self.timestamps, int(time_entry * 1e6))

    def __getitem__(self, idx_entry):
        '''
        Allow indexing by entry count or entry time
        '''
        # If a consecutive slice, run a faster operation
        if isinstance(idx_entry, slice):
            if isinstance(idx_entry.start, float):
                idx_entry.start = self.time_to_idx(idx_entry.start)
                idx_entry.stop = self.time_to_idx(idx_entry.stop)
                idx_entry.step = int(idx_entry.step * 1e6)
            return self.get_slice(idx_entry)
        elif isinstance(idx_entry, float):
            idx_entry = self.time_to_idx(idx_entry)

        # Read the entry, given the offset in bytes, and the size in bytes
        offset_entry = self.offsets[idx_entry]
        sz_entry = self.sizes[idx_entry]
        # Using pread may be faster, but hasn't been observed
        raw_entry = os.pread(self.fd_log, sz_entry, offset_entry)
        # raw_entry = self.read_raw_entry(offset_entry, sz_entry)
        # Optionally, check the correctness
        # if is_safe:
        #     sz_read = len(raw_entry)
        #     assert sz_read > 0, "Payload EOF @ {:d} / {:d}".format(offset_entry, self.log_sz)
        #     assert sz_read == sz_entry, "Bad raw entry read @ {:d}: {} != {}".format(
        #         offset_entry, sz_read, sz_entry)
        parsed_entry = self.parse_raw_entry(raw_entry)
        decoded_entry = self.decode(GET_PAYLOAD(parsed_entry))
        return decoded_entry

    def __len__(self):
        '''
        The length of the dataset is the number of log entries
        '''
        return self.n_entries

    # TODO: Add tee for GET_CHANNEL
    def __iter__(self):
        '''thread-safe iterating'''
        # it_raw = map(self.read_raw_entry, self.offsets, self.sizes)
        it_raw = map(os.pread, repeat(self.fd_log), self.sizes, self.offsets)
        it_parsed = map(self.parse_raw_entry, it_raw)
        it_decoded = map(self.decode, map(GET_PAYLOAD, it_parsed))
        return it_decoded

    def split(self, intervals, dest, index_only=True):
        '''
        Take an iterable of intervals that generates (t1, t2) for each entry
        '''
        assert os.path.isdir(dest), "Improper destination"
        for i, (t_a, t_b) in enumerate(intervals):
            idx_a = self.time_to_idx(t_a)
            idx_b = self.time_to_idx(t_b)
            if index_only:
                interval = slice(idx_a, idx_b)
                offsets_interval = self.offsets[interval]
                sizes_interval = self.sizes[interval]
                timestamps_interval = self.timestamps[interval]
                fname_index = "interval_{:06d}.idx".format(i)
                with open(os.path.join(dest, fname_index), 'wb') as f_idx:
                    for off, sz, ts in zip(offsets_interval, sizes_interval, timestamps_interval):
                        entry = struct.pack(IDX_STRUCT_FMT, off, sz, ts)
                        f_idx.write(entry)
            else:
                fname_outlog = "interval_{:06d}.lmp".format(i)
                offset_a = self.offsets[idx_a]
                offset_b = self.offsets[idx_b] + self.sizes[idx_b]
                count_bytes = offset_b - offset_a
                subprocess.run([
                    "dd", "if={}".format(self.fname_log), "of={}".format(
                        os.path.join(dest, fname_outlog)), "skip={:d}".format(offset_a),
                    "count={:d}".format(count_bytes), "iflag=skip_bytes,count_bytes"
                ])


class LMPTransformed(LMPReader):
    '''
    Transform log data.
    '''

    def __init__(self, *kargs, output_transform=None, sequence_transform=None, **kwargs):
        super().__init__(*kargs, **kwargs)
        self.output_transform = output_transform if callable(output_transform) else None
        self.sequence_transform = sequence_transform if callable(sequence_transform) else None

    def __getitem__(self, idx_get):
        '''
        Grab and transform data
        '''
        obj = super().__getitem__(idx_get)
        # Return transformed data
        if not self.output_transform:
            return obj
        if isinstance(obj, list):
            obj_transformed = list(map(self.output_transform, obj))
            if self.sequence_transform:
                obj_transformed = self.sequence_transform(obj)
        else:
            obj_transformed = self.output_transform(obj)
        return obj_transformed

    def __iter__(self):
        it_decoded = super().__iter__()
        it_transformed = map(self.output_transform, it_decoded)
        return it_transformed


if __name__ == '__main__':
    '''
    Print out information about an LMP file
    '''
    import argparse
    import csv
    parser = argparse.ArgumentParser()
    parser.add_argument("logfile")
    parser.add_argument("--splits", help="Tab separated file of timestamp a, b")
    args = vars(parser.parse_args())
    print(args)
    d = LMPReader(args.logfile)
    if args['split']:
        assert os.path.isfile(args['split']), "TSV splits is not a file"
        assert args['split'].endswith('.tsv'), "Wrong tsv extension"
        with open(args['split'], 'r', newline='') as tsvfile:
            trialsreader = csv.reader(tsvfile, delimiter='\t', quoting=csv.QUOTE_NONNUMERIC)
            intervals = list(trialsreader)
        d.split(intervals)
