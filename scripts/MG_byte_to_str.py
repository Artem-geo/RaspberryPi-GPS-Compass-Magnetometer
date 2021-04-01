"""
This program is used for conversion of magnetometer's output bin data.
After conversion you'll get .txt file with magnetic field values, date and time
"""


from textwrap import wrap
import numpy as np
import datetime


def to_hexadecimal(val, nbits):
    return hex((val + (1 << nbits)) % (1 << nbits))


def add_zero(list_with_digits):
    for i, el in enumerate(list(list_with_digits)):
        if len(el) == 1:
            list_with_digits[i] = f'0{list_with_digits[i]}'
    return list_with_digits


def binstr2formatted_list(binstr):
    '''
        Extract data from magnetometer and raspberry pi time from the bin string
    '''
    try:
        # convert bin string into the list with hexadecimal values
        hexlist = np.array(wrap(binstr.hex(), 2))

        # find 00 sign - the end of data string from magnetometer
        zero_sign = np.where(hexlist == '00')[0][0]

        # split list with hex values into 2 lists
        data_from_magn = hexlist[:zero_sign]
        time_from_pc = hexlist[zero_sign+1:]

        #
        sub_magn = np.where(data_from_magn == '1a')[0]

        for i in sub_magn:
            data_from_magn[i+1] = hex(int(data_from_magn[i+1], 16) - int('0x80', 16)).replace('0x', '')

        data_from_magn = np.delete(data_from_magn, sub_magn)

        data_from_magn = add_zero(data_from_magn)
        time_from_pc = add_zero(time_from_pc)

        return data_from_magn, time_from_pc

    except:
        pass


def str2hex(dlist, *args):
    if len(args) == 1:
        return '0x' + ''.join(dlist[args[0]:])
    return '0x' + ''.join(dlist[args[0]:args[1]])


#####################################################
# path to the file with binary data
filename = '/home/artem/Documents/python/pet_prjs/mag_heading_correction/data/20200305/test/20210305_134621_MG.txt'
#####################################################

gps_epoch = datetime.datetime(1980, 1, 6).timestamp()  # beginnig of GPS time (s)

with open(filename, 'rb') as f:
    f_conv = open(filename.replace('.txt', '_c.txt'), 'w')
    f_conv.write('GPST DATE(UTC) TIME(UTC) FIELD\n')
    for line in f.readlines():
        try:
            # get data from bin file
            magn, time_pc = binstr2formatted_list(line)
            field = int(str2hex(magn, 0, 4), 16)
            dt = int(str2hex(time_pc, 0, 4), 16)
            pph = int(str2hex(time_pc[:-1], 4), 16)

            # from file we get local time (+3 in SPb)
            # convert time to UTC (in seconds since 01.01.1970)
            meas_time = float(f'{dt}.{pph}') - 10800

            # create datetime object with UTC time
            meas_time_str = datetime.datetime.fromtimestamp(meas_time).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]

            # convert UTC time to GPST
            meas_time_gps = meas_time - gps_epoch + 18

            # write all data into the file
            f_conv.write(f'{meas_time_gps:.3f} {meas_time_str} {field/1000}\n')

        except:
            pass

    f_conv.close()
    f.close()
