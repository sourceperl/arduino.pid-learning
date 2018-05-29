#!/usr/bin/env python3

import sys
import time
import threading
from serial import Serial


class Uno(Serial):
    def send_cmd(self, cmd):
        self.write((cmd + '\r\n').encode())


def plot_data_thread():
    while True:
        msg = uno.readline().decode().strip()
        try:
            pv, out, sp = map(float, msg.split(','))
            print('%f,%f,%f' % (pv, out, sp), file=sys.stdout, flush=True)
        except ValueError:
            print('MSG: %s' % msg, file=sys.stderr, flush=True)
            continue


if __name__ == '__main__':
    uno = Uno('/dev/ttyACM0', 9600)

    # start serial IO thread
    threading.Thread(target=plot_data_thread, daemon=True).start()

    # use for ziegler indicial response
    uno.send_cmd('')
    uno.send_cmd('MAN')
    time.sleep(3.0)
    uno.send_cmd('OUT 50.0')
    time.sleep(3600.0)
