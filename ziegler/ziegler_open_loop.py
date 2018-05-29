#!/usr/bin/env python3

# compute Ziegler Nichols params from CSV data of edge response

import csv
import numpy as np
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt

# build output and measure lists from csv data
records = csv.DictReader(open('data/uno_output.csv'), fieldnames=['pv', 'out', 'sp'])
l_out = list()
l_pv = list()
for record in records:
    l_out.append(float(record['out']))
    l_pv.append(float(record['pv']))

# filter pv
l_pv = savgol_filter(l_pv, 9, 3)

# some compute
m_max = max(l_pv)
m_min = min(l_pv)
m_delta = m_max - m_min
l_dm = np.gradient(l_pv)

# search inflection point position
t_ifl = np.argmax(l_dm)

# search tangent at inflection point
ifl_tg_coef_a = l_dm[t_ifl]
ifl_tg_coef_b = l_pv[t_ifl] - ifl_tg_coef_a * t_ifl

# search t0 at first output rising edge
t0 = np.argmax(np.gradient(l_out))
t1 = (m_min - ifl_tg_coef_b) / ifl_tg_coef_a
t2 = (m_max - ifl_tg_coef_b) / ifl_tg_coef_a
ziegler_l = t1 - t0
ziegler_t = t2 - t1

print('Tangent at inflection point : y = %.2f *x + %.2f' % (ifl_tg_coef_a, ifl_tg_coef_b))
print('PV max = %.2f  min = %.2f  delta = %.2f  +63%% = %.2f' % (m_max, m_min, m_delta, m_min + m_delta * 0.63))
print('t0 = %i' % t0)
print('t1 = %i' % t1)
print('t2 = %i' % t2)
print('L = %.2f' % ziegler_l)
print('T = %.2f' % ziegler_t)
print('')
print('Ziegler-Nichols results:')
print('------------------------')
print('  P kp=%.2f (T/L)' % (ziegler_t / ziegler_l))
print(' PI kp=%.2f (0.9*T/L) ki=%.2f (0.3/L)' % (0.9 * ziegler_t / ziegler_l, 0.3 / ziegler_l))
print('PID kp=%.2f (1.2*T/L) ki=%.2f (0.5/L) kd=%.2f (0.5* L)' % (1.2 * ziegler_t / ziegler_l, 0.5 / ziegler_l, 0.5 * ziegler_l))

# plot
plt.plot(l_pv, label='process value (pv)')
plt.axhline(m_max, label='pv max', color='olive', linestyle='--')
plt.axhline(m_min, label='pv min', color='chocolate', linestyle='--')
plt.plot([t1, t2], [m_min, m_max], 'ro-', label='pv tangent')
plt.axvline(x=t0, label='t0', color='b', linestyle='--')
plt.axvline(x=t1, label='t1', color='r', linestyle='--')
plt.axvline(x=t2, label='t2', color='g', linestyle='--')
#plt.plot(l_out, label='out (y output)')
#plt.plot(l_dm, label='pv derivative')

# display
plt.title('Ziegler-Nichols open-loop edge response analysis')
plt.legend()
plt.grid()
plt.show()
