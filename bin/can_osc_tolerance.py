#!/usr/bin/python
#
# Prints information about possible CAN-bus timing configurations that fulfill
# the needs for OpenLCB, i.e., those that are good enough for 300m long buses
# at 125 kbps. The basis upon which this is built is AN1798 CAN Bit Timing
# Requirements by NXP/Freescale
# (https://www.nxp.com/docs/en/application-note/AN1798.pdf)
#
# Output is sorted by the osc tolerance decreasing. When configuring a CAN
# controller, you should pick the first line that is possible to be configured
# in your CAN controller, meaning the one which gives the best oscillator
# tolerance.
#
# Interpreting output:
# 
# 0.98  :  ['len 310 m, prop 7 ps1 4 ps2 4 sjw 4, sample 75.0, o1 1.25 o2 0.98', 'len 320 m, prop 9 ps1 5 ps2 5 sjw 4, sample 75.0, o1 1 o2 0.98']
#
# Before colon: the oscillator tolerance in percent. This is to be interpreted
# as +- this percentage at both source and destination (independently).
#
# len = max cable length
# prop = number of TQ for propagation segment
# ps1 = number of TQ for phase segment 1
# ps2 = number of TQ for phase segment 2
# sjw = number of TQ for (re)-synchronization jump width
# sample = sample point within bit in %
# o1 = osc tolerance limit from one constraint
# o2 = osc tolerance limit from another constraint
#
# There are two entries in this line, which means that two different
# configurations reach the same osc tolerance.


from collections import defaultdict

found = defaultdict(list)
optimal = []

MIN_LENGTH = 300

def eval_timing(propseg, ps1, ps2, sjw):
    global found, MIN_LENGTH, optimal
    # number of time quanta in the bit period. SyncSeg is always 1.
    ntq = 1 + propseg + ps1 + ps2
    sample_point = 1.0 * (1 + propseg + ps1) / ntq
    BAUD = 125000
    sec_per_tq = 1.0 / BAUD / ntq
    # 400 nsec for delay of trnasceiver and receiver. (100 nsec each twice for
    # tx and rx)
    TXRX_DELAY_SEC = 400e-9
    # Cable propagation delay is 5 nsec per meter.
    PROP_SEC_PER_METER = 5e-9
    max_length = (propseg * sec_per_tq - TXRX_DELAY_SEC) / PROP_SEC_PER_METER / 2
    # One formula says that the relative frequency * 2 over 10 bit time
    # cannot be more than the SJW.
    osc1_limit = sjw * 1.0 / (2 * 10 * ntq)
    # Another formula says that over 13 bits time less ps2, we cannot have more
    # drift than -ps1 or +ps2.
    osc2_limit = min(ps1, ps2) / (2 * (13*ntq - ps2))
    real_limit = min(osc1_limit, osc2_limit)
    params = 'len %.0f m, prop %d ps1 %d ps2 %d sjw %d, sample %.1f, o1 %.3g o2 %.3g' % (max_length, propseg, ps1, ps2, sjw, sample_point * 100, osc1_limit * 100, osc2_limit * 100)
    if max_length > MIN_LENGTH:
        found[real_limit].append(params)
    for x in optimal:
        if x[0] > real_limit and x[1] > max_length:
            return
    optimal = [x for x in optimal if x[0] >= real_limit or x[1] >= max_length]
    optimal.append([real_limit, max_length, params])

def print_findings():
    global found, optimal
    for (tol, v) in sorted(found.items(), reverse=True)[:10]:
        print('%-5.3g' % (tol * 100), ": ", v)
    print('\n\noptimal:')
    for x in sorted(optimal, reverse=True):
        print('%-5.3g' % (x[0] * 100), ": ", x[2])
        

# enumerates all possibilities        
for propseg in range(1, 16):
    for ps1 in range(1, 8):
        for ps2 in range(1, 8):
            for sjw in range(1, 1+min(4, min(ps1, ps2))):
                eval_timing(propseg, ps1, ps2, sjw)

print_findings()
