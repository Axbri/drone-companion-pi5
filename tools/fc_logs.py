"""List / download ArduPilot dataflash logs over MAVLink (via mavproxy tcp:5760).
  fc_logs.py list
  fc_logs.py get ID [ID ...]      -> /home/axel/fclogs/<ID>.bin
"""
import os, sys, time
from pymavlink import mavutil
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=201, source_component=190)
m.wait_heartbeat(timeout=5)
def entries():
    m.mav.log_request_list_send(1, 1, 0, 0xFFFF)
    e = {}; t0 = time.time()
    while time.time() - t0 < 15:
        r = m.recv_match(type='LOG_ENTRY', blocking=True, timeout=1)
        if r is None: continue
        e[r.id] = r
        if r.id == r.last_log_num: break
    return e
def get(lid, size):
    buf = bytearray(size); have = [False] * ((size + 89) // 90)
    m.mav.log_request_data_send(1, 1, lid, 0, size); t0 = tl = time.time()
    while not all(have) and time.time() - t0 < 900:
        r = m.recv_match(type='LOG_DATA', blocking=True, timeout=2)
        if r is None or r.id != lid:
            i = have.index(False); m.mav.log_request_data_send(1, 1, lid, i * 90, size - i * 90); continue
        idx = r.ofs // 90
        if idx < len(have) and not have[idx]:
            buf[r.ofs:r.ofs + r.count] = bytes(r.data[:r.count]); have[idx] = True
        if time.time() - tl > 10:
            tl = time.time(); print('  %d: %.0f%%' % (lid, 100 * sum(have) / len(have)), flush=True)
    m.mav.log_request_end_send(1, 1)
    os.makedirs('/home/axel/fclogs', exist_ok=True)
    open('/home/axel/fclogs/%d.bin' % lid, 'wb').write(buf)
    print('saved /home/axel/fclogs/%d.bin (%.1f MB, %.0f s)' % (lid, size / 1e6, time.time() - t0))
e = entries()
if sys.argv[1] == 'list':
    for i in sorted(e):
        print('%3d  %s  %6.1f MB' % (i, time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(e[i].time_utc)), e[i].size / 1e6))
else:
    for a in sys.argv[2:]:
        get(int(a), e[int(a)].size)
