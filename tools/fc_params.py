"""Read / set ArduPilot parameters from the Pi, through mavproxy's TCP endpoint.

Usage (on the Pi, droneweb-venv has pymavlink):
  fc_params.py get NAME [NAME ...]
  fc_params.py set NAME=VALUE [NAME=VALUE ...]     # prints old -> new (readback from the FC)

Uses tcp:127.0.0.1:5760 (mavproxy --out tcpin, see systemd/mavproxy-ntrip.service) so it does
not disturb droneweb's own udp endpoint. param_set is REAL32; ArduPilot converts to the
parameter's own type and stores it to EEPROM immediately.
"""
import sys
import time

from pymavlink import mavutil

ENDPOINT = "tcp:127.0.0.1:5760"
FC_SYS, FC_COMP = 1, 1


def _wait_value(m, name, timeout=1.5):
    t0 = time.time()
    while time.time() - t0 < timeout:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        if r and r.param_id.rstrip("\x00") == name:
            return r.param_value
    return None


def get(m, name, tries=4):
    for _ in range(tries):
        m.mav.param_request_read_send(FC_SYS, FC_COMP, name.encode(), -1)
        v = _wait_value(m, name)
        if v is not None:
            return v
    return None


def set_(m, name, val, tries=3):
    for _ in range(tries):
        m.mav.param_set_send(FC_SYS, FC_COMP, name.encode(), float(val),
                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        v = _wait_value(m, name)
        if v is not None:
            return v
    return None


def main():
    if len(sys.argv) < 3 or sys.argv[1] not in ("get", "set"):
        sys.exit(__doc__)
    m = mavutil.mavlink_connection(ENDPOINT, source_system=201, source_component=190)
    if m.wait_heartbeat(timeout=5) is None:
        sys.exit("no heartbeat from the FC")
    if sys.argv[1] == "get":
        for n in sys.argv[2:]:
            print("%-18s %s" % (n, get(m, n)))
    else:
        for kv in sys.argv[2:]:
            n, v = kv.split("=")
            print("%-18s %s -> %s" % (n, get(m, n), set_(m, n, v)))


if __name__ == "__main__":
    main()
