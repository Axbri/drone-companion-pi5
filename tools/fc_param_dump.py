"""Download the FC's full parameter list and write it in Mission Planner .param format.

  fc_param_dump.py OUT.param

Reads through mavproxy's TCP endpoint (see fc_params.py). Requests the whole list, then
re-requests anything missing individually — a dropped PARAM_VALUE would otherwise leave a
silent hole in the backup. Exits non-zero if the list is still incomplete.
"""
import sys
import time

from pymavlink import mavutil

ENDPOINT = "tcp:127.0.0.1:5760"
FC_SYS, FC_COMP = 1, 1


def main(out_path):
    m = mavutil.mavlink_connection(ENDPOINT, source_system=201, source_component=190)
    if m.wait_heartbeat(timeout=5) is None:
        sys.exit("no heartbeat from the FC")

    params = {}
    total = None
    m.mav.param_request_list_send(FC_SYS, FC_COMP)
    last_rx = time.time()
    while time.time() - last_rx < 5:
        r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if r is None:
            continue
        last_rx = time.time()
        params[r.param_id.rstrip("\x00")] = r.param_value
        total = r.param_count
        if total and len(params) == total:
            break
    print("received %d of %s" % (len(params), total))

    # fill holes by index
    if total and len(params) < total:
        print("re-requesting %d missing by index..." % (total - len(params)))
        for idx in range(total):
            if len(params) >= total:
                break
            m.mav.param_request_read_send(FC_SYS, FC_COMP, b"", idx)
            t0 = time.time()
            while time.time() - t0 < 0.4:
                r = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.2)
                if r is not None:
                    params[r.param_id.rstrip("\x00")] = r.param_value
                    break
        print("now %d of %d" % (len(params), total))

    with open(out_path, "w") as f:
        for k in sorted(params):
            v = params[k]
            f.write("%s,%s\n" % (k, ("%.6f" % v).rstrip("0").rstrip(".") if v % 1 else "%d" % int(v)))
    print("wrote %s (%d parameters)" % (out_path, len(params)))
    if total and len(params) < total:
        sys.exit("INCOMPLETE: %d of %d" % (len(params), total))


if __name__ == "__main__":
    main(sys.argv[1])
