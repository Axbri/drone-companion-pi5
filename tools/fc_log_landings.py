"""Summarise the landings in an ArduPilot dataflash log: per LAND-mode segment, the
precland state (PL.TAcq), PrecLand/land-detector messages, throttle and land_complete
events from touchdown to disarm.

  fc_log_landings.py LOG.bin
"""
import sys

from pymavlink import mavutil

EV = {10: "ARMED", 11: "DISARMED", 17: "LAND_COMPLETE_MAYBE", 18: "LAND_COMPLETE", 28: "NOT_LANDED",
      15: "AUTO_ARMED", 16: "TAKEOFF", 25: "SET_HOME", 62: "LAND_CANCELLED_BY_PILOT"}


def main(path):
    m = mavutil.mavlink_connection(path)
    events = []           # (t, kind, text)
    ctun = []             # (t, ThO, Alt)
    pl = []               # (t, TAcq)
    while True:
        r = m.recv_match(type=["MSG", "EV", "MODE", "PL", "CTUN"], blocking=False)
        if r is None:
            break
        t = r.TimeUS / 1e6
        k = r.get_type()
        if k == "MSG":
            events.append((t, "MSG", r.Message))
        elif k == "EV":
            events.append((t, "EV", EV.get(r.Id, str(r.Id))))
        elif k == "MODE":
            events.append((t, "MODE", getattr(r, "asText", None) or str(r.Mode)))
        elif k == "PL":
            pl.append((t, r.TAcq))
        elif k == "CTUN":
            ctun.append((t, r.ThO, r.Alt))
    t0 = events[0][0] if events else 0
    print("events (t rel. %.0f s):" % t0)
    for t, k, s in events:
        if k == "MODE" or k == "EV" or "PrecLand" in s or "Land" in s or "land" in s:
            print("  %8.1f  %-5s %s" % (t - t0, k, s))
    # TAcq transitions
    print("\nPL.TAcq transitions:")
    last = None
    for t, a in pl:
        if a != last:
            print("  %8.1f  TAcq=%d" % (t - t0, a))
            last = a
    # throttle around each LAND_COMPLETE / DISARMED
    print("\nthrottle (CTUN.ThO) 0.5 s samples in the 20 s before each DISARMED:")
    for t, k, s in events:
        if k == "EV" and s == "DISARMED":
            win = [(tt, th, al) for tt, th, al in ctun if t - 20 <= tt <= t]
            lastp = -1
            line = []
            for tt, th, al in win:
                if tt - lastp >= 1.0:
                    line.append("%.0f:%.2f" % (tt - t0, th))
                    lastp = tt
            print("  disarm at %.1f: %s" % (t - t0, " ".join(line)))


if __name__ == "__main__":
    main(sys.argv[1])
