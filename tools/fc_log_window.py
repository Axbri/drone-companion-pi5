"""Dump attitude request/actual, throttle, EKF velocity and position-controller targets in a
time window of a dataflash log (0.5 s samples). Times are seconds relative to the first
message in the log, like fc_log_landings.py prints them.

  fc_log_window.py LOG.bin T_START T_END
"""
import sys

from pymavlink import mavutil


def main(path, ta, tb):
    m = mavutil.mavlink_connection(path)
    t0 = None
    att = []; ctun = []; xkf = []; psc = []; ldr = []
    while True:
        r = m.recv_match(type=["ATT", "CTUN", "XKF1", "PSCN", "PSCE", "LAND", "RFND"], blocking=False)
        if r is None:
            break
        t = r.TimeUS / 1e6
        if t0 is None:
            t0 = t
        t -= t0
        if t < ta or t > tb:
            continue
        k = r.get_type()
        if k == "ATT":
            att.append((t, r.DesRoll, r.Roll, r.DesPitch, r.Pitch))
        elif k == "CTUN":
            ctun.append((t, r.ThO, getattr(r, "DSAlt", 0), getattr(r, "SAlt", 0), r.CRt))
        elif k == "XKF1" and getattr(r, "C", 0) == 0:
            xkf.append((t, r.VN, r.VE, r.VD))
        elif k == "PSCN":
            psc.append((t, r.TPN, r.PN, r.TVN, r.VN, r.TAN, r.AN))
        elif k == "RFND":
            ldr.append((t, r.Dist, getattr(r, "Stat", -1)))

    def near(lst, t, tol=0.3):
        best = None
        for x in lst:
            if abs(x[0] - t) <= tol and (best is None or abs(x[0] - t) < abs(best[0] - t)):
                best = x
        return best

    print("    t   DesR   Roll  DesP  Pitch | ThO  CRt | EKF VN   VE | PSCN tgt-pos pos  tgt-vel vel | RFND")
    t = ta
    while t <= tb:
        a = near(att, t); c = near(ctun, t); x = near(xkf, t); p = near(psc, t); l = near(ldr, t)
        print("%6.1f %6s %6s %6s %6s | %4s %5s | %6s %6s | %7s %7s %6s %6s | %s" % (
            t,
            "%.1f" % a[1] if a else "-", "%.1f" % a[2] if a else "-", "%.1f" % a[3] if a else "-", "%.1f" % a[4] if a else "-",
            "%.2f" % c[1] if c else "-", "%.0f" % c[4] if c else "-",
            "%.2f" % x[1] if x else "-", "%.2f" % x[2] if x else "-",
            "%.2f" % p[1] if p else "-", "%.2f" % p[2] if p else "-", "%.2f" % p[3] if p else "-", "%.2f" % p[4] if p else "-",
            "%.2f/%s" % (l[1], l[2]) if l else "-"))
        t += 0.5


if __name__ == "__main__":
    main(sys.argv[1], float(sys.argv[2]), float(sys.argv[3]))
