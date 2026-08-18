#!/bin/bash
# SIM7600E-H 4G via QMI raw-ip. Brings up wwan0, keeps it up, reconnects on drop.
# APN comes from /etc/qmi-network.conf (used by qmi-network). Route metric 700 so
# wlan0 (metric 600) stays primary at home; 4G is the route when WiFi is absent.
DEV=/dev/cdc-wdm0
IF=wwan0
METRIC=700

log(){ logger -t 4g-modem "$*"; echo "4g-modem: $*"; }

mask2cidr(){ local m="$1" c=0 o a b cc d; IFS=. read -r a b cc d <<<"$m"
  for o in "$a" "$b" "$cc" "$d"; do while [ "${o:-0}" -gt 0 ]; do c=$((c + o%2)); o=$((o/2)); done; done
  echo "${c:-29}"; }

wait_dev(){ local i; for i in $(seq 1 30); do [ -e "$DEV" ] && return 0; sleep 2; done; return 1; }

configure_ip(){
  local S IP GW MASK CIDR
  S="$(qmicli -d "$DEV" -p --wds-get-current-settings 2>/dev/null)"
  IP="$(awk -F': ' '/IPv4 address:/{gsub(/^ +/,"",$2); print $2; exit}' <<<"$S")"
  GW="$(awk -F': ' '/IPv4 gateway address:/{gsub(/^ +/,"",$2); print $2; exit}' <<<"$S")"
  MASK="$(awk -F': ' '/IPv4 subnet mask:/{gsub(/^ +/,"",$2); print $2; exit}' <<<"$S")"
  [ -z "$IP" ] || [ -z "$GW" ] && return 1
  CIDR="$(mask2cidr "${MASK:-255.255.255.248}")"
  ip addr flush dev "$IF"
  ip addr add "${IP}/${CIDR}" dev "$IF"
  ip link set "$IF" up
  ip route replace default via "$GW" dev "$IF" metric "$METRIC"
  log "up: ${IP}/${CIDR} gw ${GW} metric ${METRIC}"
}

start(){
  wait_dev || { log "no $DEV"; return 1; }
  ip link set "$IF" down 2>/dev/null
  echo Y > "/sys/class/net/$IF/qmi/raw_ip" 2>/dev/null
  ip link set "$IF" up
  qmi-network "$DEV" stop >/dev/null 2>&1   # clear any stale session
  qmi-network "$DEV" start || { log "qmi-network start failed"; return 1; }
  sleep 2
  configure_ip
}

stop(){
  qmi-network "$DEV" stop >/dev/null 2>&1
  ip route del default dev "$IF" 2>/dev/null
  ip addr flush dev "$IF" 2>/dev/null
  log "stopped"
}

alive(){ ping -I "$IF" -c1 -W5 8.8.8.8 >/dev/null 2>&1 || ping -I "$IF" -c1 -W5 1.1.1.1 >/dev/null 2>&1; }

[ "$1" = "stop" ] && { stop; exit 0; }

trap 'stop; exit 0' TERM INT
until start; do log "retry in 10s"; sleep 10; done
while true; do
  sleep 20
  alive && continue
  sleep 3; alive && continue        # second chance before reconnecting
  log "4G link down -- reconnecting"
  stop; sleep 3
  until start; do log "retry in 10s"; sleep 10; done
done
