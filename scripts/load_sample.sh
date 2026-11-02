#!/usr/bin/env bash
# READ-ONLY: samples host CPU / RAM / temperature every 2 s for N seconds (default 90) into /tmp/load_sample.csv.
# Sends nothing to ROS, touches no repo file.
N="${1:-90}"
OUT=/tmp/load_sample.csv
echo "t_s,load1,cpu_busy_pct,mem_avail_MB,swap_used_MB,max_temp_mC" > "$OUT"
pt=0; pi=0
for t in $(seq 0 2 "$N"); do
  read -r _ u n s idle io irq sirq st _ < /proc/stat
  tot=$((u+n+s+idle+io+irq+sirq+st)); idl=$((idle+io))
  if [ "$pt" -gt 0 ] && [ "$tot" -gt "$pt" ]; then busy=$(( 100*((tot-pt)-(idl-pi))/(tot-pt) )); else busy=NA; fi
  pt=$tot; pi=$idl
  ma=$(awk '/MemAvailable/{print int($2/1024)}' /proc/meminfo)
  sw=$(awk '/SwapTotal/{a=$2}/SwapFree/{b=$2}END{print int((a-b)/1024)}' /proc/meminfo)
  tm=$(cat /sys/class/thermal/thermal_zone*/temp 2>/dev/null | sort -n | tail -1)
  echo "$t,$(cut -d' ' -f1 /proc/loadavg),$busy,$ma,$sw,${tm:-NA}" >> "$OUT"
  sleep 2
done
awk -F, 'NR>1{ if(min==""||$4<min)min=$4; if($3!="NA"&&$3+0>b)b=$3+0; if($6!="NA"&&$6+0>m)m=$6+0; if($2+0>l)l=$2+0 }
  END{printf "min MemAvailable %d MB | peak CPU %d%% | peak load1 %.2f | peak temp %.1f C\n",min,b,l,m/1000}' "$OUT"
echo "wrote $OUT"
