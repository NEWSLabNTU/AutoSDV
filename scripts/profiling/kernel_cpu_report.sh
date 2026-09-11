#!/usr/bin/env bash
# Snapshot of where kernel CPU time goes while the stack is up.
#
# Needs no root. Answers, in order: how much of the machine is kernel, what the
# kernel is doing (softirq class), how much of it is the DDS datagram path, and
# how many DDS participants are paying for it.
#
# The numbers that matter are RATES, not the cumulative counters in /proc --
# every counter below is differenced over WINDOW seconds.
set -u
WINDOW="${1:-10}"

hr() { printf '\n\033[1m── %s\033[0m\n' "$1"; }

hr "CPU split (us=app  sy=kernel  ni=niced-app  si=softirq  hi=hardirq  id=idle)"
top -b -n2 -d 2 | grep '^%Cpu' | tail -1

hr "Load / threads"
printf 'loadavg   : %s\n' "$(cut -d' ' -f1-3 /proc/loadavg)"
printf 'cores     : %s\n' "$(nproc)"
printf 'threads   : %s\n' "$(ls -d /proc/[0-9]*/task/[0-9]* 2>/dev/null | wc -l)"

hr "softirq rate over ${WINDOW}s  (NET_RX dominating => DDS datagram storm)"
a=$(cat /proc/softirqs); sleep "$WINDOW"; b=$(cat /proc/softirqs)
paste <(echo "$a" | awk 'NR>1{s=0;for(i=2;i<=NF;i++)s+=$i;print $1,s}') \
      <(echo "$b" | awk 'NR>1{s=0;for(i=2;i<=NF;i++)s+=$i;print s}') |
  awk -v w="$WINDOW" '{d=$3-$2; if(d>0) printf "%-10s %10d/s\n",$1,d/w}' | sort -k2 -rn

hr "UDP rate over ${WINDOW}s  (RcvbufErr>0 => drops => reliable retransmit spiral)"
u1=$(grep '^Udp:' /proc/net/snmp | tail -1); sleep "$WINDOW"
u2=$(grep '^Udp:' /proc/net/snmp | tail -1)
paste <(echo "$u1") <(echo "$u2") | awk -v w="$WINDOW" '{
  printf "InDatagrams  %10d/s\nOutDatagrams %10d/s\nfanout       %10.2fx  (In/Out; >1 = one send delivered to many sockets)\nNoPorts      %10d/s\nRcvbufErrors %10d/s\nSndbufErrors %10d/s\n",
  ($12-$2)/w,($15-$5)/w,($12-$2)/($15-$5+1),($13-$3)/w,($16-$6)/w,($17-$7)/w }'

hr "per-interface packet rate over ${WINDOW}s"
for i in $(ls /sys/class/net); do
  [ -r "/sys/class/net/$i/statistics/rx_packets" ] || continue
  eval "p_$i=$(cat /sys/class/net/$i/statistics/rx_packets)"
  eval "b_$i=$(cat /sys/class/net/$i/statistics/rx_bytes)"
done
sleep "$WINDOW"
for i in $(ls /sys/class/net); do
  [ -r "/sys/class/net/$i/statistics/rx_packets" ] || continue
  p2=$(cat "/sys/class/net/$i/statistics/rx_packets"); b2=$(cat "/sys/class/net/$i/statistics/rx_bytes")
  eval "p1=\$p_$i"; eval "b1=\$b_$i"
  dp=$((p2-p1)); [ "$dp" -le 0 ] && continue
  printf '%-10s %8d pkt/s  %6d KB/s  avg %5d B  mtu %s\n' \
    "$i" "$((dp/WINDOW))" "$(( (b2-b1)/WINDOW/1024 ))" "$(( (b2-b1)/dp ))" "$(cat /sys/class/net/$i/mtu)"
done

hr "DDS cost centres"
printf 'participants (procs holding DDS sockets) : %s\n' \
  "$(ss -uanp 2>/dev/null | grep -oE 'pid=[0-9]+' | sort -u | wc -l)"
printf 'UDP sockets open                         : %s\n' "$(ss -uan 2>/dev/null | tail -n +2 | wc -l)"
echo 'CycloneDDS threads (7 per participant):'
ps -eLo comm= 2>/dev/null | grep -xE 'tev|recv|recvMC|recvUC|gc|dq\.user|dq\.builtins' |
  sort | uniq -c | sort -rn | sed 's/^/  /'
echo 'composable nodes running as their OWN process (each = 1 participant):'
printf '  component_node : %s\n' "$(pgrep -c -f component_node 2>/dev/null || echo 0)"
printf '  containers     : %s\n' "$(pgrep -c -f component_container 2>/dev/null || echo 0)"

hr "top 15 processes by CPU"
top -b -n2 -d 2 | awk '/PID/{f++} f==2' | head -16
