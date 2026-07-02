#!/bin/sh
# log_idle.sh — capture a clean IMU idle baseline on the AR.Drone 2.0.
#
# Run ON THE DRONE (paste into telnet, or push + run). Drone must be flat, still,
# and untouched. Captures ~30 s (6000 frames @ ~200 Hz) of decoded ACC+GYRO to
# /data/video/idle.csv, which is the FTP root on port 21 so the Mac can pull it:
#
#   curl -o data/sensors/idle_flat_30s.csv ftp://192.168.1.1/idle.csv
#
# Reversible: a battery unplug/replug restores stock firmware.

# free the navboard: respawner FIRST, then the flight app (see CLAUDE.md ground rules)
kill -9 $(ps | grep respawner | grep -v grep | awk '{print $1}') 2>/dev/null
killall -9 program.elf 2>/dev/null
sleep 1

# open the port and start acquisition
stty -F /dev/ttyO1 raw -echo
echo -en "\001" > /dev/ttyO1
sleep 1
dd if=/dev/ttyO1 bs=4096 count=40 of=/dev/null 2>/dev/null   # flush stale frames

OUT=/data/video/idle.csv
echo "idx ax ay az gx gy gz" > "$OUT"
cat /dev/ttyO1 | hexdump -v -e '1/1 "%d\n"' | awk '
function s16(lo,hi){v=lo+hi*256; if(v>=32768)v-=65536; return v}
BEGIN{cnt=0}
{ b[n++]=$1
  while (n>=58) {
    if (b[0]==58 && b[1]==0) {
      print cnt++, s16(b[4],b[5]),s16(b[6],b[7]),s16(b[8],b[9]),s16(b[10],b[11]),s16(b[12],b[13]),s16(b[14],b[15])
      for(i=58;i<n;i++) b[i-58]=b[i]; n-=58
    } else { for(i=1;i<n;i++) b[i-1]=b[i]; n-- }
  }
  if (cnt>=6000) exit
}' >> "$OUT"
echo "done: $(wc -l < "$OUT") lines -> $OUT"
