#!/bin/bash
# convert_map.sh — Convert kantor.posegraph ke kantor.pgm + kantor.yaml
# Jalankan di Jetson: bash ~/amr_mp/convert_map.sh

source /opt/ros/foxy/setup.bash
source ~/amr_mp/install/setup.bash

MAP_PATH="$HOME/amr_mp/maps/kantor"

echo "[convert_map] Load posegraph dan publish /map..."

# Jalankan slam_toolbox localization mode untuk load posegraph
ros2 launch slam_toolbox localization_launch.py \
  use_sim_time:=false \
  map_file_name:=${MAP_PATH} &

SLAM_PID=$!
echo "[convert_map] slam_toolbox PID: $SLAM_PID"

# Tunggu slam_toolbox siap publish /map (max 15 detik)
echo "[convert_map] Menunggu /map tersedia..."
for i in $(seq 1 15); do
    sleep 1
    MAP_COUNT=$(ros2 topic list 2>/dev/null | grep "^/map$" | wc -l)
    if [ "$MAP_COUNT" -gt "0" ]; then
        # Cek ada publisher
        PUB=$(ros2 topic info /map 2>/dev/null | grep "Publisher count" | awk '{print $3}')
        if [ "$PUB" -gt "0" ] 2>/dev/null; then
            echo "[convert_map] /map siap (publisher: $PUB). Menyimpan..."
            break
        fi
    fi
    echo "[convert_map] Tunggu... ($i/15)"
done

sleep 2

# Simpan peta
ros2 run nav2_map_server map_saver_cli -f ${MAP_PATH}
SAVE_STATUS=$?

# Matikan slam_toolbox
kill $SLAM_PID 2>/dev/null
wait $SLAM_PID 2>/dev/null

if [ $SAVE_STATUS -eq 0 ]; then
    echo "[convert_map] SUKSES: ${MAP_PATH}.pgm dan ${MAP_PATH}.yaml tersimpan."
    ls -lh ${MAP_PATH}*
else
    echo "[convert_map] GAGAL menyimpan pgm. Cek apakah posegraph valid."
fi
