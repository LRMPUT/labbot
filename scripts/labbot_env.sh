#!/usr/bin/env bash
LABBOT_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

source /opt/ros/lyrical/setup.bash
if [ -f "$LABBOT_WS/install/setup.bash" ]; then
  source "$LABBOT_WS/install/setup.bash"
fi

# Fast DDS: user data over TCP instead of fragmented UDP (much better over WiFi).
# All communicating processes must use the same value.
export FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA
