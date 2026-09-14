#!/bin/bash
# Copies the actual working code out of the development tree into this repository.
# Run on the workstation. Nothing is fabricated: every file here is the one that ran.
set -e

SRC_GELLO=/home/data/gello/gello_software/ros2/src/franka_gello_state_publisher
SRC_TOOLS=/home/data/gello/tools
SRC_CAL=/home/data/gello/calib_final.json
R="$(cd "$(dirname "$0")" && pwd)"

echo "=== driver ==="
cp "$SRC_GELLO/franka_gello_state_publisher/feetech/driver.py"                  "$R/driver/driver.py"
cp "$SRC_GELLO/franka_gello_state_publisher/feetech/motor_configs/sts3215.yaml" "$R/driver/sts3215.yaml"
cp "$SRC_TOOLS/gen_sts3215_yaml.py"                                             "$R/driver/gen_sts3215_yaml.py"
touch "$R/driver/__init__.py"
wc -l "$R"/driver/*.py "$R"/driver/*.yaml

echo "=== tools ==="
for F in scan_ids.py changeid.py remap_chain.py permute_ids.py jog.py torque_off.py \
         set_zero.py verify_sign.py measure_all.py grip_range.py chk_range.py \
         probe.py test_driver.py; do
  if [ -f "$SRC_TOOLS/$F" ]; then
    cp "$SRC_TOOLS/$F" "$R/tools/$F"
  else
    echo "  MISSING: $F"
  fi
done
ls -1 "$R/tools/"

echo "=== config ==="
cp "$SRC_GELLO/config/uarm_fr3.yaml" "$R/config/uarm_fr3.yaml.example"
cp "$SRC_CAL"                        "$R/config/calib_final.json"

echo "=== strip machine-specific serial from example config ==="
sed -i 's#^\( *com_port: *"\).*\("\)#\1usb-1a86_USB_Single_Serial_XXXXXXXXXX-if00\2#' "$R/config/uarm_fr3.yaml.example"
grep -n com_port "$R/config/uarm_fr3.yaml.example"

echo "=== udev ==="
cat > "$R/config/99-uarm.rules" << 'RULES'
# Waveshare Serial Bus Servo Driver Board (CH340).  Pins the device to /dev/uarm
# and makes it world-accessible, so no dialout re-login is needed.
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3", MODE="0666", SYMLINK+="uarm"
RULES

echo "=== normalise hardcoded paths ==="
cd "$R"
grep -rl "/home/data/gello" driver tools 2>/dev/null | while read -r F; do
  sed -i 's#/home/data/gello/gello_software/ros2/src/franka_gello_state_publisher#<GELLO_WS>/src/franka_gello_state_publisher#g' "$F"
  sed -i 's#/home/data/gello/calib_final.json#<REPO>/config/calib_final.json#g' "$F"
  echo "  $F"
done

echo "=== syntax check ==="
FAIL=0
for F in driver/*.py tools/*.py; do
  python3 -m py_compile "$F" 2>/dev/null || { echo "  SYNTAX FAIL: $F"; FAIL=1; }
done
find . -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
[ "$FAIL" = 0 ] && echo "  all OK"

echo "=== leftover absolute paths (should be none) ==="
grep -rn "/home/data\|/home/pil\|/home/user" driver tools config 2>/dev/null || echo "  none"

echo
echo "done."
