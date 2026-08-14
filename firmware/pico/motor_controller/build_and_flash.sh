
#!/bin/bash
set -e # stop on errors

sudo -v # prompt for password once

# keep password until this script exits
while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done 2>/dev/null & 
KEEPALIVE_PID=$!
trap 'kill "$KEEPALIVE_PID" 2>/dev/null' EXIT

echo "=== Configuring ==="
cd ~/rover-testbed/firmware/pico/motor_controller
rm -rf build && mkdir build && cd build
cmake ..
echo "=== Building firmware ==="
make -j$(nproc)
echo "=== Verifying build output ==="
ls motor_controller_left.elf motor_controller_right.elf
echo "=== Flashing left Pico ==="
sudo openocd -f ~/raspberrypi5-swd-left.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/motor_controller/build/motor_controller_left.elf verify reset exit"
echo "=== Flashing right Pico ==="
sudo openocd -f ~/raspberrypi5-swd-right.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/motor_controller/build/motor_controller_right.elf verify reset exit"
echo "=== Done ==="

# To make this executable:
# chmod +x ~/rover-testbed/firmware/pico/motor_controller/build_and_flash.sh
