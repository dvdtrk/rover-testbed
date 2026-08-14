
## First time only

cd ~/rover-testbed/firmware/pico/motor_controller
mkdir -p build && cd build
cmake ..

## Rebuild

cd ~/rover-testbed/firmware/pico/motor_controller/build
make -j$(nproc)

## Verify

ls *.elf

## Flash

# Left:
sudo openocd -f ~/raspberrypi5-swd-left.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/motor_controller/build/motor_controller_left.elf verify reset exit"

# Right:
sudo openocd -f ~/raspberrypi5-swd-right.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/motor_controller/build/motor_controller_right.elf verify reset exit"

## To run on single side (left or ride) (NO ROS)

sudo minicom -D /dev/ttyAMA2 -b 115200

and enter inputs
f = forward
b = backward
s = stop

## For full directional motor control (NO ROS)

python3 rover_control.py

use WASD keys to control

## Notes
- `cmake ..` only needed again if CMakeLists.txt changes
