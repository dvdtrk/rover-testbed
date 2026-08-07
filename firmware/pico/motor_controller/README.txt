
# Make sure transport header files are included

cp ~/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.c ~/firmware/pico/motor_controller/
cp ~/micro_ros_raspberrypi_pico_sdk/pico_uart_transport.h ~/firmware/pico/motor_controller/

# Build

cd build
cmake ..
make

# Flash

# Left:
sudo openocd -f ~/raspberrypi5-swd-left.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/blink/build/blink_left.elf verify reset exit"

# Right:
sudo openocd -f ~/raspberrypi5-swd-right.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/blink/build/blink_right.elf verify reset exit"

