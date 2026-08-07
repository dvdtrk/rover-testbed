
# Make sure config is set up for openocd:

cat > ~/raspberrypi5-swd-left.cfg << 'EOF'
adapter driver linuxgpiod
adapter gpio swclk 17 -chip 0
adapter gpio swdio 16 -chip 0
transport select swd
EOF

cat > ~/raspberrypi5-swd-right.cfg << 'EOF'
adapter driver linuxgpiod
adapter gpio swclk 19 -chip 0
adapter gpio swdio 18 -chip 0
transport select swd
EOF


# To build:

cd build
cmake ..
make -j$(nproc)


## FLASHING:

# Left motor:

sudo openocd -f ~/raspberrypi5-swd-left.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/blink/build/blink_left.elf verify reset exit"

# Right motor:

sudo openocd -f ~/raspberrypi5-swd-right.cfg -f target/rp2040.cfg \
  -c "program $HOME/rover-testbed/firmware/pico/blink/build/blink_right.elf verify reset exit"

