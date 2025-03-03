sudo avrdude -C ./avrdude.conf -v -p atmega328p -c avrisp -P /dev/ttyACM0 -U flash:w:./debug/main.hex:i
