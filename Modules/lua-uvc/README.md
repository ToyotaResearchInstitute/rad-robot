sudo nano /etc/udev/rules.d/99-cameras.rules

SUBSYSTEMS=="usb", ATTRS{idVendor}=="046d", ATTRS{idProduct}=="0843", MODE="0666"
