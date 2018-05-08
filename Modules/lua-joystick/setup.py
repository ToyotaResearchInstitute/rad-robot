#!/usr/bin/env python
from distutils.core import setup, Extension

joystick_module = Extension(
    'joystick',
    sources = ['py_joystick.c'],
    libraries=['usb-1.0', 'pthread'],
    include_dirs=['/usr/include/libusb-1.0']
    )

setup(
    name = 'joystick',
    version = '1.0',
    description = 'This provides joystick access on Linux',
    ext_modules = [joystick_module]
    )
