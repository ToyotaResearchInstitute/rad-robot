#!/usr/bin/env luajit

local js = require'joystick'
js.open(arg[1])
while true do
  local axes = js.axis()
  print('Axes', unpack(axes))
  --
  local buttons = js.button()
  print('Buttons', unpack(buttons))
  --
  os.execute('sleep 0.1')
end

js.close()
