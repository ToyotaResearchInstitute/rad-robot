icwd = os.getenv('PWD')
local init = require('init')

local unix = require('unix');
--local main = require('main');
local main_op = require('main_op');

while 1 do 
  tDelay = 0.005*1E6;
--  main.update();
  main_op.update();
  unix.usleep(tDelay);
end

