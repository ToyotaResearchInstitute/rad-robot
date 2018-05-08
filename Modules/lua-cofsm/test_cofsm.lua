#!/usr/bin/env lua5.1
local fsm = require'cofsm'

local function botGo()
  print('Entry', 'ok')
  local distance = 0
  local evt = false
  while coroutine.yield(evt) do
    print('Update', 'go', distance)
    distance = distance + 1
    -- error('oops')
    evt = distance > 3 and 'stop'
  end
  print('Exit', 'woo')
  return 'bye'
end
local function botStop()
  print('Entry', 'eek')
  local distance = 3
  local evt = false
  while coroutine.yield(evt) do
    print('Update', 'whoa', distance)
    distance = distance - 1
    evt = distance <= 0 and 'go'
  end
  print('Exit', 'phew')
  return 'hi'
end

print("init...")
local botFsm = fsm.new{
  {'botStop', botStop},
  {'botStop', 'go', 'botGo'},
  {'botGo', 'stop', 'botStop'},
}

-- Update the coroutines
if botFsm.add_coro and true then
  print("add coro...")
  botFsm:add_coro('botGo', botGo)
end

-- Add a new state
print("add state...")
botFsm:add_state('botThink')

print("add transitions...")
botFsm:add_transition('botStop', 'think', 'botThink')
botFsm:add_transition('botThink', 'done', 'botStop')

print('===')
print('Simple demo')
print('Initial State:', botFsm.current_state)
botFsm:enter('botStop')
local status, evt
for i=1,20 do
  print(i, 'Current State:', botFsm.current_state)
  status, evt = botFsm:update()
  if i == 8 then
    botFsm:dispatch('go')
  else
    botFsm:dispatch(evt)
  end
  print("Status/event", status, evt)
end
print('Final State:', botFsm.current_state)
print('===')
print()

print('Current State:', botFsm.current_state)
for i=1, 5 do
  print('- call', i, botFsm:update())
end
print('! Trigger', 'go', botFsm:dispatch'go')
print('! Trigger', 'unknown', botFsm:dispatch'go')
print('* Current State:', botFsm.current_state)
for i=1, 5 do
  print('- call', i, botFsm:update())
end
print('! Trigger', 'go', botFsm:dispatch'go')
print('! Trigger', 'stop', botFsm:dispatch'stop')
print('* Current State:', botFsm.current_state)
for i=1, 5 do
  print('- call', i, botFsm:update())
end
print('! Trigger', 'think', botFsm:dispatch'think')
print('* Current State:', botFsm.current_state)
for i=1, 5 do
  print('- call', i, botFsm:update())
end
print('! Trigger', 'done', botFsm:dispatch'done')
for i=1, 5 do
  print('- call', i, botFsm:update())
end