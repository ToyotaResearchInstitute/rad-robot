local Body = require'Body'

local t_entry, t_update

local state = {}
state._NAME = 'lidarIdle'

function state.entry()
  print(state._NAME..' Entry' ) 
    -- Update the time of entry
  local t_entry_prev = t_entry -- When entry was previously called
  t_entry = Body.get_time()
  t_update = t_entry
end

function state.update()
--  print(_NAME..' Update' )
  -- Get the time of update
  local t = Body.get_time()
  local t_diff = t - t_update
  -- Save this at the last update time
  t_update = t
end

function state.exit()
  print(state._NAME..' Exit' ) 
end

return state