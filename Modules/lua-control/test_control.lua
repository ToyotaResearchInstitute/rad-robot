#! / usr / bin / env luajit
local unpack = unpack or require'table'.unpack

local DEBUG_ANNOUNCE = false
local RUN_SIMULATION = true
math.randomseed(123)
local nloops = 10e3 --5e3 --2000

local grid = require'grid'
local atan2 = require'math'.atan2 or require'math'.atan
local racecar = require'racecar'
local transform = require'transform'
local tf2D_inv = require'transform'.tf2D_inv
local usleep = require'unix'.usleep
local vector = require'vector'
local vpose = require'vector'.pose

-- Test the control library
local control = require'control'
local generate_control_points = require'control'.generate_control_points
local generate_path = require'control'.generate_path

-- For grid drawing
local function set_mid(map, idx) map[idx] = 127 end
local function set_quarter(map, idx) map[idx] = 63 end
-- Path increments: one inch
local ds = 0.10
-- Which path to simulate
-- local desired_path = 'polygon'
-- local desired_path = 'outerA'
-- local desired_path = 'outer'
local desired_path = 'outerB'

-- Holodeck Grid
local g_holo = assert(grid.new{
  scale = 0.01,
  xmin = 0, xmax = 4.5,
  ymin = -1, ymax = 6
})

local g_loop = assert(grid.new{
  scale = 0.01,
  xmin = 0, xmax = 4.5,
  ymin = -1, ymax = 6
})

local routes = {}
-- Inner and outer are merely two different lanes
-- On the same road
routes.inner = {
  vpose{1.5, -0.25, math.rad(90)},
  vpose{1.5, 4.5, math.rad(0)},
  vpose{3.5, 4.5, math.rad(270)},
  vpose{3.5, -0.25, math.rad(180)},
  turning_radius = 0.3,
  closed = true
}
routes.outer = {
  vpose{0.75, -0.75, math.rad(0)},
  vpose{4, -0.75, math.rad(90)},
  vpose{4, 5.75, math.rad(180)},
  vpose{0.75, 5.75, math.rad(270)},
  turning_radius = 0.3,
  closed = true
}
-- Smaller loops
----[[
routes.outerA = {
  vpose{0.75, -0.75, math.rad(0)},
  vpose{4, -0.75, math.rad(90)},
  vpose{4, 2.5, math.rad(180)},
  vpose{0.75, 2.5, math.rad(270)},
  turning_radius = 0.3,
  closed = true
}
--]]
----[[
routes.outerB = {
  vpose{4, 5.75, math.rad(180)},
  vpose{0.75, 5.75, math.rad(270)},
  vpose{0.75, 2.5, math.rad(0)},
  vpose{4, 2.5, math.rad(90)},
  turning_radius = 0.3,
  closed = true
}
--]]

--[[
routes.triangle = {
  vpose{1 + 0, 0, math.rad(0)},
  vpose{1 + 1, 0, math.rad(120)},
  vpose{1 + 0, 2, math.rad(270)},
  turning_radius = 0.1
}
 
routes.polygon = {
  vpose{0.125 + 0, 0, math.rad(0)},
  vpose{0.125 + 2, 0, math.rad(30)},
  vpose{0.125 + 2 + math.sqrt(3), 0 + 1, math.rad(90)},
  vpose{0.125 + 2 + math.sqrt(3), 0 + 1 + 2, math.rad(180)},
  vpose{0.125 + 0, 0 + 1 + 2, math.rad(270)},
  turning_radius = 0.25,
  closed = false
}
--]]

-- Generate the knots
-- These show the points before and after a turn
local route_knots = {}
for name, route in pairs(routes) do
  local knots = assert(generate_control_points(route))
  route_knots[name] = knots
end
-- Print the knots
for name, knots in pairs(route_knots) do
  print("Route knots", name)
  for i, kn in ipairs(knots) do print(i, unpack(kn)) end
end

-- Intersection lookup helper
local all_knots = {
  tree = require'kdtree'.create(2),
last = {n = 0}}
for name, knots in pairs(route_knots) do
  print(name, "n_knots", #knots)
  for i, knot in ipairs(knots) do
    local info = {name, i}
    table.insert(all_knots, info)
    -- Insert with the ID to the route information
    all_knots.tree:insert(knot, #all_knots)
  end
end

-- Go from a route to a list of points (path)
local paths = {}
for name, knots in pairs(route_knots) do
  g_holo:fill(0)
  local path, length = generate_path(knots, {
  ds = ds, grid_raster = g_holo, closed = routes[name].closed})
  assert(path, length)
  assert(#path > 0, "No points in path")
  assert(length > 0, "No path length")
  -- Since we are drawing, save the drawing of the path(s)
  assert(g_holo:save("/tmp/path_"..name..".pgm"))
  -- Add to the table of paths
  path.length = length
  paths[name] = path
  print(string.format("Route [%s] Length [%.2f meters] Points [%d]",
  name, path.length, #path))
end

-- Parameters for trajectory following
local my_path = paths[desired_path]
local my_speed = 0.1 -- meters per second
local dt = 0.1
local lookahead = 0.33
local threshold_close = 0.25

-- Draw the desired path with tolerences and print the coordinates
g_holo:fill(0)
-- Draw each point in the path as a circle
print("Desired path:", desired_path)
for _, pt in ipairs(my_path) do print(pt[1], pt[2], math.deg(pt[3])) end
-- Show the closeness for the nearby lookup
for _, pt in ipairs(my_path) do g_holo:circle(pt, threshold_close, set_quarter) end
-- Show the path waypoint intervals
for _, pt in ipairs(my_path) do g_holo:circle(pt, ds / 2, set_mid) end
g_holo:path(my_path)
assert(g_holo:save"/tmp/path.pgm")

if not RUN_SIMULATION then return end

local pp_params = {
  path = my_path,
  lookahead = lookahead,
  threshold_close = threshold_close
}
local pp = control.pure_pursuit(pp_params)

-- Simulation parameters
local dheading_max = math.rad(45) -- radians of front wheels
local wheel_base = 0.3

-- Initialize our simulated pose
local pose_rbt = vector.pose()
pose_rbt.x, pose_rbt.y, pose_rbt.a = unpack(my_path[1])
if type(pose_rbt.a) ~= 'number' then
  print("Taking direction for the next point...")
  pose_rbt.a = atan2(my_path[2][2] - pose_rbt.y, my_path[2][1] - pose_rbt.x)
end

-- Set the environment for displaying in-browser
local env = {
  viewBox = {g_holo.xmin, g_holo.ymin, g_holo.xmax, g_holo.ymax},
  observer = pose_rbt,
  time_interval = dt,
  speed = my_speed,
  -- Show the knots for better printing
  lanes = {route_knots.inner, route_knots.outer},
  trajectory_turn = {route_knots.outerA, route_knots.outerB},
}

local function simulate_vehicle(state, control_inp)
  local steering = control_inp.steering
  local speed = control_inp.speed
  local pose_x, pose_y, pose_a = unpack(state.pose)
  -- Kinematic model propagation
  steering = math.min(math.max(-dheading_max, steering), dheading_max)
  local dpose_a = (speed * dt) / wheel_base * math.tan(steering)
  local dpose_x, dpose_y = transform.rot2D(speed * dt, 0, pose_a)
  -- Add some noise
  local dnoise_x, dnoise_y = unpack(vector.randn(2, 0.01 * dt, 0))
  local dnoise_a = unpack(vector.randn(1, math.rad(1) * dt, 0))
  -- Give the new pose
  local pose1 = {
    pose_x + dpose_x + dnoise_x,
    pose_y + dpose_y + dnoise_y,
  transform.mod_angle(pose_a + dpose_a + dnoise_a)}
  -- Return the state
  return {
    pose = pose1
  }
end

-- Show how well the path performs
g_holo:fill(0)
-- Show the closeness for the nearby lookup
for _, pt in ipairs(my_path) do g_holo:circle(pt, threshold_close, set_quarter) end
-- Show the resolution of the steps
for _, pt in ipairs(my_path) do g_holo:circle(pt, ds / 2, set_mid) end

-- Let's write a ply for visualizing each loop
local p_history = {}
local loop_counter = 0
for iloop = 1, nloops do
  if not racecar.running then break end
  p_history[iloop] = {pose_rbt[1], pose_rbt[2], loop_counter}
  print("\n== Iteration", iloop)
  -- print(env)
  print("Pose", pose_rbt)
  print("desired_path", desired_path)
  -- Draw the point where we are
  g_holo:point(pose_rbt)
  g_loop:point(pose_rbt)
  
  local result, err = assert(pp(pose_rbt))
  if type(result) ~= 'table' then
    print("Improper", result, err)
    break
  elseif result.err then
    print("Err1", result.err)
    break
  end
  print("== Path result ==")
  print("Path size", #my_path)
  for k, v in pairs(result) do
    if type(v) == 'table' then
      print(k, unpack(v))
    else
      print(k, v)
    end
  end
  print("==")
  
  -- Intersection check
  local intersection_close = 2 * ds
  print("intersection_close", intersection_close)
  local upcoming_intersections = all_knots.tree:nearest(result.p_lookahead, intersection_close)
  -- Stay on the current route
  local intersection_routes = {}
  local n_intersection_routes = 0
  local has_intersection_choice = false
  -- TODO:
  intersection_routes[desired_path] = true
  table.insert(intersection_routes, desired_path)
  n_intersection_routes = n_intersection_routes + 1
  --
  if type(upcoming_intersections) == 'table' then
    print("Route | Lookahead:", unpack(result.p_lookahead))
    for _, upcoming in ipairs(upcoming_intersections) do
      
      local iknot = upcoming.user
      local name, idx = unpack(all_knots[iknot])
      local knot = route_knots[name][idx]
      
      -- TODO: Check distance and orientation
      local d_knot = vector.distance(knot, result.p_lookahead)
      local da_knot = transform.mod_angle(knot[3] - pose_rbt[3])
      print(string.format("Route | Nearby knot [%d]: %s[%d] %.2f away, %.2f deg",
      iknot, name, idx, d_knot, math.deg(da_knot)))
      if name == desired_path then
        print("Route | Desired is close", unpack(knot))
        -- Set the desired route knot point in this case
        assert(intersection_routes[name] == true)
        intersection_routes[name] = iknot
      elseif math.abs(da_knot) < math.rad(90) then
        --[[
        -- Check that these are unique
        print("Route | Check knot is close", unpack(knot))
        for name1, iknot1 in pairs(intersection_routes) do
          local name_idx = all_knots[iknot1]
          if name_idx then
            local knot1 = route_knots[name1][ name_idx[2] ]
            print("Comparing...", vector.new(knot), vector.new(knot1))
            if vector.eq(knot, knot1) then
              print("Route | is close!")
              break
            end
          end
        end
        --]]
        -- Add
        -- TODO: Don't duplicate knot points...
        intersection_routes[name] = iknot
        n_intersection_routes = n_intersection_routes + 1
        intersection_routes[n_intersection_routes] = name
        --
      end
    end
    assert(n_intersection_routes == #intersection_routes)
    print("selector0", table.concat(intersection_routes, ','))
    
    --
    if not vector.set_eq_set(intersection_routes, all_knots.last) then
      all_knots.last = intersection_routes
      has_intersection_choice = true
      print(iloop, "intersection_routes")
      for k, v in pairs(intersection_routes) do print(k, v) end
    end
  end
  -- upcoming_intersections = all_knots.tree:nearest(result.p_lookahead_path, threshold_close)
  
  -----------------------------
  -- Check if reaching the goal
  local n_path = #my_path
  local p_final = my_path[n_path]
  local x_rbt, y_rbt, th_rbt = unpack(pose_rbt)
  local px_final, py_final = unpack(p_final)
  local dx_final, dy_final = px_final - x_rbt, py_final - y_rbt
  local d_goal = math.sqrt(dx_final * dx_final + dy_final * dy_final)
  local done_path = false
  if result.id_lookahead >= n_path and d_goal <= ds then
    -- TODO: When within lookahead distance of the goal,
    -- Should there be a different behavior?
    -- i.e. a junction within the lookahead is important
    -- break
    done_path = true
  end
  -- End goal check
  -----------------------------
  
  -- Random routes
  -- print("Routes available", #intersection_knots)
  -- Choose a (random) new path
  if has_intersection_choice and #intersection_routes > 1 then
    local selector = math.random(#intersection_routes)
    print("selector", selector, table.concat(intersection_routes, ','))
    local desired_path1 = intersection_routes[selector]
    print("desired_path1", desired_path1)
    if desired_path1 ~= desired_path then
      assert(g_loop:save(string.format("/tmp/loop_%02d_%s.pgm", loop_counter, desired_path)))
      g_loop:fill(0)
      loop_counter = loop_counter + 1
      print(string.format("[%d] Route change! [%s] -> [%s]",
      iloop, desired_path, desired_path1), vector.pose(pose_rbt))
      desired_path = desired_path1
      my_path = paths[desired_path]
      pp_params.path = my_path
      pp = assert(control.pure_pursuit(pp_params))
    end
  elseif done_path and my_path.closed then
    -- Re-run this path
    print("Re-run path!")
    pp = control.pure_pursuit(pp_params)
  elseif done_path then
    break
  end
  
  -------------------------
  -- Run forward simulation
  -- Set the angle of the front wheels
  local steering = math.atan(result.kappa * wheel_base)
  print("Steering", math.deg(steering))
  local state = simulate_vehicle({pose = pose_rbt},
  {steering = steering, speed = my_speed})
  pose_rbt = vector.pose(state.pose)
  -- End forward simulation
  -------------------------
  
  -- Should we broadcast?
  if DEBUG_ANNOUNCE then
    env.observer = pose_rbt
    racecar.announce("risk", env)
    result.steering = steering
    result.velocity = my_speed
    racecar.announce("control", result)
    usleep(1e4)
  end
end
if DEBUG_ANNOUNCE then
  racecar.announce("control", {steering = 0, velocity = 0})
end
assert(g_holo:save"/tmp/simulated.pgm")
assert(g_loop:save(string.format("/tmp/loop%02d.pgm", loop_counter)))

print("p_history", #p_history)
assert(require'fileformats'.save_ply("/tmp/simulation.ply", p_history))

