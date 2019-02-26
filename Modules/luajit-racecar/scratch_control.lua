-- Purely simulation below
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

end