#!/usr/bin/env luajit

math.randomseed(123)
local rrt = require'rrt'

local tree = rrt.new{
  {-1, 1}, {-1,1}
}

tree:set_system{
  is_collision = function(self, s)
    if self:distState(s, {0,0}) < 0.25 then
      return true
    end
    return false
  end
}

-- Go from 0,0 to 1,1
tree:plan({-0.9,-0.9}, {0.9, 0.9})

-- Periodically, check to find the best trajectory
-- Should do this within a computational bound
for i=1,10e3 do
  tree:iterate()
  if i>1e3 and tree.goal.parent then
    print(string.format("Iteration %d | Cost: %f",
                        i, tree.lowerBoundCost))
    break
  elseif i % 100 == 0 then
    print(string.format("Iteration %d | Cost: %f",
                        i, tree.goal.parent and tree.lowerBoundCost or 0/0))
  end
  -- for _, vertex in ipairs(tree) do print(vertex.id, unpack(vertex)) end
end

if not tree.goal.parent then return end

print("Node path from goal to start")
local cur = tree.goal
repeat
  print(cur.id, tree:distState(cur, {0,0}), unpack(cur))
  cur = cur.parent
until not cur