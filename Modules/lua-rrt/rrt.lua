-- Rapidly-exploring Random Trees
local lib = {}
local kdtree = require'kdtree'
local unpack = unpack or require'table'.unpack

local function get_ball_radius(numVertices, nDim, gamma)
  gamma = gamma or 1
  return gamma * math.pow(math.log(numVertices + 1.0)/(numVertices + 1.0), 1.0/nDim)
end

local function sample(self)
  local out = {}
  local bias = math.random()
  if bias < self.BIAS_THRESHOLD then
    for i,v in ipairs(self.intervals) do
      local a, b = unpack(v)
      local perturbation = (2*math.random() - 1) * self.GOAL_PERTURBATION
      out[i] = math.max(a, math.min(b, self.goal[i] + perturbation))
    end
  else
    for i, c in ipairs(self.centers) do
      -- Interval [-2, 1] has ranges of 3, center of -0.5
      out[i] = (math.random() - 0.5) * self.ranges[i] + c
    end
  end
  return out
end

local function plan(self, start, goal)
  -- Clear the tree to prepare
  self.kd:clear()
  self.tree = {}
  -- Add the starting state
  self.start = {
    parent=nil,
    costFromRoot = 0,
    costFromParent = 0,
    id=1,
    unpack(start, 1, self.nDim)
  }
  -- Add the goal state
  self.goal = {
    parent = nil, -- lowerBoundVertex
    costFromRoot = math.huge, -- lowerBoundCost
    costFromParent = math.huge,
    id=0,
    unpack(goal, 1, self.nDim)
  }
  -- Add the start state to the kd-tree and rrt
  self.kd:insert(self.start, self.start.id)
  self.tree[self.start.id] = self.start
  -- Do not add the goal to the kd-tree
end

-- Iterate the dimensions
-- Default distance
-- Return the distance and each dimensions' distances
local function distState0(self, from, to)
  local diff = {}
  for i, f in ipairs(from) do diff[i] = to[i] - f end
  local s = 0
  for _, d in ipairs(diff) do s = s + math.pow(d, 2) end
  return math.sqrt(s), diff
end

local function is_near_goal0(self, state)
  return self:distState(state, self.goal) < self.CLOSE_DISTANCE
end

-- Walk from state to goal
-- Returns boolean idicating if we got to the goal state (exactConnection)
-- and the final state that we did get to,
-- walking in one direction to the goal
local function extend0(self, state, target)
  -- Assume that both to and from are collision-free
  local cost, dists = self:distState(state, target)
  local incrementTotal = cost / self.DISCRETIZATION_STEP
  local numSegments = math.floor(incrementTotal)
  -- print("numSegments", numSegments)
  -- // normalize the distance according to the discretization step
  for i=1,#dists do dists[i] = dists[i] / incrementTotal end
  -- print("dists", unpack(dists))
  local cur = {unpack(state)}
  for _=1,numSegments do
    -- print("cur", unpack(cur))
    if self:is_collision(cur) then return false, cur end
    -- Walk a step towards the target
    for i, d in ipairs(dists) do cur[i] = cur[i] + d end
  end
  return true, target, cost
end

-- System comparison and local kinematics/dynamics paths
local function set_system(self, funcs)
  if type(funcs)~='table' then return false end
  -- Distance between two states
  if type(funcs.distance)=='function' then
    self.distState = funcs.distance
  end
  -- Cost to go from one state to the next
  -- TODO: Just an output from extend?
  if type(funcs.evaluateCostToGo)=='function' then
    self.evaluateCostToGo = funcs.evaluateCostToGo
  end
  -- Is a state colliding with an obstacle?
  if type(funcs.is_collision)=='function' then
    self.is_collision = funcs.is_collision
  end
  -- How to move from one state to the next (we may not get there, but just close!)
  if type(funcs.extend)=='function' then
    self.extend = funcs.extend
  end
  if type(funcs.is_near_goal)=='function' then
    self.is_near_goal = funcs.is_near_goal
  end
  return true
end

-- Returns the best candidate if we can connect, else false
local function findBestCandidate(self, state, neighbors)
  -- print("stateRandom", unpack(state))
  -- Which neighbor should be our parent?
  local candidates = {}
  for _, neighbor in ipairs(neighbors) do
    local costFromParent = self:distState(neighbor, state)
    local costFromRoot = neighbor.costFromRoot + costFromParent
    local parent = self.tree[neighbor.id]
    -- print("neighbor", unpack(neighbor))
    if costFromParent>0 then
      table.insert(candidates, {
        parent = parent,
        costFromRoot = costFromRoot,
        costFromParent = costFromParent,
        unpack(state)
      })
    end
  end
  if #candidates == 0 then return false end
  -- Sort by costFromRoot
  table.sort(candidates, function(a, b) return a.costFromRoot < b.costFromRoot end)
  -- Now, try to extend this state to the best neighbor
  for _, c in ipairs(candidates) do
    local is_collision_free = self:extend(c.parent, state)
    if is_collision_free then
      return c
    end
  end
  return false
end

local function rewire(self, neighbors, vertexNew)
  for _, neighbor in ipairs(neighbors) do
    local is_collision_free, _, cost = self:extend(vertexNew, neighbor)
    if is_collision_free then
      -- Now, check if actually a better way to reach neightbor
      local totalCost = vertexNew.costFromRoot + cost
      if totalCost < (neighbor.costFromRoot - self.EPS_REWIRE) then
        -- Rewire the neighbor to come from the new node,
        -- rather than its previous parent
        neighbor.parent = vertexNew
        neighbor.costFromRoot = totalCost
        neighbor.costFromParent = cost
        -- TODO: Set the cost to the goal...
        if self:is_near_goal(neighbor) and neighbor.costFromRoot < self.lowerBoundCost then
          self.goal.parent = neighbor
          self.lowerBoundCost = neighbor.costFromRoot
        end
      end
    end
  end
end

local function iterate(self)
  -- // 1. Sample a new state
  local stateRandom
  local n_sample_tries = 0
  repeat
    if n_sample_tries>100 then
      return false, "Number of sample tries exceeded!"
    end
    stateRandom = sample(self)
    n_sample_tries = n_sample_tries + 1
  until not self:is_collision(stateRandom)

  -- // 2. Compute the set of all near vertices
  local numVertices = self.kd:size()
  -- // 3.b Extend the best parent within the near vertices
  local r = get_ball_radius(numVertices, self.nDim)
  local nearby = self.kd:nearest(stateRandom, r)
  local within_ball = type(nearby) == 'table'
  -- // 3.a Extend the nearest
  nearby = within_ball and nearby or self.kd:nearest(stateRandom)
  -- Link the kd-tree output the node variables
  for i=1,#nearby do
    local id = nearby[i].user
    nearby[i] = self.tree[id]
  end
  -- // 3. Find the best parent and extend from that parent
  local candidate = findBestCandidate(self, stateRandom, nearby)
  if not candidate then return false, "No valid candidates" end

  -- // Check for admissible cost-to-go until the goal
  -- This is the A* bit: Ignore samples that aren't guided
  if self.goal.parent then -- lowerBoundVertex
    local costToGo = self:evaluateCostToGo(candidate)
    if costToGo >= 0 then
      -- > lowerBoundCost
      if (candidate.costFromRoot + costToGo) > self.lowerBoundCost then
        return false, "Short circuit"
      end
    end
  end

  -- // 3.c add the trajectory from the best parent to the tree
  -- TODO: Should have an "add_candidate" function for safety
  -- and consistency b/t kd tree and rrt tree
  -- TODO: #tree should equal numVertices
  candidate.id = #self.tree + 1
  self.kd:insert(candidate, candidate.id)
  self.tree[candidate.id] = candidate
  -- Check if near the goal
  if self:is_near_goal(candidate) and candidate.costFromRoot < self.lowerBoundCost then
    -- print("Found nearby candidate!", candidate.costFromRoot, self.lowerBoundCost)
    self.lowerBoundCost = candidate.costFromRoot
    self.goal.parent = candidate
  end

  -- // 4. Rewire the tree
  if within_ball then
    -- The new node may be a means to getting to its neighbors more quickly
    -- Thus, check changing each neightbors' parent to the new node
    rewire(self, nearby, candidate)
  end
  return true
end

-- Input: list of min and max for each dimension {{a,b},{c,d},...}
function lib.new(intervals)
  local nDim = #intervals
  local ranges = {}
  local centers = {}
  for _,v in ipairs(intervals) do
    local a, b = unpack(v)
    table.insert(centers, (a + b)/2)
    table.insert(ranges, b - a)
  end
  -- Rapidly-exploring Random Tree
  -- List of the vertices. Don't touch ;)
  -- Hash table of methods and properties
  local tree = {
    nDim = nDim,
    intervals = intervals,
    centers = centers,
    ranges = ranges,
    -- Set the system functions
    set_system = set_system,
    distState = distState0,
    extend = extend0,
    evaluateCostToGo = function(self, s) return self:distState(s, self.goal) end,
    is_collision = function() return false end,
    is_near_goal = is_near_goal0,
    -- Our path result
    start = nil,
    goal = nil,
    lowerBoundCost = math.huge,
    -- k-d tree for fast lookup
    kd = kdtree.create(nDim),
    -- Plan from start to goal
    plan = plan,
    -- Iterate the plan,
    iterate = iterate,
    -- Adjustable parameters
    BIAS_THRESHOLD = 0.1, -- Fraction
    GOAL_PERTURBATION = 0.1,
    CLOSE_DISTANCE = 0.01,
    DISCRETIZATION_STEP = 0.005, -- TODO: Unit
    EPS_REWIRE = 0, --0.001
  }

  -- Return the object
  return tree
end

return lib
