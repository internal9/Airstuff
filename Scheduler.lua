local RS = game:GetService("RunService")

local Scheduler = {}



-- frame budgeting & 1 / hz effectively acts as a fixed delta time
function Scheduler.add(phase, hz, step_fn)
  local elapsed = 0
  local step_size = 1 / hz
  RS[phase]:Connect(function(delta_time)
    --elapsed += delta_time
      local t = os.clock()
      
  end)
end

return Scheduler
