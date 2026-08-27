local RS = game:GetService("RunService")

local Scheduler = {}

local events = {}

function Scheduler.add(phase, hz, step_fn)
  RS[phase]:Connect(
end

return Scheduler
