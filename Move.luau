-- TO BE revamped to allow 360 vertical turns
--!optimize 2
local Grid3D = require("../Modules/Grid3D")
local Physics = require("../Modules/Physics")
local WorldInfo = require("../Info/WorldInfo")
local Constants = require("../Info/Constants")
local PlaneInfos = require("../Info/PlaneInfos")

local grid = WorldInfo.grid
local world = WorldInfo.world
local components = WorldInfo.components
local tags = WorldInfo.tags

-- phase out  'GoalDirection' later for 'GoalVelocity'? unit might want to have a target speed too (via 'UnitThrottle')
local planes = world:query(components.PlaneModel, components.GoalDirection):cached()

local function crash_effect(plane_model)
	local explosion = Instance.new("Explosion")
	explosion.TimeScale = 0.5
	explosion.ExplosionType = Enum.ExplosionType.NoCraters
	explosion.Position = plane_model.Position
	explosion.Parent = plane_model
end

local function aim_amount(unit_dir: Vector3, unit_pos: Vector3, target_pos: Vector3)
	return unit_dir:Dot((target_pos - unit_pos).Unit)
end

-- approximate
local function turn_radius(speed, turn_rate)
	return speed / turn_rate
end

-- [-math.pi, math.pi]
local function yaw_pi_range(radians: number)
	if radians > math.pi then
		return radians - math.pi * 2
	elseif radians < -math.pi then
		return radians + math.pi * 2
	end
	return radians
end

local function rotate_vector3_around(vector3, radians, axis)
	return CFrame.fromAxisAngle(axis, radians):VectorToWorldSpace(vector3)
end

local function get_goal_cf(plane: MeshPart, goal_dir: Vector3, delta_time)--, delta_time: number)
	local plane_info = PlaneInfos[plane.Name]
	local start_dir = plane.CFrame.LookVector 

	-- clamp 'dir' based off turn rate
	local yaw_diff = yaw_pi_range(
		math.atan2(-goal_dir.X, -goal_dir.Z) -
			math.atan2(-start_dir.X, -start_dir.Z)
	)

	local pitch_diff =
		math.atan2(goal_dir.Y, math.sqrt(goal_dir.X^2 + goal_dir.Z^2)) -
		math.atan2(start_dir.Y, math.sqrt(start_dir.X^2 + start_dir.Z^2))

	local yaw = math.clamp(yaw_diff, -plane_info.max_turn_rate, plane_info.max_turn_rate)
	local pitch = math.clamp(pitch_diff, -plane_info.max_turn_rate, plane_info.max_turn_rate)
	goal_dir = rotate_vector3_around(start_dir, yaw, Vector3.yAxis)

	local pitch_axis = goal_dir:Cross(Vector3.yAxis)
	pitch_axis = pitch_axis ~= Vector3.zero and pitch_axis.Unit or Vector3.xAxis
	goal_dir = rotate_vector3_around(goal_dir, pitch * delta_time, pitch_axis * delta_time)

	local old_roll_angle = plane.Orientation.Z
	local roll_angle =
		math.atan(
			(plane_info.speed ^ 2) /
			(0.35 * turn_radius(plane_info.speed, math.clamp(yaw_diff, -plane_info.max_turn_rate, plane_info.max_turn_rate)))
		)

	local step_roll_angle_change =
		math.clamp(
			roll_angle - math.rad(old_roll_angle),
			-plane_info.max_roll_rate, plane_info.max_roll_rate
		)

	roll_angle =
		math.rad(old_roll_angle) +
		step_roll_angle_change * delta_time

	local pos_change = (goal_dir * plane_info.speed)
	
	local goal_cf = CFrame.lookAt(plane.Position, plane.Position + goal_dir)-- * CFrame.Angles(0, 0, roll_angle)
	return goal_cf	-- timestamp of 1s
end

-- simply move to velocity's normal vector (direction), don't confuse with in direction of a position or wahtever
local function step_move(plane: MeshPart, scaled_vel: Vector3, scaled_angular_vel: Vector3, delta_time: number)
--	plane.Position += scaled_vel * delta_time
--	plane.Orientation += scaled_angular_vel * 180/math.pi * delta_time
--	warn(scaled_angular_vel)

	local new_x, new_y, new_z = Grid3D.to_cell_coords(grid.cell_size, plane.Position)
	return new_x, new_y, new_z, plane.Position.Y <= 1	-- grid is 1 based index, so ground = 1 instead of 0
end

 
 -- get 
 -- make nose point towards velocity too ig
 -- lift only generated from main wing, so velocity and direction won't match as rudder has no simulation
-- clamp plane position to stay in grid, like absolute value or something
-- this handles downed planes too, which stay in grid so others can try avoiding their falling wreck
local function step(delta_time: number)
	require("../Modules/Vector3Visualizer").clear_visual_parts()
	
	for id, plane_model, goal_direction in planes do
		--goal_direction = rotate_vector3_around(Vector3.zAxis, math.rad(45), Vector3.yAxis)
		goal_direction = Vector3.zAxis
		
		local x1, y1, z1 = Grid3D.to_cell_coords(grid.cell_size, plane_model.Position)
		
		-- instead of flaps, just clamp the goal dir into a cframe and calculate using that
		local current_vel = world:get(id, components.WorldVelocity)
		local current_angular_vel = world:get(id, components.WorldAngularVelocity)
		
		local goal_cf = get_goal_cf(plane_model, goal_direction, delta_time)
		
		local new_vel, new_angular_vel = Physics.update_movement(
			current_vel,
			current_angular_vel,
			plane_model.CFrame,
			plane_model.Name, 
			1,
			delta_time
		)
		
		--require("../Modules/Vector3Visualizer").visualize(
		--	plane_model.Position, current_vel.Unit * 10, Color3.new(1, 1, 1)
		--)
		
		--world:set(id, components.WorldVelocity, new_vel)
		--world:set(id, components.WorldAngularVelocity, new_angular_vel)
		
		local scaled_world_new_vel = new_vel * Constants.METER_TO_STUD_SCALED
		local scaled_world_new_angular_vel = new_angular_vel * Constants.METER_TO_STUD_SCALED
		
		local x2, y2, z2, into_ground = step_move(
			plane_model,
			scaled_world_new_vel,
			scaled_world_new_angular_vel,
			delta_time
		)
		
		if true then continue end
--		print((new_vel - current_vel).Magnitude / delta_time)
	
--		print(new_vel.Magnitude)
		if plane_model.Position.Y > 250 then
			print(plane_model.Position.Y)
		end
		
		if into_ground then
			-- batch?
			xpcall(function()
				Grid3D.cell_value_remove(grid, x1, y1, z1,  plane_model)
			end, function()
				print("IDK", id, x1, y1, z1)
			end)
			--Grid3D.cell_value_remove(grid, x1, y1, z1,  plane_model)

			crash_effect(plane_model)
			plane_model:SetAttribute("ID", nil)
			world:delete(id)
		else	
			if x2 ~= x1 or y2 ~= y1 or z2 ~= z1 then
				Grid3D.cell_value_remove(grid, x1, y1, z1,  plane_model)
				Grid3D.cell_add(grid, x2, y2, z2,  plane_model)
			end
		end
	end
end

local function init()
	for id, plane_model in planes:iter() do
		local x, y, z = Grid3D.to_cell_coords(grid.cell_size, plane_model.Position)
		Grid3D.cell_add(grid, x, y, z,  plane_model)
		
--		local vel = Vector3.zero --plane_model.CFrame.LookVector * 50
--		print(vel)
--		print(Physics.get_velocity(vel, plane_model.CFrame, plane_model.Name, 1))
	end
end

return {init = init, step = step}
