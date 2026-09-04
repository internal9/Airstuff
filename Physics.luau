--[[
	MAYBE ACCOUNT FOR ROBLOX AUTP HYSICS THROTTLING!! (AUTO CHANGES FRAMERATE)
	MORE CONTROL 
]]
--!optimize 2

local PlaneInfos = require("../Info/[INTERMEDIATE PHYSICS] PlaneInfos")

local Physics = {}

--[[
	Real-world SI units:
		mass        = kg
		distance    = m
		speed       = m/s
		force       = N
		acceleration= m/s^2
		angles      = radians

	Roblox coordinate convention:
		LookVector  = forward / -Z
		RightVector = +X
		UpVector    = +Y

	SIMPLIFICATIONS:
		- Plane mass & CoM fixed, ignores fuel changes & external stuff
		- No control surfaces
		- Only aerodynamics for wings (no fuselage)
]]

local GRAVITY_ACCEL = Vector3.yAxis * -9.81
local AIR_DENSITY = 1.225
local EPSILON = 1e-6

-- Approximate sea-level speed of sound.
-- This is intentionally constant for this simplified model.
--local SPEED_OF_SOUND = 343

-- ============================================================
-- VALIDATION
-- ============================================================

local function assert_finite(value: number, name: string)
	assert(value == value, `{name} is NaN`)
	assert(value ~= math.huge and value ~= -math.huge, `{name} is infinite`)
end

local function assert_vector_finite(value: Vector3, name: string)
	assert_finite(value.X, `{name}.X`)
	assert_finite(value.Y, `{name}.Y`)
	assert_finite(value.Z, `{name}.Z`)
end

--[[
	NOTES:
		- AoA is just angle between wing's chord line & relative wind (FOR SIMPLICITY, the wing's forward dir is treated
		as the chord line), it accounts for wing's local rotated cf to plane,
		HOWEVER, wings are treated as one aerodynamic object (FOR SIMPLICITY), instead of being segmented,
		so the shape of the wing (e.g. swept / delta wing) does not change CF's rotation,
		Rotation ONLY depends on context dihedral / anhedral, OR if it's a vertical stabilizer, the angle from the vertical (90 - angle)
		
		- e.g. Sideslip angle is just AoA of vertical stabilizer accounting for wing rotation relative to plane
			- e.g. vertical stabilizer has a local CF behind CoM, which is rotated 90 degrees, since the
				vertical stabilizer is basically just a wing rotated 90 degrees
			
		- Thus, allows for anhedral / dihedral simulation & wing angle imbalances
	
	SIMPLIFICATIONS:
		- Relative wind is simplified to flight_dir, no external weather wind whatsoever,
		BUT similarly to relative wind, opposes flight path
]]
local function get_AoA(relative_wind_local: Vector3)
	return math.atan2(relative_wind_local.Y, relative_wind_local.Z)
end

-- ============================================================
-- LIFT COEFFICIENT
-- ============================================================
local function get_dynamic_pressure(speed: number): number
	return 0.5 * AIR_DENSITY * (speed * speed)
end

local function get_induced_drag_coeff(lift_coeff: number, aspect_ratio: number, oswald_eff_factor: number): number
	if aspect_ratio <= EPSILON or oswald_eff_factor <= EPSILON then return 0 end
	return (lift_coeff * lift_coeff) / (math.pi * oswald_eff_factor * aspect_ratio)
end

local function get_lift(lift_coeff: number, dynamic_pressure: number, ref_area: number): number
	return dynamic_pressure * ref_area * lift_coeff
end

local function get_drag(
	parasitic_drag_coeff: number,
	lift_coeff: number,
	dynamic_pressure: number,
	ref_area: number,
	aspect_ratio: number,
	oswald_eff_factor: number
)
	
	local total_drag_coeff = parasitic_drag_coeff + get_induced_drag_coeff(lift_coeff, aspect_ratio, oswald_eff_factor)
	return dynamic_pressure * ref_area * total_drag_coeff
end

local function get_lift_coeff(AoA: number, lift_coeff_vs_AoA_curve: FloatCurve): number
	-- FloatCurve is periodic conceptually in the intended model,
	-- but only evaluate the aerodynamic range we actually support.
	return lift_coeff_vs_AoA_curve:GetValueAtTime(AoA)
end

local SPEED_OF_SOUND = 5000

-- Placeholder for future transonic/supersonic modelling.
local function get_wave_drag_coeff(speed: number): number
	local mach = speed / SPEED_OF_SOUND

	if mach < 0.8 then
		return 0
	end

	if mach < 1.2 then
		-- Simple smooth transonic rise.
		local t = (mach - 0.8) / 0.4
		t = t * t * (3 - 2 * t)

		return 0.02 * t
	end

	-- Simple supersonic approximation.
	return 0.02 / math.sqrt(mach)
end

local function get_thrust(max_thrust: number, unit_throttle: number): number
	unit_throttle = math.clamp(unit_throttle, 0, 1)
	-- Quadratic throttle -> thrust approximation.
	return max_thrust * (unit_throttle * unit_throttle)
end

local function get_mach(speed: number): number
	return speed / SPEED_OF_SOUND
end

-- ============================================================
-- DEBUG VISUALIZATION
-- ============================================================

local Vector3Visualizer = require("../Modules/Vector3Visualizer")

local function visualize_force(
	position: Vector3,
	force: Vector3,
	color: Color3
)
	Vector3Visualizer.visualize(
		position,
		force,
		color
	)
end

-- TODO (?): Apply force & torque THEN get linear & angular acceleration for less calculations

local function box_inertia_tensor(mass: number, size: Vector3)
	local x = size.X
	local y = size.Y
	local z = size.Z

	local Ixx = mass / 12 * (y*y + z*z)
	local Iyy = mass / 12 * (x*x + z*z)
	local Izz = mass / 12 * (x*x + y*y)

	return Vector3.new(Ixx, Iyy, Izz)
end


--[[
	- Figure if vel & angular vel need be reset after each loop step, prevent wrong accumulation
	- Maybe add basic flaps simulation (flap_ratio, no physical model)?
	
	- local_CoA_cf, positive +Z positional component is backwards from CoA, as -Z is lookVector / forwards dir 
]]
local function apply_wing_aerodynamics(
	plane_cf: CFrame,
	wing_info: PlaneInfos.WingInfo,
	world_current_linear_vel: Vector3
): (Vector3, Vector3)
	
	local wing_cf = plane_cf * wing_info.local_CoA_cf
	--local speed = world_current_linear_vel.Magnitude

	-- Relative wind.

	-- Aircraft velocity points in the direction the aircraft is moving.
	-- Aerodynamic drag opposes this, so use -velocity as the incoming
	-- relative wind direction.
	local relative_wind_world = -world_current_linear_vel
	local relative_wind_local = wing_cf:VectorToObjectSpace(relative_wind_world)
	local local_speed = relative_wind_local.Magnitude
	
	-- Simplified AoA
	-- The wing's local +Z is treated as the chord direction.
	
	-- Depending on how your actual wing CFrames are authored, you may
	-- need to flip the sign here.
	local AoA = math.clamp(get_AoA(relative_wind_local.Unit), wing_info.min_aoa, wing_info.max_aoa)
	
	-- temp
	print(math.deg(AoA))

	local dynamic_pressure = get_dynamic_pressure(local_speed)
	local lift_coeff = get_lift_coeff(AoA, wing_info.lift_coeff_vs_AoA_curve)

	-- Drag opposes aircraft velocity.
	local local_drag_dir = -relative_wind_local.Unit

	-- Lift is perpendicular to the relative wind and the wing's
	-- lift plane.
	--
	
	local local_wing_up = wing_info.local_CoA_cf.UpVector
	local local_lift_dir = local_wing_up - local_drag_dir * local_wing_up:Dot(local_drag_dir)
--	local lift_dir = wing_up - drag_dir.Unit * wing_up:Dot(drag_dir)

	--if lift_dir_magnitude <= EPSILON then
	--	return Vector3.zero, Vector3.zero
	--end

	local lift = get_lift(lift_coeff, dynamic_pressure, wing_info.area)
	local drag = get_drag(
		wing_info.parasitic_drag_coeff,
		lift_coeff,
		dynamic_pressure,
		wing_info.area,
		wing_info.aspect_ratio,
		wing_info.oswald_eff_factor
	)
	-- no wave drag 

	-- World-space forces
	local local_lift_force = local_lift_dir * lift
	local local_drag_force = local_drag_dir * drag
	local total_local_force = local_lift_force + local_drag_force
	local total_local_torque = plane_cf.Position:Cross(total_local_force)
	--local total_force_world = lift_force + drag_force

	-- local_CoA_cf.Position is relative to aircraft CoM.
	--local offset_world = plane_cf:VectorToWorldSpace(wing_info.local_CoA_cf.Position)
--	local torque_world = offset_world:Cross(total_force_world)
	
	
	-- Debug
	visualize_force(plane_cf.Position, wing_cf:VectorToWorldSpace(local_lift_force.Unit) * 5, Color3.fromRGB(0, 255, 0))
	visualize_force(plane_cf.Position, wing_cf:VectorToWorldSpace(-local_drag_force.Unit) * 5, Color3.fromRGB(255, 0, 0))

	return total_local_force, total_local_torque
end

--[[
	TEMP:
]]
function Physics.update_movement(
	current_linear_vel: Vector3,
	current_angular_vel: Vector3,
	plane_cf: CFrame,
	--goal_dir: Vector3,	-- treated like an instant cf after rotation, replacing flaps
	plane_name: string,
	unit_throttle: number,
	delta_time: number
): (Vector3, Vector3)
	
	local new_linear_vel, new_angular_vel = Vector3.zero, Vector3.zero
	local total_force, total_torque = Vector3.zero, Vector3.zero
	
	local physics_info = PlaneInfos.physics[plane_name]
	local mass = physics_info.mass
	
	unit_throttle = math.clamp(unit_throttle, 0, 1)
	-- goal_dir will be reached via angular velocity (hopefully)
	local thrust_force = plane_cf.LookVector * physics_info.max_thrust * (unit_throttle * unit_throttle)
	local weight_force = GRAVITY_ACCEL * mass

	total_force += thrust_force + weight_force
	
	-- Avoid divide by 0 errors for aerodynamic forces calculations.. anddd would probably be neglible anyways
	if current_linear_vel.Magnitude <= EPSILON then
		new_linear_vel = current_linear_vel + (total_force / mass) * delta_time
		-- no change in torque since only weight & thrust are simplified to apply directly at CoG
		return new_linear_vel, current_angular_vel
	end
	
	for _, wing_info in physics_info.wings do
		-- TEMP
		if wing_info._disabled == true then continue end
		
		-- accumulate force & torques local to each wing
		local local_wing_force, local_wing_torque = apply_wing_aerodynamics(plane_cf, wing_info, current_linear_vel)
		
		--total_force += wing_force
		--total_torque += wing_torque
	end
	
--	print(total_force / mass)
	
	-- Temp approx
	-- Since 'total_torque' is in world coords
	local _world_inertia_tensor = plane_cf:VectorToWorldSpace(box_inertia_tensor(mass, Vector3.new(12.07, 3.84, 11.61)))
	--local angular_momentum = _inertia_tensor * current_angular_vel
	--local gyro_torque = angular_momentum:Cross(current_angular_vel)
--	total_torque -= gyro_torque	


	local linear_accel = total_force / mass
	local angular_accel = total_torque / _world_inertia_tensor
	new_linear_vel = current_linear_vel + (linear_accel * delta_time)
	new_angular_vel = current_angular_vel + (angular_accel * delta_time)
	return new_linear_vel, new_angular_vel
end

return Physics
