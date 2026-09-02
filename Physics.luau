-- Half-assisted AI, tanks
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
		- Relative wind is simplified to flight_dir, no external weather wind whatsoever
]]
local function get_AoA(local_current_flight_dir: Vector3)
	return math.atan2(-local_current_flight_dir.Y, local_current_flight_dir.Z)
end

local function get_AoA_old(ref_cf: CFrame, world_flight_dir: Vector3)
	-- flight dir relative to roation of 'ref_cf' 
	local local_flight_dir = ref_cf:VectorToObjectSpace(world_flight_dir)
	local AoA = math.atan2(-local_flight_dir.Y, local_flight_dir.Z)
	return AoA
end

-- ============================================================
-- LIFT COEFFICIENT
-- ============================================================
local function get_dynamic_pressure(speed: number): number
	return 0.5 * AIR_DENSITY * (speed * speed)
end

local function get_induced_drag_coeff(lift_coeff: number, aspect_ratio: number, oswald_eff_factor: number ): number
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
	-- PROB TEMP
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

local function get_lift_direction(goal_cf: CFrame, flight_dir: Vector3): Vector3
	local plane_up = goal_cf.UpVector

	-- Project aircraft UpVector onto the plane perpendicular
	-- to velocity.
	local projected_up =
		plane_up
	- flight_dir * plane_up:Dot(flight_dir)

	local magnitude = projected_up.Magnitude

	if magnitude > EPSILON then
		return projected_up / magnitude
	end

	-- Degenerate case: velocity almost parallel to UpVector.
	local right = goal_cf.RightVector
	local fallback = right:Cross(flight_dir)

	if fallback.Magnitude > EPSILON then
		return fallback.Unit
	end

	return Vector3.yAxis
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

local function get_offseted_force_result(
	cf: CFrame,
	offset_local: Vector3,
	force_world: Vector3,
	mass: number
): (Vector3, Vector3)

	assert(mass > EPSILON, "Mass must be > 0")

	-- Offset is supplied in aircraft-local coordinates.
	local offset_world = cf:VectorToWorldSpace(offset_local)

	-- Linear acceleration.
	local linear_accel = force_world / mass

	-- Torque must use vectors expressed in the same coordinate space.
	local torque_world = offset_world:Cross(force_world)

	-- NOTE:
	-- This is only a placeholder approximation.
	-- torque / mass is NOT physically correct angular acceleration.
	--
	-- Keep it here only if you're intentionally using a fake
	-- angular response.
	local angular_accel = torque_world / mass
	return linear_accel, angular_accel
end

local function apply_force_at_offset()
	
end

--[[
	- Figure if vel & angular vel need be reset after each loop step, prevent wrong accumulation
	- Maybe add basic flaps simulation (flap_ratio, no physical model)?
	
	- local_CoA_cf, positive +Z positional component is backwards from CoA, as -Z is lookVector / forwards dir 
]]
local function apply_wing_aerodynamics(wing_info: PlaneInfos.WingInfo, world_current_linear_vel: Vector3): (Vector3, Vector3)
	local local_linear_accel: Vector3, local_angular_accel: Vector3
	-- cf relative to CoM
	local local_wing_cf = wing_info.local_CoA_cf
	-- perpendicular to chord line (SIMPLIFIED to LookVector in this case)
	
	-- local vel's difference is that it has a direction relative to reference CF
	local local_current_linear_vel = local_wing_cf:VectorToObjectSpace(world_current_linear_vel)
	local local_current_speed = local_current_linear_vel.Magnitude
	
	local dynamic_pressure = get_dynamic_pressure(local_current_speed)
	local AoA = math.clamp(get_AoA(local_current_linear_vel.Unit), wing_info.min_aoa, wing_info.max_aoa)
	local lift_coeff = get_lift_coeff(AoA, wing_info.lift_coeff_vs_AoA_curve)
	
	local drag_dir = -local_current_linear_vel.Unit
	-- lift is perpendicular to drag
	-- 1st Cross product to get rightVector, 2nd to get perpendicular to linear vel
	-- Prob could just simplify via CFrame.Angles, fromAxisAngle, or other thing
	local lift_dir = drag_dir:Cross(local_wing_cf.UpVector):Cross(drag_dir)
	
	local lift_force = lift_dir * get_lift(lift_coeff, dynamic_pressure)
	local drag_force = drag_dir * get_drag(wing_info.parasitic_drag_coeff, lift_coeff, dynamic_pressure)
end

--[[
	TEMP:
]]
function Physics.steer_towards_dir(
	world_current_linear_vel: Vector3,
	world_current_angular_vel: Vector3,
	plane_cf: CFrame,
	goal_dir: Vector3,	-- treated like an instant cf after rotation, replacing flaps
	plane_name: string,
	unit_throttle: number,
	delta_time: number
): (Vector3, Vector3)

	local new_linear_vel: Vector3, new_angular_vel: Vector3 = Vector3.zero, Vector3.zero
	local total_linear_accel: Vector3, total_angular_accel: Vector3 = Vector3.zero, Vector3.zero
	
	local physics_info = PlaneInfos.physics[plane_name]
	local mass = physics_info.mass

	unit_throttle = math.clamp(unit_throttle, 0, 1)
	
	-- goal_dir will be reached via angular velocity (hopefully)
	local thrust_force = plane_cf.LookVector * physics_info.max_thrust * (unit_throttle * unit_throttle)
	local weight_force = GRAVITY_ACCEL * mass


	-- Avoid divide by 0 errors for aerodynamic forces calculations.. anddd would probably be neglible anyways
	if world_current_linear_vel.Magnitude <= EPSILON then
		local total_force = thrust_force + weight_force
		total_linear_accel = total_force / mass
		new_linear_vel = world_current_linear_vel + (total_linear_accel * delta_time)
		return new_linear_vel, world_current_angular_vel
	end
	
	total_linear_accel, total_angular_accel = Vector3.zero, Vector3.zero
	for _, wing_info in physics_info.wings do
		local linear_accel, angular_accel = apply_wing_aerodynamics(wing_info, world_current_linear_vel)
		total_linear_accel += linear_accel
		total_angular_accel += angular_accel
	end
	
	new_linear_vel += total_linear_accel * delta_time
	new_angular_vel += total_angular_accel * delta_time
	return new_linear_vel, new_angular_vel
end

return Physics
