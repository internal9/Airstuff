--[[
	NOTES:
	- Plane's 'PivotOffset' should be set to their center of gravity (which is basically same as center of mass on Earth),
		only plane's CoM is used, which ignores any extra payload (addons, etc)
	- Max lift coeffs for modern jets ar areound 2 - 3
	- Lift & drag coeffs vs. AoA plots (& possibly velocity later):
		- curve plots are used, values likely not perfect
		- at subsonic or lower speeds irl, lift coeff increases linearly to speed

	- Simplifications:
		- No control surfaces
		
	- To Add:
	
	- Maybe To Add:
		- Swept wing simulation
		- Aerodyamics beyond wings (e.g. each plane part contributes parasitic drag)
		
	- Other:
		- Lookup tables for aerodynamic forces over observation (requires re-generating if adjusting any coeffs)
		- Lookup table for aerodynamic coefficients over observation
		- Group as many constants as possible to avoid redundant expression evaluation
]]

local AirfoilInfo = require("./AirfoilInfo")
local WorldInfo = require("./WorldInfo")
local constants = WorldInfo.constants

local NEWPlaneInfos = {
	--guns = {},
	--missiles = {},
	--body = {},
	--physics = {},
	--physics_limits = {}
}

--[[
	physics scale is only applied to final world position, others kept real-world for consistency & easier
	
	make sure to convert units depending on equations (e.g. N -> kN)
	metric units: (change some scale?)
	force is in newtons N
	liter in L
	area in m^2
	and so on
	mass in kG
	
	angles: radians in all computations
	
	mayb:
	airfoil
	better drag & lift forces & coeffs
]]

export type WingInfo = {
	--[[
		- relative to plane, includes wing's local rotation to plane too
		- Z offset is CoA (center of aerodynamics) distance from CoM in meters
		- CoA is a simpler alternative to center of pressure
	]]
	local_CoA_cf: CFrame,
	area: number,	
	-- wing area from perspective of top (2D perspective, ignores dihedral / anhedral angle & angle of incidence of wing)
	projected_area: number,
	-- this case, refers to tip of stabilizer to fuselage or an approximate point (height?)
	span: number,
	-- wing-span^2 / projected-wing-area
	aspect_ratio: number,
	-- vertical stabilizer's airfoil is symmetric, and ofc wouldn't want lateral drag when slideslip angle = 0
	parasitic_drag_coeff: number,
	-- lower for vertical stabilizers ig
	oswald_eff_factor: number,
	--  May partially usee NACA Airfoil data
	lift_coeff_vs_AoA_curve: FloatCurve,
	
	-- Temp,
	min_aoa: number,
	max_aoa: number,
	
	-- Maybe (?)
	flap_ratio: number?,
	max_flap_deflection_angle: number?
}

local headers = {}
local headers_count = 0

local function header(name: string)
	local value = headers[name]
	if value == nil then headers_count += 1 headers[name] = headers_count value = headers[name] end
	return value
end

local function description(name: string)
	return function(a: string) end
end

-- So that constants aren't contioniously evaluated in expressions e.g. math.pi * constant
-- Or move said functionality into a different module
local function setup_aerodynamic_constants()

end

local function curve_plot(set)
	local plot = Instance.new("FloatCurve")
	for i, v in set do
		plot:InsertKey(FloatCurveKey.new(i, v, Enum.KeyInterpolationMode.Cubic))
	end
	return plot
end

local OSWALD_EFF_FACTORS = {	-- this also changess  based on speeeeed
	STRAIGHT_WING = 0.7,
	DELTA_WING = 0.9
}

local guns = {}
local missiles = {}
local body = {}			-- dimensions
local performance = {}		-- operational info & limits (e.g. max G-Force)
local physics = {}			-- physics coefficients

NEWPlaneInfos.guns = guns
NEWPlaneInfos.missiles = missiles
NEWPlaneInfos.body = body
NEWPlaneInfos.performance = performance
NEWPlaneInfos.physics = physics

description("F-84G")
	[[
		- Poor energy retentiion
		- Poor turning speed
		- Slow acceleration rate
		- Poor climb rate
		- Good straight-line stability
		- High speeds in dives / straight line
		- Tough airframe
		- High-speed limits at low altitudes
	]]

guns["f84g"] = {}

body["f84g"] = {
	hp = 12,
	length = 11.61,
	height = 3.84,
}

-- poor energy retention in hard turns
performance["f84g"] = {
	-- L, extra 860 * 2 Liters from external wing-tip fuel tanks
	max_fuel = 1_700 + 860 * 2,
	min_g_force = -5,
	max_g_force = 11,
	--max_instant_turn_rate = 123132132,

	-- idk about these
	zero_lift_drag_coeff = 0.0174533,
	min_AoA = -0.0523599,	-- min / max should just depend on airfoils
	max_AoA = 0.314159,	-- critical angle of attack before a stall
}

physics["f84g"] = {
	-- Only lift & drag of wings are simulated, fuselage is ignored
	mass = 8_457,
	max_thrust = 25_000,

	-- either rename terms for laterial axis, or have an array of 'wing' data types to encompass all wing surfaces

	--critical_mach_number = 0,	-- do this? or just simplify to fixed values for all aircraft
	-- basic flaps?
	wings = {
		-- Combined characteristics of the two main wings for simplicity
		[header("Main Wings")] = {
			-- temp var
			aerodynamic_center = 0.834,
			-- local to plane's CFrame
			local_CoA_cf = CFrame.new(0, 0, 0.834),
			
			wing_area = 24.2,	--  flat 
			wing_span = 11.1,		-- meters
			projected_wing_area = 24.2,	-- as viewed from top, confused
			aspect_ratio = 5.1,	-- wing_span^2 / wing_area

			--[[
				- 'aerodynamic center' is a SIMPLER alternative to CoP,
				- it changes based off speed
				- subsonic: approx 25% of chord length FROM leading edge
				- supersonic: approx 50% of chord length FROM leading edge
				
				- if AoC is behind CoM, usually more stable, so main wing by itself can't product turning, just stabilizing
				that's why stabilators OR control surfaces will be simulated
			]]

			parasitic_drag_coeff = 0.022,	-- relatively constant at subsonic speeds
			oswald_eff_factor = 0.7,	-- 0.7 very common for straight wing

			--[[
				- 90 to 90 deg range, lift coeff v aoa curve is periodic,
				aoa is clamped as lift coeff at 180, 90 deg are pretty much near / at 0
				
				- Tried getting best data, this might be a little based off NACA 65-210 airfoil too
			]]
			lift_coeff_vs_AoA_curve = curve_plot({
				[math.rad(-90)] = 0.01,
				[math.rad(-7)] = -0.4,
				[math.rad(-1.5)] = 0,
				[math.rad(0)] = 0.15,
				[math.rad(18)] = 1.2,
				[math.rad(90)] = 0.01,
			}),
			
			min_aoa = math.rad(-90),
			max_aoa = math.rad(90),
		},

		[header("Vertical Stabilizer")] = {
			wing_area = 2.7,	-- m^2
			wing_span = 2.05,	-- this case, refers to tip of stabilizer to fuselage or an approximate point (height?)
			aspect_ratio = 1.556,
			-- meters from plane's CoM (replace w/ CoA_cf?)
			aerodynamic_center = 6.55,
			-- 90 deg rotation, to simulate side lift & drag (lift & drag at yaw instead of pitch)
			local_CoA_cf = CFrame.new(0, 0, 6.55) * CFrame.Angles(0, 0, math.rad(90)),
			-- At sideslip angle of 0 deg, this resists plane's forward movement
			parasitic_drag_coeff = 0.0009,
			-- lower for vertical stabilizers ig
			oswald_eff_factor = 0.6,
			--[[
				- based off NACA 65-0010 airfoil, f-84g's actual airfoil data is properiatery
				- extend beyond [-90, 90] deg? so that not max 'side_force_coeff' when
				it's like 360 deg (practically same as 0 deg effect)
				
				- make stabilizer less effetive beyond max side slip angle (for f84g recorded was 32 deg),
				& add in stalling
			]]
			lift_coeff_vs_AoA_curve = curve_plot({
				[math.rad(-32)] = -2,	
				[math.rad(-15)] = -1.4,
				[math.rad(0)] = 0,	-- try 0.2?
				[math.rad(15)] = 1.4,
				[math.rad(32)] = 2 ,
			}),
			
			min_aoa = math.rad(-32),
			max_aoa = math.rad(32),
		},		

	}
}



return NEWPlaneInfos
