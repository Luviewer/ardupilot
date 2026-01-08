-- This script is an example setting up a custom dynamic 6DoF motor matrix
-- allowing a vehicle to change geometry in flight
-- Based on the 6DoF example frame: https://youtu.be/QUDhnYvH66k

-- Setup motor number (zero indexed), testing order (1 indexed), and reversible flag
Motors_6DoF_dynamic:add_motor(0, 1, true)  -- motor 0: reversible
Motors_6DoF_dynamic:add_motor(1, 2, true)  -- motor 1: reversible
Motors_6DoF_dynamic:add_motor(2, 3, true)  -- motor 2: reversible
Motors_6DoF_dynamic:add_motor(3, 4, true)  -- motor 3: reversible
Motors_6DoF_dynamic:add_motor(4, 5, true)  -- motor 4: reversible
Motors_6DoF_dynamic:add_motor(5, 6, true)  -- motor 5: reversible

-- Create factor table for 6DOF (includes forward and right factors)
factors = motor_factor_table_6dof()

-- Motor 0 factors: roll, pitch, yaw, throttle, forward, right
factors:roll(0, 0.003285)
factors:pitch(0, 0.470445)
factors:yaw(0, 0.031489)
factors:throttle(0, 0.885866)
factors:forward(0, 0.563927)
factors:right(0, 0.031026)

-- Motor 1 factors
factors:roll(1, -0.373686)
factors:pitch(1, -0.267196)
factors:yaw(1, 0.092861)
factors:throttle(1, 0.789066)
factors:forward(1, -0.223271)
factors:right(1, 0.495176)

-- Motor 2 factors
factors:roll(2, 0.370400)
factors:pitch(2, -0.203249)
factors:yaw(2, -0.053720)
factors:throttle(2, 0.841080)
factors:forward(2, -0.340656)
factors:right(2, -0.526202)

-- Motor 3 factors
factors:roll(3, -0.146980)
factors:pitch(3, -0.216342)
factors:yaw(3, -0.338296)
factors:throttle(3, 0.001577)
factors:forward(3, 0.900822)
factors:right(3, -0.460986)

-- Motor 4 factors
factors:roll(4, -0.205533)
factors:pitch(4, -0.035715)
factors:yaw(4, 0.359267)
factors:throttle(4, -0.048371)
factors:forward(4, 0.010753)
factors:right(4, -1.013924)

-- Motor 5 factors
factors:roll(5, 0.143881)
factors:pitch(5, -0.227449)
factors:yaw(5, 0.375220)
factors:throttle(5, -0.046098)
factors:forward(5, 0.811593)
factors:right(5, 0.431718)

-- Must load factors before init
Motors_6DoF_dynamic:load_factors(factors)

-- We're expecting 6 motors
assert(Motors_6DoF_dynamic:init(6), "Failed to init Motors_6DoF_dynamic")

motors:set_frame_string("6DoF Dynamic example")

-- At any time we can re-load new factors to change geometry in flight
-- Example: modify factors based on flight mode or vehicle configuration
-- Note: use pcall to protect the script from crashing when making changes in flight
-- See 'protected_call.lua' example for error handling

-- Update function to demonstrate dynamic factor changes
local UPDATE_RATE_MS = 100  -- Update every 100ms (10Hz)

function update()
    -- Example: You could update factors here based on flight conditions
    -- Motors_6DoF_dynamic:load_factors(new_factors)
    
    return update, UPDATE_RATE_MS
end

gcs:send_text(6, "6DoF Dynamic mixer loaded")
return update()
