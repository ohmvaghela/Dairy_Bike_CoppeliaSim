function sysCall_init()
	-- Paramters
	TorqueMax = 20			-- Max Torque
	LargeW = 1000				-- Maximum Velocity
	TUpdate = 1e-2     -- Should be multiple of 1e-2
	TControl = 5e-2    -- Should be multiple of T_update
	K = {140, 30, 0, 0, 1}

	-- Get object handles
	Body  = sim.getObjectHandle('DairyBikeChassis')
	RWMotor = sim.getObjectHandle('ReactionWheelMotor')
	ReactionWheel = sim.getObjectHandle('ReactionWheel')
	Rear = sim.getObjectHandle('RearMotor')
	Cylinder = sim.getObjectHandle('MTB_PickLink')

	-- Intialization
	LastUpdateTime = sim.getSimulationTime()
	LastControlTime = sim.getSimulationTime()
	States = {0, 0, 0, 0, 0}
	Integral = 0
	ExtraMassOnBody = 0
	ArmHasPackage = 0
	Body_slope = 0

	-- Threads
	CoroutineMain = coroutine.create(coroutine_main)
	UpdateExtraMassOnBody = coroutine.create(update_extra_mass_on_body)
	UpdateArmHasPkg = coroutine.create(update_arm_has_pkg)
end

function sysCall_actuation()
	if coroutine.status(CoroutineMain)~='dead' then
		local ok,errorMsg=coroutine.resume(CoroutineMain)
		if errorMsg then
			error(debug.traceback(CoroutineMain,errorMsg),2)
		end
	end
	if coroutine.status(UpdateExtraMassOnBody)~='dead' then
		local ok,errorMsg=coroutine.resume(UpdateExtraMassOnBody)
		if errorMsg then
			error(debug.traceback(UpdateExtraMassOnBody,errorMsg),2)
		end
	end
	if coroutine.status(UpdateArmHasPkg)~='dead' then
		local ok,errorMsg=coroutine.resume(UpdateArmHasPkg)
		if errorMsg then
			error(debug.traceback(UpdateArmHasPkg,errorMsg),2)
		end
	end
end

--[[
Function Name: update_extra_mass_on_body
	- Input:
	- Output:
	- Modifies:
		- ExtraMassOnBody
	- Logic:
		- takes input signal 'ChangeInMassOnBody' and adds to ExtraMassOnBody
		- clears current signal 
	- Example Call:
		- update_extra_mass_on_body()
--]]

function update_extra_mass_on_body()
	while true do
			ExtraMassOnBody = ExtraMassOnBody + sim.waitForSignal('ChangeInMassOnBody')
			sim.clearFloatSignal('ChangeInMassOnBody')
	end
end

--[[
Function Name: update_arm_has_pkg
	- Input:
	- Output:
	- Modifies:
		- ExtraMassOnBody
	- Logic:
		- takes input signal 'ArmHasPackage' and set ArmHasPackage variable   
		- clears current signal 
	- Example Call:
		- update_arm_has_pkg()
--]]

function update_arm_has_pkg()
	while true do
			ArmHasPackage = sim.waitForSignal('ArmHasPackage')
			sim.clearIntegerSignal('ArmHasPackage')
	end
end

function coroutine_main()
	while true do
			local CurrentTime = sim.getSimulationTime()
            -- The time step defined for code
			if (CurrentTime-LastUpdateTime>(TUpdate-1e-4)) then
					update_states()-- update bike states
					if ((CurrentTime-LastControlTime)>(TControl-1e-3)) then
                            update_lqr_weights() -- update lqr weights
							-- provide torque values from lqr weights and bike states
                            local torques = lqr() 
							generate(torques) -- generates torque
							LastControlTime = CurrentTime
					end
					LastUpdateTime = CurrentTime
			end
	end
end

--[[
Function Name: LQR
	- Input:
	- Output:
		- torques: table of torque proportional to the states
	- Modifies:
	- Logic:
		- Multiplies the values of K(LRQ weights) with respective states	
	- Example Call:
		- torques = lqr()
--]]

function lqr()
	-- Calculate desired torque
	local torques = {}
	for i = 1,#K,1 do
		 torques[i] = K[i] * States[i]
	end

	return torques
end

--[[
Function Name: update_states
	- Input:
	- Output:
	- Modifies:
		- states of the body 
			- q1 -> Body's orientation
			- q2 -> Body's angular velocity
			- q3 -> Integral term for state 'q1'
			- q4 -> reaction wheel's orientation
			- q5 -> reaction wheel's angular velocity
	 - Logic:	
	- Example Call:
		- update_states()
--]]

function update_states()
	-- Fetch States
	local body_abg = sim.getObjectOrientation(Body, -1)
	local body_yaw, body_pitch, body_roll = sim.alphaBetaGammaToYawPitchRoll(body_abg[1], body_abg[2], body_abg[3])
	local q1 = -body_roll

	local q2 = (q1-States[1])/TUpdate
	
	Integral = Integral + q1
	local q3 = Integral

	local rw_abg = sim.getObjectOrientation(ReactionWheel, -1)
	local rw_yaw, rw_pitch, rw_roll = sim.alphaBetaGammaToYawPitchRoll(rw_abg[1], rw_abg[2], rw_abg[3])
	local q4 = -rw_pitch

	local q5 = -sim.getJointVelocity(RWMotor)
	
	Body_slope = math.deg(math.abs(body_pitch))
	States = {q1, q2, q3, q4, q5}
end

--[[
Function Name: generate
	- Input:
		- Table of torques that are proportional to states
	- Output:
	- Modifies:
		- Set the torque of reaction motor 
	- Logic:	
		- Adds all the values of torques that is given as input
		- then checks the direction of torque and stores in variable 'torque_sign'
		- also absolute value of torque is stored in 'torque_mag'
		- Now to provide to necessary torque joint's force's max value is set to 'torque_mag'
		- So when given target velocty it can generate desired torque 
		- For the same reason the maximum velocity is set very large 'LargeW'
	- Example Call:
		- generate()
--]]

function generate(torques)
	-- Declare necessary variables
	local torque = 0
	local torque_mag, torque_sign;

	for i=1,#torques,1 do
			torque = torque+torques[i]
	end

	-- Get magnitude and sign of the torque
	if (torque>=0) then
			torque_mag = torque
			torque_sign = 1
	else
			torque_mag = -1*torque;
			torque_sign = -1
	end

	-- Limit the torque to torque_max
	if (torque_mag>TorqueMax) then
			torque_mag = TorqueMax
	end

	-- Control the rw_motor
	sim.setJointMaxForce(RWMotor, torque_mag)
	if (torque_sign==1) then
			sim.setJointTargetVelocity(RWMotor, -LargeW)
	else
			sim.setJointTargetVelocity(RWMotor, LargeW)
	end
end

--[[
Function Name: update_lqr_weights
	- Input:
	- Output:
	- Modifies:
		- Modifies LRQ weights 
	- Logic:	
		- Variation of weights starts from a predefined value
		- modification is solely based on Extra mass on body added'ExtraMassOnBody' and inclination on slope'Body_slope' 
	- Example Call:
		- update_lqr_weights()
--]]

function update_lqr_weights()

	if ArmHasPackage==1 then
			K = {
					203 + ExtraMassOnBody*3.763 ,--158
					48 + ExtraMassOnBody*0.8562, --38
					0,
					0,
					1.0524
			}
	else
			K = {
					203 + ExtraMassOnBody*3.763  + Body_slope*1.5,
					48 + ExtraMassOnBody*0.8562 + Body_slope*0.45,
					0,
					0,
					1.0524
			}
	end
end