function sysCall_init()
	-- ObjectHandles
	SuctionPadHandle = sim.getObjectHandle("suctionPadSensor")
	LoopClosureDummy1 = sim.getObjectHandle("suctionPadLoopClosureDummy1")
	LoopClosureDummy2 = sim.getObjectHandle("suctionPadLoopClosureDummy2")
	J3 = sim.getObjectHandle("J3")

	sim.setLinkDummy(LoopClosureDummy1, -1)
	sim.setObjectParent(LoopClosureDummy1, J3, true)

	LoopClosureDummyObjMatrix = sim.getObjectMatrix(LoopClosureDummy2, -1)
	sim.setObjectMatrix(LoopClosureDummy1, -1, LoopClosureDummyObjMatrix)

	sim.setIntegerSignal("MTB_ObjConnected", 0)
	Corout = coroutine.create(coroutineMain)
end

function sysCall_cleanup()
	sim.setLinkDummy(LoopClosureDummy1, -1)
	sim.setObjectParent(LoopClosureDummy1, J3, true)
	LoopClosureDummyObjMatrix = sim.getObjectMatrix(LoopClosureDummy2, -1)
	sim.setObjectMatrix(LoopClosureDummy1, -1, LoopClosureDummyObjMatrix)
end

function sysCall_actuation()
	if coroutine.status(Corout) ~= "dead" then
		local ok, errorMsg = coroutine.resume(Corout)
		if errorMsg then
			error(debug.traceback(Corout, errorMsg), 2)
		end
	end
end

--[[
Function Name: coroutineMain
	- Input:
	- Output:
	- Modifies:
	- Logic:
		- Listens for "MTB_PickUpOrDropReady" (set by MTB_Axis1)
		- Attaches/detaches object from PickLink according to the signal
		- Sets "PickUpConnected" after picking the object
		- Sets "PickUpDropped" after dropping the object
	- Example Call:
		- coroutineMain()
--]]
function coroutineMain()
	while true do
		-- Set the active/inactive state (directly controlled by the robot program):
		local pickUpOrDropSignal = sim.waitForSignal("MTB_PickUpOrDropReady")

		local active = false
		local pickupObjName = ""
		if pickUpOrDropSignal == 1 then
			active = true
			pickupObjName = sim.waitForSignal("MTB_PickUpObject")
			sim.clearStringSignal("MTB_PickUpObject")
		else
			if pickUpOrDropSignal == 2 then
				-- Drop the object
				sim.setLinkDummy(LoopClosureDummy1, -1)
				sim.setObjectParent(LoopClosureDummy1, J3, true)
				LoopClosureDummyObjMatrix = sim.getObjectMatrix(LoopClosureDummy2, -1)
				sim.setObjectMatrix(LoopClosureDummy1, -1, LoopClosureDummyObjMatrix)
				sim.clearIntegerSignal("MTB_PickUpOrDropReady")
				sim.setIntegerSignal("MTB_PickUpDropped", 1)
			end
		end

		-- Pick up logic
		local parent = sim.getObjectParent(LoopClosureDummy1)
		if (active == false) then
			sim.setLinkDummy(LoopClosureDummy1, -1)
			sim.setObjectParent(LoopClosureDummy1, J3, true)
			LoopClosureDummyObjMatrix = sim.getObjectMatrix(LoopClosureDummy2, -1)
			sim.setObjectMatrix(LoopClosureDummy1, -1, LoopClosureDummyObjMatrix)
		else
			if (parent == J3) then
				-- Use dummy linking to attach/pick the object
				local shape = sim.getObjectHandle(pickupObjName)
				sim.setObjectParent(LoopClosureDummy1, shape, true)
				sim.setLinkDummy(LoopClosureDummy1, LoopClosureDummy2)
				sim.setIntegerSignal("MTB_PickUpConnected", 1)
				sim.clearIntegerSignal("MTB_PickUpOrDropReady")
			end
		end
	end
end
