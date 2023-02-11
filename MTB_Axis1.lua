function sysCall_init()
	-- Object Handles
	Axis1Handle = sim.getObjectHandle("MTB_Axis1")
	Axis2Handle = sim.getObjectHandle("MTB_Axis2")
	Axis4Handle = sim.getObjectHandle("MTB_Axis4")
	RevJointHandle = sim.getObjectHandle("MTB_Axis3")
	ChassisHandle = sim.getObjectHandle("DairyBikeChassis")
	ProximitySensorHandle = sim.getObjectHandle("suctionPadSensor")
	ArmTipHandle = sim.getObjectHandle("MTB_ArmTip")

	-- Tolerances for various controllers 
	Tolerances = {}
	Tolerances["xy_angle"] = math.rad(0.1) -- 0.05
	Tolerances["xy_dist"] = 0.02
	Tolerances["z"] = 0.001
	Tolerances["rev"] = math.rad(1)

	-- Default z-pos for ArmTip (To avoid collisions)
	TipDefaultZPos = 0.01
	-- Z-pos for ArmTip holding an object
	TipZPosWithObject = 0.02

	-- Height and length of the objects (same for all)
	ObjHeight = 0.1
	ObjLength = 0.1

	-- Clearance for length and breadth
	ObjLengthClearance = 0.02

	-- Clearance for Z co-ordinate while picking
	PickupClearance = 0.02
	-- Clearance for Z co-ordinate while placing
	PlaceClearance = 0.03

	-- Proportionality coefficients for various P-controllers
	KP = {}
	-- Handles of all the tables
	TableHandles = {}
	-- Positions of all the storage positions in the bucket of DB
	StorageAreaPos = {}
	-- Will store 6 drop positions on the table
	DropPosTable = {}
	-- Mapping from destination name to table handles
	DestToTable = {}
	propagateValues()

	-- local str =
	--	"['location ', 'Paneer', 'Curd', 'Milk'],['House_1', 1, 0, 0],['School', 0, 0, 2],['Dhaba', 1, 1, 0],['Hospital', 0, 1, 1],['House_2', 1, 0, 0]"
	local str = sim.waitForSignal("original_conf")
	print("SIGNAL", str)

	--[[
		A table maintaining the number of items of "itemName" to be delivered at a given "destName"
		It has the following format
		[
			[destName] : [
				[itemName]: quantity
			]
		]
	--]]
	Orders = {}
	extractOrders(str)
	print(Orders)
	-- Number of items required for each item-type
	RequiredItems = {}
	propagateRequiredItems()

	-- Storage-area index of each item
	ObjectStorageAreaIndex = {}
	ObjectStorageAreaIndex["Feed"] = {}
	-- For object ~= "Feed", ActiveObjectIndex[object] = smallest index of object avl in Bucket
	-- For feed -> number of feeds in the bucket
	ActiveObjectIndex = {}
	ActiveObjectIndex["Feed"] = 0
	EmptyStoragePos = {}
	Corout = coroutine.create(coroutineMain)
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
		- Listens for "destinationSignal" (sent by SteeringMotor)
		- Performs all the actions required for that destination
		- Sets "readyToMove" signal after its done
	- Example Call:
		- coroutineMain()
--]]
function coroutineMain()
	while true do
		local destName = sim.waitForSignal("destinationSignal")
		sim.clearStringSignal("destinationSignal")
		print("Destination: " .. destName)
		if destName == "Loading_Point" then
			handleLoadingPoint()
		elseif destName == "Delivery_Point_House1" then
			handleDeliveryPoint("House_1")
		elseif destName == "Delivery_Point_School" then
			handleDeliveryPoint("School")
		elseif destName == "Delivery_Point_Dhaba" then
			handleDeliveryPoint("Dhaba")
		elseif destName == "Delivery_Point_Hospital" then
			handleDeliveryPoint("Hospital")
		elseif destName == "Delivery_Point_House2" then
			handleDeliveryPoint("House_2")
		elseif destName == "Delivery_Point_DairyFarm" then
			handleDairyFarm()
		else
			print("unknown location!!")
		end
		sim.setIntegerSignal("readyToMove", 1)
	end
end

--[[
Function Name: propagateValues
	- Input:
	- Output:
	- Modifies:
		- KP
		- TableHandles
		- StorageAreaPos
		- DropPosTable
		- DestToTable
	- Logic:
		- Initializes mappings, positions, and parameters
	- Example Call:
		- propagateValues()
--]]
function propagateValues()
	KP["rev"] = 8
	KP["z"] = 3
	KP["xy"] = 16

	for i = 0, 13, 1 do
		TableHandles[i] = sim.getObjectHandle("Table" .. i)
	end

	for i = 1, 8, 1 do
		local dummy = sim.getObjectHandle("PosDummy" .. i)
		StorageAreaPos[i] = sim.getObjectPosition(dummy, ChassisHandle)
	end

	DropPosTable = {
		{0.1490, 0, 0.075}, -- 3
		{0.1490, 0, -0.075}, -- 4
		{0.1490, 0, 0.225}, -- 2
		{0.1490, 0, -0.225}, -- 5
		{0.1490, 0, 0.375}, -- 1
		{0.1490, 0, -0.375} -- 6
	}

	DestToTable["House_1"] = {TableHandles[4], TableHandles[5]}
	DestToTable["School"] = {TableHandles[6], TableHandles[7]}
	DestToTable["Dhaba"] = {TableHandles[12], TableHandles[13]}
	DestToTable["Hospital"] = {TableHandles[8], TableHandles[9]}
	DestToTable["House_2"] = {TableHandles[10], TableHandles[11]}
	DestToTable["DairyFarm"] = {TableHandles[2], TableHandles[3]}
end

--[[
Function Name: extractOrders
	- Input:
		- str: a string that denotes the orders
		(string recevied from original_conf/bonus_conf signal)
	- Output:
	- Modifies:
		- Orders: A table that denotes the number of items of each types
		that is to be delivered at each destination
	- Logic:
		- Uses RegEx to parse the given string
		- Parses a string in to a 2D table, and uses that table to fill Orders
		- The first row in the parsed table is assumed to be a header
		- In the rest of the rows, the values should be according to the header of the column
	- Example Call:
		- Orders = {}
		- local orderString = sim.waitForSignal('original_conf')
		- extractOrders(orderString)
--]]
function extractOrders(str)
	local isFirst = true
	local indices = {}
	-- parse the 2D table
	for line in string.gmatch(str, "%[([^%[%]]+)%]") do
		if isFirst then
			-- First row -> Header
			local i = 1
			for word in string.gmatch(line, "([^,]+)") do
				local cleanWord = (string.match(word, "[^'%s]+"))
				indices[cleanWord] = i
				i = i + 1
			end
			isFirst = false
		else
			local row = {}
			local i = 1
			for word in string.gmatch(line, "([^,]+)") do
				-- create a table by parsing the line
				local cleanWord = string.match(word, "[^'%s]+")
				row[i] = cleanWord
				i = i + 1
			end
			-- fill appropriate values in orders table
			local destName = row[indices["location"]]
			Orders[destName] = {}
			for k, v in pairs(indices) do
				if k ~= "location" then
					Orders[destName][k] = tonumber(row[v])
				end
			end
		end
	end
end

--[[
Function Name: propagateRequiredItems
	- Input:
	- Output:
	- Modifies:
		- RequiredItems
	- Logic:
		- Counts the quantity of each item to fullfill all the orders
	- Example Call:
		- Orders = {.....}
		- RequiredItems = {}
		- propagateRequiredItems()
--]]
function propagateRequiredItems()
	for _, order in pairs(Orders) do
		for k, v in pairs(order) do
			if RequiredItems[k] == nil then
				RequiredItems[k] = 0
			end
			RequiredItems[k] = RequiredItems[k] + v
		end
	end
end

--[[
Function Name: handleLoadingPoint
	- Input:
	- Output:
	- Modifies:
		- ObjectStorageAreaIndex
		- ActiveObjectIndex
	- Logic:
		- To be called after reaching at LoadingPoint
		- Puts all required items in the bucket of DB
		- Adds storage-area-index to ObjectStorageAreaIndex table for each item placed in the bucket
		- Sets ActiveObjectIndex for each item-type to point at the first item of that item-type
		- After picking all the items, it moves arms to the neutral position
	- Example Call:
		- RequiredItems = {.....}
		- objectStorageAreaIndex()
		- ActiveObjectIndex = {}
		- handleLoadingPoint()
--]]
function handleLoadingPoint()
	local index = 1
	for name, quant in pairs(RequiredItems) do
		if quant >= 1 then
			ObjectStorageAreaIndex[name] = {}
			ActiveObjectIndex[name] = 1
			for i = 1, quant, 1 do
				ObjectStorageAreaIndex[name][i] = index
				putObjectInStorageArea(name .. (i - 1), index)
				index = index + 1
			end
		end
	end
	goToJointPosXY(0, 0)
end

--[[
Function Name: handleDeliveryPoint
	- Input:
		- delPointName: name of the delivery point
	- Output:
	- Modifies:
		- ActiveObjectIndex (increments by one for each item of given item-type delivered at "delPointName")
		- ObjectStorageAreaIndex (adds feeds' storage-area-indices)
		- EmptyStoragePos (if #feed != #itemsPlaces, adds empty-storage-area indices in this table)
	- Logic:
		- For a given delPointName, it puts items on the table according to the Orders, and picks feeds on those table
		- It first finds an empty slot on either of the tables, and places the first order item,
			and then it picks a feed (if any) and puts it in the bucket,
			then it again places an object (if any), and picks a feed (if any) and so on
		- Increments ActiveObjectIndex of an item-type by one for each item placed of the same item-type
		- Increments ActiveObjectIndex of Feed by the number of feeds picked at this point
	- Example Call:
			- destinationName = sim.waitForSignal('DestinationSignal')
			- handleDeliveryPoint(destinationName)
--]]
function handleDeliveryPoint(delPointName)
	local destTables = DestToTable[delPointName]
	local feedsToBePicked = {}
	local feedIndex = 1
	local dropPosIndex = -1
	local dropTableHandle = -1
	for i = 1, #destTables, 1 do
		local objHandles = getObjectsOnTable(destTables[i])
		local feedHandles, feedIds = getFeedsOnTable(objHandles)
		for j = 1, #feedHandles, 1 do
			feedsToBePicked[feedIndex] = feedIds[feedIndex]
			feedIndex = feedIndex + 1
		end
	end

	local order = Orders[delPointName]
	local index = 1
	local numItemsDelivered = 0
	feedIndex = 1
	for name, quant in pairs(order) do
		numItemsDelivered = numItemsDelivered + quant
		for i = 1, quant, 1 do
			-- find an empty place
			dropPosIndex = -1
			dropTableHandle = -1
			for j = 1, #destTables, 1 do
				local objHandles = getObjectsOnTable(destTables[j])
				dropPosIndex = getEmptySlotOnTable(destTables[j], objHandles)
				if dropPosIndex ~= -1 then
					dropTableHandle = destTables[j]
					break
				end
			end
			if dropPosIndex == -1 then
				print("[ERROR] NO DROP POSITIONS FOUND!!!!!")
				return
			end
			putObjectOnTable(name .. (ActiveObjectIndex[name] - 1), DropPosTable[dropPosIndex], dropTableHandle)

			local emptyPos = ObjectStorageAreaIndex[name][ActiveObjectIndex[name]]
			ActiveObjectIndex[name] = ActiveObjectIndex[name] + 1
			index = index + 1

			if feedsToBePicked[feedIndex] ~= nil then
				-- pick a feed
				putObjectInStorageArea("Feed" .. (feedsToBePicked[feedIndex]), emptyPos)
				ObjectStorageAreaIndex["Feed"][feedIndex] = emptyPos
				ActiveObjectIndex["Feed"] = ActiveObjectIndex["Feed"] + 1
				feedIndex = feedIndex + 1
			else
				table.insert(EmptyStoragePos, emptyPos)
			end
		end
	end

	while feedsToBePicked[feedIndex] ~= nil do
		-- pick a feed
		local emptyPos = EmptyStoragePos[1]
		if emptyPos == nil then
			print("[ERROR] No empty slots in storage area to place the feed")
			return
		end
		table.remove(EmptyStoragePos, 1)
		putObjectInStorageArea("Feed" .. (feedsToBePicked[feedIndex]), emptyPos)
		ObjectStorageAreaIndex["Feed"][feedIndex] = emptyPos
		-- ActiveObjectIndex["Feed"] = ActiveObjectIndex["Feed"] + 1
		feedIndex = feedIndex + 1
	end
	goToJointPosXY(0, 0)
end

--[[
Function Name: handleDairyFarm
	- Input:
	- Output:
	- Modifies:
	- Logic:
		- Puts all the feeds the tables
	- Example Call:
		- handleDairyFarm()
--]]
function handleDairyFarm()
	local farmTables = DestToTable["DairyFarm"]
	local farmTable = farmTables[1]
	local index = 1
	for i = 1, ActiveObjectIndex["Feed"], 1 do
		putObjectOnTable("Feed" .. (i - 1), DropPosTable[index], farmTable)
		index = index + 1
		if index > 6 then
			index = 1
			farmTable = farmTables[2]
		end
	end
	goToJointPosXY(0, 0)
end

--[[
Function Name: getObjectsOnTable
	- Input:
		- tableHandle: handle of the table under consideration
	- Output:
		- objectHandles: A list of objects in contact with the "tableHandle"
	- Modifies:
	- Logic:
		- Used getContactInfo to iteratively get all the objects in contact with the table
	- Example Call:
		- local objects = getObjectOnTable(TableHandles[1])
--]]
function getObjectsOnTable(tableHandle)
	-- Table used as a HashSet
	local objectHandlesSet = {}
	local contactIndex = 0
	while true do
		local objects = sim.getContactInfo(sim.handle_all, tableHandle, contactIndex)
		if objects == nil then
			break
		end
		if objects[1] ~= tableHandle then
			objectHandlesSet[objects[1]] = true
		else
			objectHandlesSet[objects[2]] = true
		end
		contactIndex = contactIndex + 1
	end

	local objectHandles = {}
	local index = 1
	for objectHandle in pairs(objectHandlesSet) do
		objectHandles[index] = objectHandle
		index = index + 1
	end
	return objectHandles
end

--[[
Function Name: getFeedsOnTable
	- Input:
		- objectHandles: A list of objects in contact with the table under consideration
	- Output:
		- feedHandles: A list of feed-handles of all the feeds that are in contact with the table
		- feedIds: A list of feed-ids of all the feeds that are in contact with the table
	- Modifies:
	- Logic:
	- Example Call:
		- local objects = getObjectsOnTable(TableHandles[1])
		- local feedHandles, feedIds = getFeedsOnTable(objects)
--]]
function getFeedsOnTable(objectHandles)
	local feedHandles = {}
	local feedIds = {}
	local index = 1
	for i = 1, #objectHandles, 1 do
		local objName = sim.getObjectName(objectHandles[i])
		if string.sub(objName, 1, 4) == "Feed" then
			feedHandles[index] = objectHandles[i]
			feedIds[index] = string.sub(objName, 5)
			index = index + 1
		end
	end
	return feedHandles, feedIds
end

--[[
Function Name: extractOrders
	- Input:
		- tableHandle: handle of the table under consideration
		- objHandles: list of objects in contact with the "tableHandle"
	- Output:
		- index: index of an empty slot on the table, -1 if all slots are occupied
	- Modifies:
	- Logic:
	- Example Call:
		- local table = TableHandles[1]
		- local objects = getObjectsOnTable(table)
		- local slotIndex = getEmptySlotOnTable(table, objects)
--]]
function getEmptySlotOnTable(tableHandle, objHandles)
	if #objHandles == 0 then
		return 1
	end
	local index = 1
	local isValid
	while index <= #DropPosTable do
		isValid = true
		local xLow = DropPosTable[index][3] - (1.21 * ObjLength + ObjLengthClearance)
		local xHigh = DropPosTable[index][3] + (1.21 * ObjLength + ObjLengthClearance)
		for i = 1, #objHandles, 1 do
			local feedPos = sim.getObjectPosition(objHandles[i], tableHandle)[3]
			if feedPos >= xLow and feedPos <= xHigh then
				isValid = false
				break
			end
		end
		if isValid then
			return index
		end
		index = index + 1
	end
	print("[WARNING] No empty slot found on table " .. tableHandle)
	return -1
end

--[[
Function Name: putObjectOnTable
	- Input:
		- objName: Name of the object
		- dropPos: Position where the object is to be dropped (w.r.t. "tableHandle")
		- tableHandle: Handle of the table where the object is to be placed
	- Output:
	- Modifies:
	- Logic:
		- Picks the object
		- Transforms the dropPos to be in reference frame of DairyBikeChassis
		- Places the object at dropPos
		- Moves ArmTip at default Z value
	- Example Call:
		- putObjectOnTable('Paneer0', DropPosTable[1], TableHandles[1])
--]]
function putObjectOnTable(objName, dropPos, tableHandle)
	local objHandle = sim.getObjectHandle(objName)

	-- PICK
	goToObject(objHandle)

	-- signal pick-link to pick-up the object
	sim.setStringSignal("MTB_PickUpObject", objName)
	sim.setIntegerSignal("MTB_PickUpOrDropReady", 1)
	sim.waitForSignal("MTB_PickUpConnected")
	sim.clearIntegerSignal("MTB_PickUpConnected")
	-- indicate to reaction wheel that a package is attached to the arm
	signalArmHasPackage(1)

	-- PLACE
	-- transform dropPos (newDropPos is w.r.to the Chassis)
	local objMatrix = sim.getObjectMatrix(tableHandle, ChassisHandle)
	local newDropPos = sim.multiplyVector(objMatrix, dropPos)

	moveToZ(TipZPosWithObject)
	goToPlacePos(objHandle, newDropPos)

	-- signal pick-link to drop the object
	sim.setIntegerSignal("MTB_PickUpOrDropReady", 2)
	sim.waitForSignal("MTB_PickUpDropped")
	sim.clearIntegerSignal("MTB_PickUpDropped")

	-- indicate to reaction wheel that a package is removed from the arm
	signalArmHasPackage(0)
	local objMass = sim.getShapeMassAndInertia(objHandle)
	signalChangeInMassOnBody(-objMass)

	-- move tip to default Z position
	moveToZ(TipDefaultZPos)
end

--[[
Function Name: putObjectInStorageArea
	- Input:
		- objName: Name of the object to be placed
		- index: Index in the storage-area where the "objName" is to be placed
	- Output:
	- Modifies:
	- Logic:
		- Goes to the object's position (with tip just above the object)
		- Signals PickLink to conenct to the object
		- Also sets ArmHasSignal and ExtraMassOnBody signals for the reaction wheel
		- Goes to the StorageArea[index]
		- Signals the PickLink to drop the object
		- Sets ArmHasSignal to 0
	- Example Call:
		- putObjectInStorageArea('Paneer0', 3)
--]]
function putObjectInStorageArea(objName, index)
	local objHandle = sim.getObjectHandle(objName)

	-- PICK
	goToObject(objHandle)

	-- signal pick-link to pick-up the object
	sim.setStringSignal("MTB_PickUpObject", objName)
	sim.setIntegerSignal("MTB_PickUpOrDropReady", 1)
	sim.waitForSignal("MTB_PickUpConnected")
	sim.clearIntegerSignal("MTB_PickUpConnected")

	-- indicate to reaction wheel that a package is attached to the arm
	signalArmHasPackage(1)
	local objMass = sim.getShapeMassAndInertia(objHandle)
	signalChangeInMassOnBody(objMass)

	-- PLACE
	goToPlacePos(objHandle, StorageAreaPos[index])

	-- signal pick-link to drop the object
	sim.setIntegerSignal("MTB_PickUpOrDropReady", 2)
	sim.waitForSignal("MTB_PickUpDropped")
	sim.clearIntegerSignal("MTB_PickUpDropped")

	-- indicate to reaction wheel that a package is removed from the arm
	signalArmHasPackage(0)

	-- move pick-link to default position
	moveToZ(TipDefaultZPos)
end

--[[
Function Name: goToObject
	- Input:
		- objHandle: Handle of the object to be picked
	- Output:
	- Modifies:
	- Logic:
		- First move the tip to default position in Z axis (to avoid collisions)
		- Move to the x,y co-ordinates of the object
		- Lower the arm
		- Correct the x,y co-orindates of the tip for good measure
	- Example Call:
		- goToObject(sim.getObjectHandle('Paneer0'))
--]]
function goToObject(objHandle)
	-- make sure the PickLink is not extended
	moveToZ(TipDefaultZPos)

	-- move to x, y co-ordinates of the object
	local dist = getDistXYFromObj(objHandle)
	-- the loop would be required if the movement of arm causes
	-- a large tilt
	while dist > Tolerances["xy_dist"] do
		local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
		local basePos = sim.getObjectPosition(Axis1Handle, ChassisHandle)
		moveToXY(objPos[1] - basePos[1], objPos[2] - basePos[2])
		dist = getDistXYFromObj(objHandle)
	end

	-- lower the arm
	adaptiveMoveToZ(objHandle)
	local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
	local basePos = sim.getObjectPosition(Axis1Handle, ChassisHandle)
	-- correct arm-tip's x,y co-ordinates again (for a good measure)
	moveToXY(objPos[1] - basePos[1], objPos[2] - basePos[2])
end

--[[
Function Name: goToPlacePos
	- Input:
		- objHandle: Handle of the object to be placed
		- placePos: Position where the object is to be placed (w.r.t. DairyBikeChassis)
	- Output:
	- Modifies:
	- Logic:
		- Move tip to default Z position (to avoid collisions)
		- Correct orientation of the object to be placed (make one of its side parallel DairyBikeChassis)
		- Move to the x,y co-ordinates of the place position
		- Correct orientation of the object once again
		- Lower the object
		- Correct x,y co-ordinates for a good measure
	- Example Call:
		- goToPlacePos(sim.getObjectHandle('Paneer0'), tableDropPos1WRTChassis)
--]]
function goToPlacePos(objHandle, placePos)
	-- move to default z and start with a correct orientation
	moveToZ(TipZPosWithObject)
	correctOrientation(objHandle)

	-- move to x, y co-ordinates of place position
	local dist = getDistXY_WRT_Chassis(placePos[1], placePos[2])
	while dist > Tolerances["xy_dist"] do
		local x, y = getTargetTipXYForPlace(objHandle, placePos)
		moveToXY(x, y)
		dist = getDistXY_WRT_Chassis(placePos[1], placePos[2])
	end

	-- correct x,y co-ordinates again if needed
	-- correction might be required if the arm's movement causes a large tilt
	dist = getDistXY_WRT_Chassis(placePos[1], placePos[2])
	while dist > Tolerances["xy_dist"] do
		local x, y = getTargetTipXYForPlace(objHandle, placePos)
		moveToXY(x, y)
		dist = getDistXY_WRT_Chassis(placePos[1], placePos[2])
	end

	-- correct the orientation
	-- (to make sure that the orientation of the object does not cause collision)
	correctOrientation(objHandle)

	-- lower the object
	local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
	local height = getHeightWithClearance(placePos[3] - objPos[3], PlaceClearance)
	-- sim.setJointTargetVelocity(revoluteJoint, 0)
	moveToZ(height)

	-- correct x,y co-ordinates once again
	local x, y = getTargetTipXYForPlace(objHandle, placePos)
	moveToXY(x, y)
end

--[[
Function Name: correctOrientation
	- Input:
		- objHandle: Handle of the object whose orientation is to be corrected
	- Output:
	- Modifies:
	- Logic:
		- Rotates revoluteJoint such that the a side of "objHandle"
		becomes parallel to DairyBikeChassis
	- Example Call:
		correctOrientation(sim.getObjectHandle('Paneer0'))
--]]
function correctOrientation(objHandle)
	local orientation = sim.getObjectOrientation(objHandle, ChassisHandle)
	local gamma = orientation[3]
	local revAngle = (gamma) % (math.pi / 2)
	goToJointPosRev(-revAngle)
end

--[[
Function Name: moveToXY
	- Input:
		- x: x co-ordinate of the target position
		- y: y co-ordinate of the target position
		- thetaTolerance: tolerance (in radians) for joint values of axis1 & axis2
	- Output:
	- Modifies:
	- Logic:
		- Calculates thetas (joint-values for axis1 and axis2) for x, y
		- Calls goToJointPosXY to reach the desired joint values
	- Example Call:
		- moveToXY(x, y, Tolerances['xy-theta'])
--]]
function moveToXY(x, y, thetaTolerance)
	local theta1, theta2 = getThetasFromXY(x, y)
	goToJointPosXY(theta1, theta2, thetaTolerance)
end

--[[
Function Name: moveToZ
	- Input:
		- targetZ: z co-ordinate of the target position
		- zTolerance: Tolerance on Z axis
	- Output:
	- Modifies:
	- Logic:
		- Implements a proportional controller to reach "targetZ"
	- Example Call:
		- moveToZ(dropPosZ, Tolerances['z'])
--]]
function moveToZ(targetZ, zTolerance)
	local jointHandle = Axis4Handle
	local currZ = sim.getJointPosition(jointHandle)

	if zTolerance == nil then
		zTolerance = Tolerances["z"]
	end
	while (math.abs(targetZ - currZ) > (zTolerance)) do
		local diff = (targetZ - currZ)
		local absDiff = math.abs(diff)
		local diffSign = diff / absDiff
		local vel = KP["z"] * diffSign * (absDiff) ^ (0.5)
		sim.setJointTargetVelocity(jointHandle, vel)
		currZ = sim.getJointPosition(jointHandle)
	end
	sim.setJointTargetVelocity(jointHandle, 0)
end

--[[
Function Name: adaptiveMoveToZ
	- Input:
		- objHandle: Handle of the object whose top's z co-ordinate (pos[3]+objHeight/2+clearance)
			is to be reached by the arm-tip
		- zTolerance: Tolerance for P-controller
	- Output:
	- Modifies:
	- Logic:
		- A P-controller similar to moveToZ
		- The targetZ is updated in each iteration of the for loop
			(to compensate the jitters in bike's balancing)
	- Example Call:
		- adaptiveMoveToZ(sim.getObjectHandle('Cheese0'), Tolerances['z'])
--]]
function adaptiveMoveToZ(objHandle, zTolerance)
	local jointHandle = Axis4Handle
	local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
	local tipPos = sim.getObjectPosition(ArmTipHandle, ChassisHandle)

	local diff = getHeightWithClearance(objPos[3] - tipPos[3], PickupClearance)

	-- go to appropriate height using the proximity sensor
	local isObjDetected, dist = getDistanceFromProximitySensor(objHandle)
	if isObjDetected then
		diff = -(dist - PickupClearance)
	end

	if zTolerance == nil then
		zTolerance = Tolerances["z"]
	end

	while (math.abs(diff) > (zTolerance)) do
		local absDiff = math.abs(diff)
		local diffSign = diff / absDiff
		local vel = KP["z"] * diffSign * (absDiff) ^ (0.5)
		sim.setJointTargetVelocity(jointHandle, vel)
		-- update curr and target both!
		diff = getHeightWithClearance(objPos[3] - tipPos[3], PickupClearance)
		isObjDetected, dist = getDistanceFromProximitySensor(objHandle)
		if isObjDetected then
			diff = -(dist - PickupClearance)
		end
	end
	sim.setJointTargetVelocity(jointHandle, 0)
end

--[[
	Function Name: getThetasFromXY
	- Input:
		- x: Target x co-ordinate (w.r.t. Axis1)
		- x: Target y co-ordinate (w.r.t. Axis1)
	- Output:
		- theta1: Joint-value for Axis1 to reach the target (in radians)
		- theta2: Joint-value for Axis2 to reach the target (in radians)
	- Modifies:
	- Logic:
		- Geometrically finds joint-values of Axis1 and Axis2 to reach (x, y)
		- Tries to avoid solutions in with joint-value > 160 or < -160
		- In case of multiple valid soltions, returns the closest one
	- Example Call:
		- local theta1, theta2 = getThetasFromXY(x, y)
	--]]
function getThetasFromXY(x, y)
	local xSq = x * x
	local ySq = y * y

	local sqrtExpr = (-(250000 * xSq + 250000 * ySq - 2601) * (250000 * xSq + 250000 * ySq - 106929))
	if sqrtExpr < 0 then
		print("[WARNING] No feasible solutions to reach the object!")
		return 0, 0
	end
	sqrtExpr = math.sqrt(sqrtExpr)

	-- equations are obtained using MATLAB
	local theta1Alpha =
		2 *
		math.atan(
			(138000 * y - (2601 * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601) +
				(250000 * xSq * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601) +
				(250000 * ySq * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601)) /
				(250000 * xSq + 138000 * x + 250000 * ySq - 16677)
		)
	local theta1Beta =
		2 *
		math.atan(
			(138000 * y + (2601 * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601) -
				(250000 * xSq * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601) -
				(250000 * ySq * sqrtExpr) / (250000 * xSq + 250000 * ySq - 2601)) /
				(250000 * xSq + 138000 * x + 250000 * ySq - 16677)
		)

	local theta2Alpha = -2 * math.atan(sqrtExpr / (250000 * xSq + 250000 * ySq - 2601))
	local theta2Beta = 2 * math.atan(sqrtExpr / (250000 * xSq + 250000 * ySq - 2601))

	if
		((theta1Alpha < math.rad(160) and theta1Alpha > -math.rad(160)) and
			(theta2Alpha < math.rad(160) and theta2Alpha > -math.rad(160))) and
			((theta1Beta < math.rad(160) and theta1Beta > -math.rad(160)) and
				(theta2Beta < math.rad(160) and theta2Beta > -math.rad(160)))
	 then
		-- 2 valid solutions -> return the solution closest to current joint values
		local currTheta1 = sim.getJointPosition(Axis1Handle)
		local currTheta2 = sim.getJointPosition(Axis2Handle)
		local alphaDiff = math.max(math.abs(currTheta1 - theta1Alpha), math.abs(currTheta2 - theta2Alpha))
		local betaDiff = math.max(math.abs(currTheta1 - theta1Beta), math.abs(currTheta2 - theta2Beta))
		if alphaDiff < betaDiff then
			return theta1Alpha, theta2Alpha
		end
		return theta1Beta, theta2Beta
	end

	if
		((theta1Alpha < math.rad(160) and theta1Alpha > -math.rad(160)) and
			(theta2Alpha > math.rad(160) and theta2Alpha < -math.rad(160)))
	 then
		return theta1Alpha, theta2Alpha
	end

	return theta1Beta, theta2Beta
end

--[[
Function Name: goToJointPosXY
	- Input:
		- targetTheta1: Target joint value for Axis1 (in radians)
		- targetTheta2: Target joint value for Axis2 (in radians)
		- thetaTolerance: Tolerance for the P-controller in radians
	- Output:
	- Modifies:
	- Logic:
		- P-controller for Axis1 & Axis2
		- Avoids joint-values > 160 or < -160
	- Example Call:
		- goToJointPosXY(targetTheta1, targetTheta2, Tolerances['xy-theta'])
--]]
function goToJointPosXY(targetTheta1, targetTheta2, thetaTolerance)
	targetTheta1 = targetTheta1 + math.rad(160)
	targetTheta2 = targetTheta2 + math.rad(160)

	local currTheta1 = sim.getJointPosition(Axis1Handle) + math.rad(160)
	local currTheta2 = sim.getJointPosition(Axis2Handle) + math.rad(160)

	if thetaTolerance == nil then
		thetaTolerance = Tolerances["xy_angle"]
	end

	local error = math.max(math.abs(targetTheta1 - currTheta1), math.abs(targetTheta2 - currTheta2))
	while (error > thetaTolerance) do
		-- controller for axis1
		local diff = (targetTheta1 - currTheta1)
		local absDiff = math.abs(diff)
		if absDiff > thetaTolerance then
			local diffSign = diff / absDiff
			local vel = 4 * diffSign * (absDiff) ^ (0.5)
			sim.setJointTargetVelocity(Axis1Handle, vel)
		end

		-- controller for axis2
		diff = (targetTheta2 - currTheta2)
		absDiff = math.abs(diff)
		if absDiff > thetaTolerance then
			local diffSign = diff / absDiff
			local vel = 4 * diffSign * (absDiff) ^ (0.5)
			sim.setJointTargetVelocity(Axis2Handle, vel)
		end

		-- update errors
		currTheta1 = sim.getJointPosition(Axis1Handle) + math.rad(160)
		currTheta2 = sim.getJointPosition(Axis2Handle) + math.rad(160)
		error = math.max(math.abs(targetTheta1 - currTheta1), math.abs(targetTheta2 - currTheta2))
	end
	sim.setJointTargetVelocity(Axis1Handle, 0)
	sim.setJointTargetVelocity(Axis2Handle, 0)
end

--[[
Function Name: goToJointPosRev
	- Input:
		- targetTheta: Target joint-value for the revolute-joint
		- thetaTolerance: Tolerance for P-controller
	- Output:
	- Modifies:
		- A P-controller for revolute-joint
	- Logic:
	- Example Call:
		- goToJointPosRev(objPos[3], Tolerance['rev'])
--]]
function goToJointPosRev(targetTheta, thetaTolerance)
	local currTheta = sim.getJointPosition(RevJointHandle)

	if thetaTolerance == nil then
		thetaTolerance = Tolerances["rev"]
	end

	while (math.abs(targetTheta - currTheta) > thetaTolerance) do
		local diff = (targetTheta - currTheta)
		local absDiff = math.abs(diff)
		local diffSign = diff / absDiff
		local vel = KP["rev"] * diffSign * (absDiff)
		sim.setJointTargetVelocity(RevJointHandle, vel)
		currTheta = sim.getJointPosition(RevJointHandle)
	end
	sim.setJointTargetVelocity(RevJointHandle, 0)
end

--[[
Function Name: getTargetTipXYForPlace
	- Input:
		- objHandle: Handle of the picked object
			(attached to PickLink)
		- pos: Target x,y co-ordinates (w.r.t. DairyBikeChassis)
	- Output:
		- x: x-co-ordinate of ArmTip such that "objHandle's" COM's x co-ordinates is at pos[1]
		- y: y-co-ordinate of ArmTip such that "objHandle's" COM's y co-ordinates is at pos[2]
	- Modifies:
	- Logic:
		- Based on the difference between (x,y) of ArmTip and (x, y) of object's COM
			calculates ArmTip's position such that object's COM reaches (pos[1], pos[2])
	- Example Call:
		- local x, y = getTargetTipXYForPlace(sim.getObjectHandle('Paneer0'), dropPosTable1WRTChassis)
--]]
function getTargetTipXYForPlace(objHandle, pos)
	local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
	local tipPos = sim.getObjectPosition(ArmTipHandle, ChassisHandle)

	local deltaX = objPos[1] - tipPos[1]
	local deltaY = objPos[2] - tipPos[2]

	local basePos = sim.getObjectPosition(Axis1Handle, ChassisHandle)

	local x = pos[1] - basePos[1] - deltaX
	local y = pos[2] - basePos[2] - deltaY

	return x, y
end

--[[
Function Name: getDistanceFromProximitySensor
	- Input:
		- objHandle: Handle of the object whose distance is to be measured
	- Output:
		- isObjectDetected: true if "objHandle" is detected by the proximity sensor
		- distance: distance of the object from the proximity sensor if it's detected, -1 otherwise
	- Modifies:
	- Logic:
		- Uses sim.handleProximitySensor
	- Example Call:
		- local dist = getDistanceFromProximitySensor(sim.getObjectHandle('Paneer0'))
--]]
function getDistanceFromProximitySensor(objHandle)
	local _, distance, _, detectedObjectHandle, _ = sim.handleProximitySensor(ProximitySensorHandle)
	if objHandle == detectedObjectHandle then
		return true, distance
	end
	return false, -1
end

--[[
Function Name: getDistXYFromObj
	- Input:
		- objHandle: Handle of the object from which the distance is to be measured
	- Output:
		- distance: Distance of "objHandle" from ArmTip
	- Modifies:
	- Logic:
		- Calls getDistXY_WRT_Chassis with object's (x,y) w.r.t. DairyBikeChassis
	- Example Call:
		- local dist = getDistXYFromObj(sim.getObjectHandle('Paneer0'))
--]]
function getDistXYFromObj(objHandle)
	local objPos = sim.getObjectPosition(objHandle, ChassisHandle)
	return getDistXY_WRT_Chassis(objPos[1], objPos[2])
end

--[[
Function Name: getDistXY_WRT_Chassis
	- Input:
		- x: x co-ordinate w.r.t DairyBikeChassis
		- y: y co-ordinate w.r.t DairyBikeChassis
	- Output:
		- distance: Distance of (x,y) from ArmTip
	- Modifies:
	- Logic:
		- Computes Euclidean distance of (x,y) from the ArmTip
	- Example Call:
		- local distance = getDistXY_WRT_Chassis(x, y)
--]]
function getDistXY_WRT_Chassis(x, y)
	local tipPos = sim.getObjectPosition(ArmTipHandle, ChassisHandle)
	local basePos = sim.getObjectPosition(Axis1Handle, ChassisHandle)
	x = x - basePos[1]
	y = y - basePos[2]
	local tipX = tipPos[1] - basePos[1]
	local tipY = tipPos[2] - basePos[2]
	local dist = math.sqrt((tipX - x) ^ 2 + (tipY - y) ^ 2)
	return dist
end

--[[
Function Name: getHeightWithClearance
	- Input:
		- diff: difference of height to be covered
		- clearance: clearance value for height
	- Output:
		- diff that incorporates clearance
	- Modifies:
	- Logic:
	- Example Call:
		- local newHeight = getHeightWithClearance(diff, clearance)
--]]
function getHeightWithClearance(diff, clearance)
	local absDiff = math.abs(diff)
	local diffSign = diff / absDiff
	absDiff = absDiff - ObjHeight / 2 - clearance
	return absDiff * diffSign
end

--[[
Function Name: signalArmHasPackage
	- Input:
		- ArmHasPackage: 1 if an object is attached to the PickLink, 0 otherwise
	- Output:
	- Modifies:
	- Logic:
		- Sets "ArmHasPackage" signal for the reaction wheel
	- Example Call:
		- signalArmHasPackage(1)
--]]
function signalArmHasPackage(ArmHasPackage)
	sim.setIntegerSignal("ArmHasPackage", ArmHasPackage)
end

--[[
Function Name: signalChangeInMassOnBody
	- Input:
		- extraMass: Additional mass added to (+ve)/removed from (-ve) the Dairy Bike
	- Output:
	- Modifies:
	- Logic:
		- Sets "ChangeInMassOnBody" signal for the reaction wheel
	- Example Call:
		- signalChangeInMassOnBody(+2)
--]]
function signalChangeInMassOnBody(extraMass)
	sim.setFloatSignal("ChangeInMassOnBody", extraMass)
end
