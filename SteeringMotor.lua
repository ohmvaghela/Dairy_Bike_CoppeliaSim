function sysCall_init()
    -- Object handles
    Body = sim.getObjectHandle("DairyBikeChassis")
    SteeringMotor = sim.getObjectHandle("SteeringMotor")
    RearMotor = sim.getObjectHandle("RearMotor")
    FrontMotor = sim.getObjectHandle("FrontMotor")
    DefaultPath = sim.getObjectHandle("Path")
    FrontWheel = sim.getObjectHandle("FrontWheel_Respondable")
    FrontFollower = sim.getObjectHandle("FrontFollower")
    COMFollower = sim.getObjectHandle("COMFollower")
    RearFollower = sim.getObjectHandle("RearFollower")

    -- Destinations
    Destinations = {
        sim.getObjectHandle("Loading_Point"),
        sim.getObjectHandle("Delivery_Point_House1"),
        sim.getObjectHandle("Delivery_Point_School"),
        sim.getObjectHandle("Delivery_Point_Dhaba"),
        sim.getObjectHandle("Delivery_Point_Hospital"),
        sim.getObjectHandle("Delivery_Point_House2"),
        sim.getObjectHandle("Path__ctrlPt52"),
        sim.getObjectHandle("Delivery_Point_DairyFarm"),
        sim.getObjectHandle("Start_Stop_Point")
    }
    StopPoint = Destinations[#Destinations]

    -- Path preprocessing
    local path_data = sim.unpackDoubleTable(sim.readCustomDataBlock(DefaultPath, "PATH"))
    local path_pose = Matrix(#path_data // 7, 7, path_data)
    DefaultPathPositions = path_pose:slice(1, 1, path_pose:rows(), 3):data() -- Array of (x, y, z) of the path points wrt path
    DefaultPathLengths = sim.getPathLengths(DefaultPathPositions, 3) -- Array of the lengths of each path point from the starting point

    -- Bike length
    local front_follower = sim.getObjectPosition(FrontFollower, DefaultPath)
    local com_follower = sim.getObjectPosition(COMFollower, DefaultPath)
    local rear_follower = sim.getObjectPosition(RearFollower, DefaultPath)
    local front_vector = Vector:new()
    front_vector.x = front_follower[1] - com_follower[1]
    front_vector.y = front_follower[2] - com_follower[2]
    front_vector.z = front_follower[3] - com_follower[3]
    local rear_vector = Vector:new()
    rear_vector.x = com_follower[1] - rear_follower[1]
    rear_vector.y = com_follower[2] - rear_follower[2]
    rear_vector.z = com_follower[3] - rear_follower[3]

    -- Parameters
    TControl = 5e-2     -- Time for the control loop
    DeltaL = DefaultPathLengths[2] - DefaultPathLengths[1] -- Twice the distance between the consecutive two points on the path
    
    -- The parameter of the Stanley controller
    KHeading = 1.2
    KCrossTrack = 0.9
    KSoft = 0.1
    
    -- The parameter of the reverse Stanley controller
    KReverseHeading = 2
    KReverseCrossTrack = 2

    MaxSteerAngle = math.rad(50)    -- Upper limit on steering angle
    MaxAngularSpeed = math.rad(500) -- Upper limit on angular speed of the rear motor
    MinAngularSpeed = 4             -- Lower limit on angular speed of the rear motor (for longitudinal controller)

    -- Predictive control paramters
    TPredict = 3 * TControl         -- Time gap between each prediction state
    KPredictions = {0.500000, 0.254330, 0.129368, 0.065804, 0.033472, 0.017026} -- Weights of the predictions
    NumberOfPredictions = #KPredictions -- Number of predictions
    
    LFront = front_vector:mag() -- Length from COMFollower to the FrontFollower
    LRear = rear_vector:mag()   -- Length from COMFollower to the RearFollower
    Acceleration = 3            -- For RearMotor torque of 20 Nm
    WheelRadius = 0.15          -- Radius of the rear wheel
    ThresholdStopingDistance = 0.03     -- Threshold distance for enabling the stop controller
    ReverseHeadingErrorThreshold = math.rad(150)    -- Threshold heading error to enable the reverse controller
    BrakingDistance = 0.4               -- Threshold distance to activate the brake controller
    MaxSpeed = MaxAngularSpeed * WheelRadius    -- Maximum allowed speed of the bike
    -- Brake parameters
    -- They are derived from the brake function
    -- Our brake function is:
    -- Speed = BrakingSharpness * (distance + BrakingSmoothness)^0.5 - BrakingSharpness * BrakingSmoothness
    BrakingSharpness = MaxSpeed / (MaxSpeed / Acceleration - BrakingDistance) ^ 0.5      
    BrakingSmoothness = (BrakingSharpness / (2 * Acceleration)) ^ 2 - BrakingDistance
    -- Check to make sure that BrakingDistance is under limits and it
    -- generates the valid braking paramters
    if
        (BrakingSharpness < 0) or (BrakingSmoothness < 0) or (BrakingSharpness ~= BrakingSharpness) or
            (BrakingSmoothness ~= BrakingSmoothness)
     then
        print("Error in braking paramters")
        sim.stopSimulation()
    end

    -- Intializatio
    LastTime = sim.getSimulationTime()
    CurrentTime = sim.getSimulationTime()
    DestinationSignal = "destinationSignal"     -- Signal which passes the name of destination to the manipulator
    MoveSignal = "readyToMove"                  -- Signal which receives the move command from the manipulator
    ControlState = ""                           -- Decides the controller to be enabled
    PathState = "Default"                       -- Tells if the Path is newly created or default path is being used
    -- Path variables which will be used by the function for the calculations
    -- This variables will be updated according to the destination
    Path = nil
    PathLengths = nil
    PathPositions = nil

    -- Coroutines
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

function coroutineMain()
    while true do
        CurrentTime = sim.getSimulationTime()
        if (CurrentTime - LastTime > (TControl - 1e-4)) then
            -- Update all the variables
            update()

            -- High level controller
            local desired_angular_speed, desired_steering_angle = hybrid_controller()

            -- Low level controller
            rotate_steering(desired_steering_angle)
            generate_angular_speed(desired_angular_speed)

            -- Update time
            LastTime = CurrentTime
        end
    end
end

--[[
Function Name: hybrid_controller
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Type: Float
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Type: Float
            - Desired steering angle of the steering motor
Logic:
    - According to the value of ControlState, it selects the controller
Example Call:
```
hybrid_controller()
```
--]]
function hybrid_controller()
    local desired_angular_speed, desired_steering_angle

    if ControlState == "Forward" then
        desired_angular_speed, desired_steering_angle = predictive_controller()
    elseif ControlState == "Reverse" then
        desired_angular_speed, desired_steering_angle = reverse_controller()
    elseif ControlState == "Slope" then
        desired_angular_speed, desired_steering_angle = slope_controller()
    elseif ControlState == "ForwardBrake" then
        desired_angular_speed, desired_steering_angle = forward_brake_controller()
    elseif ControlState == "ReverseBrake" then
        desired_angular_speed, desired_steering_angle = reverse_brake_controller()
    elseif ControlState == "Stop" then
        desired_angular_speed, desired_steering_angle = stop_controller()
    else
        print("Error in hybrid controller: Unknown state found")
        sim.stopSimulation()
    end

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name: stop_controller
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Type: Float
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Type: Float
            - Desired steering angle of the steering motor
Logic:
    - Stops the rear and steering motor
    - Turns the color of StopPoint to the black in the end
Example Call:
```
stop_controller()
```
--]]
function stop_controller()
    local desired_angular_speed = 0
    local desired_steering_angle = 0

    -- If covered all the destinations turn the start_stop_point black
    if #Destinations == 0 then
        sim.setShapeColor(StopPoint, "", sim.colorcomponent_ambient_diffuse, {0, 0, 0})
    end

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name: reverse_brake
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Type: Float
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Type: Float
            - Desired steering angle of the steering motor
Logic:
        - Produces the desired speed of the bike in backward direcion
        - Its target is to take the FrontFollower of the bike to the destination
        - Calculates the desired angular speed according to the braking function
        - Calculates the desired steering angle using longitudinal controller
Example Call:
```
reverse_brake_controller()
```
--]]
function reverse_brake_controller()
    local desired_angular_speed, desired_steering_angle
    -- Get heading error
    local rear_position_vector = Vector:new(sim.getObjectPosition(RearFollower, Path))
    local rear_heading_error = get_heading_error(rear_position_vector)
    local bike_speed = get_bike_speed()

    -- Get reverse heading error
    local heading_error_mag = math.abs(rear_heading_error)
    local heading_error_sign = rear_heading_error / heading_error_mag
    local reverse_heading_error = -1 * heading_error_sign * (math.pi - heading_error_mag)

    -- Get cross track error
    local reverse_cross_track_error = -1 * get_cross_track_error(rear_position_vector)

    if (#Destinations ~= 0) then
        local destination = Destinations[1]
        local distance_along_path = get_distance_along_path(FrontFollower, destination)
        desired_angular_speed =
            -(BrakingSharpness * (distance_along_path + BrakingSmoothness) ^ 0.5 -
            BrakingSharpness * BrakingSmoothness ^ 0.5) / WheelRadius

        desired_steering_angle = reverse_lateral_controller(reverse_cross_track_error, reverse_heading_error, bike_speed)
    else
        print("Error: Brakes are required without destination")
        sim.stopSimulation()
    end

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name: reverse_controller
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Desired steering angle of the steering motor
Logic:
    - Calculates the desired steering angle using the reverse lateral controller
    - Finds the desired angular speed using the longitudunal controller
Example Call:
```
reverse_controller()
```
--]]
function reverse_controller()
    local desired_angular_speed, desired_steering_angle
    -- Get heading error
    local rear_position_vector = Vector:new(sim.getObjectPosition(RearFollower, Path))
    local rear_heading_error = get_heading_error(rear_position_vector)
    local bike_speed = get_bike_speed()

    -- Get reverse heading error
    local heading_error_mag = math.abs(rear_heading_error)
    local heading_error_sign = rear_heading_error / heading_error_mag
    local reverse_heading_error = -1 * heading_error_sign * (math.pi - heading_error_mag)

    -- Get cross track error
    local reverse_cross_track_error = -1 * get_cross_track_error(rear_position_vector)

    -- Basic controller
    desired_steering_angle = reverse_lateral_controller(reverse_cross_track_error, reverse_heading_error, bike_speed)
    desired_angular_speed = -longitudinal_controller(get_steering_angle())

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name: slope_controller()
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Desired steering angle of the steering motor
Logic:
    - In slope, predictive controller doesn't perform good because velocity isn't in complete control
    because of which precition of future states fail
    - Lateral controller is used to find out the desired steering angle
    - Longitudinal controller is used to calculate the desired angular speed
Example Call:
```
slope_controller()
```
--]]
function slope_controller()
    -- Get heading and cross track error
    local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower, Path))
    local heading_error = get_heading_error(com_position_vector)
    local cross_track_error = get_cross_track_error(com_position_vector)
    local bike_speed = get_bike_speed()

    -- Basic Stanley controller
    local desired_steering_angle = lateral_controller(cross_track_error, heading_error, bike_speed)
    local desired_angular_speed = longitudinal_controller(get_steering_angle())

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name:
Input:
Output:
    - Returns:
        - desired_angular_speed:
            - Desired angular speed of the rear motor
        - desired_steering_angle:
            - Desired steering angle of the steering motor
Logic:
    - Produces desired speed of the bike in forward direction
    - Its target is to take the RearFollower of the bike to the destination
    - Calculates the desired angular speed according to the braking function
    - Calculates the desired steering angle using longitudinal controller
Example Call:
```
forward_brake_controller()
```
--]]
function forward_brake_controller()
    local desired_angular_speed, desired_steering_angle
    -- Get heading error, cross track error, and bike speed
    local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower, Path))
    local heading_error = get_heading_error(com_position_vector)
    local cross_track_error = get_cross_track_error(com_position_vector)
    local bike_speed = get_bike_speed()

    if (#Destinations ~= 0) then
        local destination = Destinations[1]
        local distance_along_path = get_distance_along_path(RearFollower, destination)

        -- Basic controller
        desired_angular_speed =
            (BrakingSharpness * (distance_along_path + BrakingSmoothness) ^ 0.5 -
            BrakingSharpness * BrakingSmoothness ^ 0.5) /
            WheelRadius
        desired_steering_angle = lateral_controller(cross_track_error, heading_error, bike_speed)
    else
        print("Error: Brakes are required without destination")
        sim.stopSimulation()
    end

    return desired_angular_speed, desired_steering_angle
end

--[[
Function Name: update
Input:
Output:
    - Returns:
    - Modifies:
        - Path:
            - Path to be followed
        - PathPosition:
            - Table of positions of points present in the path
        - PathLengths:
            - Table of length of points from the starting point of the path
        - Destinations:
            - The list of all the destinations in the order to be covered
            - The destination at the top of this list is always the current destination
        - PathState:
            - Tells whether to use the default path or to create a new path for the current destination
        - ControlState:
            - Tells which controller to choose
Logic:
    - All the update work is done in this function
    - It updates path, destination, state of the controller
Example Call:
```
update()
```
--]]
function update()
    -- Update path, destination, path state
    delete_path()
    update_destination()
    update_path_state()
    update_path()

    -- Update the control states
    update_control_state()
end

--[[
Function Name: update_path
Input:
Output:
    - Returns:
    - Modifies:
        - Path:
            - Path to be followed
        - PathPosition:
            - Table of positions of points present in the path
        - PathLengths:
            - Table of length of points from the starting point of the path
Logic:
    - Updates the path to be followed according to the destination
    - This is to avoid random behaviour of the bike where 3 paths merge
    - This also helps in reversing of the bike
Example Call:
```
update_path()
```
--]]
function update_path()
    -- Create new path if the Path variable is nil
    if (Path == nil) and (#Destinations~=0) then
        local destination = Destinations[1]
        local path_points_list = get_path_point_list(destination)
        if #path_points_list == 0 then
            Path = DefaultPath
            PathLengths = DefaultPathLengths
            PathPositions = DefaultPathPositions
        else
            create_new_path(path_points_list)
        end
    elseif (Path == nil) and (#Destinations == 0) then
        Path = DefaultPath
        PathLengths = DefaultPathLengths
        PathPositions = DefaultPathPositions
    end
end

--[[

Function Name: create_new_path
Input:
    - path_point_list:
        - Type: List of integers
        - It contains the number of the control points in the order in which
        they should be connected to make the path
Output:
    - Returns:
    - Modifies:
        - Path
            - Path to be followed
        - PathPosition:
            - Positions of points present in the path
        - PathLengths:
            - Length of points from the starting point of the path
Logic:
    - Retrieve the positions of the control points mentioned in the path_point_list
    - Create a new path from those positions
Example Call:
```
create_new_path(path_points_list)
```
--]]
function create_new_path(path_points_list)
    -- Create path object
    local control_points = points_to_pose(path_points_list)
    Path = sim.createPath(control_points, 0, 1000, 0.7)

    -- Set other global variables realted to path
    local path_data = sim.unpackDoubleTable(sim.readCustomDataBlock(Path, "PATH"))
    local path_pose = Matrix(#path_data // 7, 7, path_data)
    PathPositions = path_pose:slice(1, 1, path_pose:rows(), 3):data()
    PathLengths = sim.getPathLengths(PathPositions, 3)
end

--[[
Function Name: points_to_pose
Input:
    - path_point_list:
        - List of integers
        - It contains the number of the control points in the order in which
        they should be connected to make the path
Output:
    - Returns:
        - points_pose_list:
            - List of positions of the control points whose number is in the path_point_list
Logic:
    - Retrieve control points handles
    - Get object positions of from those handles and put it on the table
Example Call:
```
point_to_pose(path_points_list)
```
--]]
function points_to_pose(path_points_list)
    local points_pose_list = {}
    local pose
    local point_number
    local point
    local point_name = ""
    for i = 1, #path_points_list, 1 do
        point_number = path_points_list[i]
        point_name = "Path__ctrlPt" .. point_number
        point = sim.getObjectHandle(point_name)
        pose = sim.getObjectPose(point, -1)
        for j = 1, 7, 1 do
            points_pose_list[7 * (i - 1) + j] = pose[j]
        end
    end

    return points_pose_list
end

--[[
Function Name: update_path_state
Input:
Output:
    - Returns:
    - Modifies:
        - PathState:
            - Stores whether the current path is newly created or the default
            is used
Logic:
    - According to the destination, it selects whether to follow the default path
    or create a new one and follow
    - When the default path to the destination has a point where two path lines
    intersects, the PathState for those destination is set to be "New"
Example Call:
```
update_path_state()
```
--]]
function update_path_state()
    if (ControlState == "Stop") and (#Destinations ~= 0) then
        local destination = Destinations[1]
        if sim.getObjectName(destination) == "Loading_Point" then
            PathState = "Default"
        elseif sim.getObjectName(destination) == "Delivery_Point_House1" then
            PathState = "Default"
        elseif sim.getObjectName(destination) == "Delivery_Point_School" then
            PathState = "Default"
        elseif sim.getObjectName(destination) == "Delivery_Point_Dhaba" then
            PathState = "New"
        elseif sim.getObjectName(destination) == "Delivery_Point_Hospital" then
            PathState = "New"
        elseif sim.getObjectName(destination) == "Delivery_Point_House2" then
            PathState = "New"
        elseif sim.getObjectName(destination) == "Path__ctrlPt52" then
            PathState = "New"
        elseif sim.getObjectName(destination) == "Delivery_Point_DairyFarm" then
            PathState = "New"
        elseif sim.getObjectName(destination) == "Start_Stop_Point" then
            PathState = "Default"
        else
            print('Error: Unknown destination')
            sim.stopSimulation()
        end
    elseif (ControlState == "Stop") and (#Destinations == 0) then
        PathState = "Default"
    end
end

--[[
Function Name: update_destination
Input:
Output:
    - Modifies:
        - Destinations:
            - The table which contains all the destination in the order to be covered
Logic:
    - Sends the name of the destination reached to the manipulators
    - Pops the coversed destination from the stack
Example Call:
```
update_destination()
```
--]]
function update_destination()
    if (ControlState == "Stop") and (#Destinations ~= 0) then
        local destination = Destinations[1]

        -- Send the signal
        local destination_name = sim.getObjectName(destination)
        sim.setStringSignal(DestinationSignal, destination_name)

        -- Clear the destination
        table.remove(Destinations, 1)
    end
end

--[[
Function Name: delete_path
Input:
Output:
    - Modifies:
        - Path:
            - Path to be followed
        - PathPosition:
            - Table of positions of points present in the path
        - PathLengths:
            - Table of length of points from the starting point of the path
Logic:
    - If the bike has reached and has stopped on a destination, and
    a new path was created to reach here, then delete the path which
    was created.
    - While creating a new path all the required control points were
    also created again thus it is required to delete them.
    - Retrieve the table which contains all the child control points
    present in the path and the path object itself.
    - Iterate through the table and delete each object.
Example Call:
```
delete_path()
```
--]]
function delete_path()
    -- Deleting the path once reached the destination
    if ControlState == "Stop" then
        if PathState == "New" then
            -- Delete control points and paths
            local path_tree = sim.getObjectsInTree(Path)
            for i = 1, #path_tree, 1 do
                sim.removeObject(path_tree[i])
            end
        end

        -- Debug
        Path = nil
        PathLengths = nil
        PathPositions = nil
    end
end

--[[
Function Name: get_path_point_list
Input:
    - destination:
        - This is a string
        - The name of the destination
Output:
    - Returns:
        - new_path_point_list:
            - The table of number of control points
            - The points are in order in which they should be joined to
            make a path
Logic:
    - According to the destination update the new_path_point_list
    - If the new_path_point_list is empty new path is not required
Example Call:
```
local destination = 'Delivery_Point_House1'
local new_path_point_list = get_path_point_list(destination)
```
--]]
function get_path_point_list(destination)
    local new_path_point_list = {}
    if sim.getObjectName(destination) == "Loading_Point" then
        new_path_point_list = {}
    elseif sim.getObjectName(destination) == "Delivery_Point_House1" then
        new_path_point_list = {}
    elseif sim.getObjectName(destination) == "Delivery_Point_School" then
        new_path_point_list = {}
    elseif sim.getObjectName(destination) == "Delivery_Point_Dhaba" then
        new_path_point_list = {22, 23, 24, 68, 69, 70, 71, 25, 26, 27, 28, 29, 30, 31}
    elseif sim.getObjectName(destination) == "Delivery_Point_Hospital" then
        local j = 1
        for i = 30, 47, 1 do
            new_path_point_list[j] = i
            j = j + 1
        end
    elseif sim.getObjectName(destination) == "Delivery_Point_House2" then
        new_path_point_list = {46, 48, 49, 50, 51, 52, 53}
    elseif sim.getObjectName(destination) == "Path__ctrlPt52" then
        new_path_point_list = {53, 52, 51}
    elseif sim.getObjectName(destination) == "Delivery_Point_DairyFarm" then
        new_path_point_list = {
            50, 51, 54, 55, 56, 57, 58, 59, 60, 61, 62, 73, 62, 63, 64, 65, 66, 67, 35, 34, 33, 32, 31, 30, 29, 0, 1, 2, 3
        }
    elseif sim.getObjectName(destination) == "Start_Stop_Point" then
        new_path_point_list = {}
    end

    return new_path_point_list
end
--[[
Function Name: update_control_state
Input:
Output:
    - Modifies:
        - ControlState:
            - Decides the controller to be used
Logic:
    - Checks the requirement conditions for each controller and sets the
    ControlState accordingly
Example Call:
```
```
--]]
function update_control_state()
    local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower, Path))
    local heading_vector = get_bike_heading_vector()
    local heading_error = get_heading_error(com_position_vector)

    if require_stop(heading_error) then
        ControlState = "Stop"
    elseif require_forward_brake(heading_error) then
        ControlState = "ForwardBrake"
    elseif require_reverse_brake(heading_error) then
        ControlState = "ReverseBrake"
    elseif require_slope_control(heading_vector) then
        ControlState = "Slope"
    elseif require_reverse_control(heading_error) then
        ControlState = "Reverse"
    else
        ControlState = "Forward"
    end
end

--[[
Function Name: require_reverse_control
Input:
    - heading_error:
        - The difference in the bike's heading direction and the tangent to the
        path at the point closest to the bike
Output:
    - Returns:
        - required:
            - Boolean value
            - Tells whether the reverse control is required
Logic:
    - If heading error is above the threshold, reverse control is required
Example Call:
```
local com_position_vector = sim.getObjectPosition(COMFollower, Path)
local heading_error = get_heading_error(com_position_vector)
local required = require_reverse_control(heading_error)
```
--]]
function require_reverse_control(heading_error)
    local required = false

    if math.abs(heading_error) >= ReverseHeadingErrorThreshold then
        required = true
    end

    return required
end

--[[
Function Name: require_slope_control
Input:
    - heading_error:
        - The difference in the bike's heading direction and the tangent to the
        path at the point closest to the bike
Output:
    - Returns:
        - required:
            - Boolean value
            - Tells whether the reverse control is required
Logic:
    - If heading vector of bike makes an angle with horizontal plane, more than
    the threshold than the slope controller is required 
Example Call:
```
local com_position_vector = sim.getObjectPosition(COMFollower, Path)
local heading_error = get_heading_error(com_position_vector)
local required = require_slope_control(heading_error)
```
--]]
function require_slope_control(heading_vector)
    local required = false
    local horizontal_vector_along_bike =
        Vector:new(
        {
            heading_vector.x,
            heading_vector.y,
            0
        }
    )

    -- Get angle
    local slope_angle = heading_vector:angle(horizontal_vector_along_bike)

    -- If slope is there
    if slope_angle > math.rad(10) then
        required = true
    end

    return required
end

--[[
Function Name: require_stop
Input:
    - heading_error:
        - The difference in the bike's heading direction and the tangent to the
        path at the point closest to the bike
Output:
    - Returns:
        - required:
            - Boolean value
            - Tells whether the reverse control is required
Logic:
    - If the heading_error is greater than ReverseHeadingErrorThreshold, and the
    distance between FrontFollower and the destination is less than
    the ThresholdStopingDistance, stopping is required
    - If the heading_error is less than ReverseHeadingErrorThreshold, and the
    distance between RearFollower and the destination is less than
    the ThresholdStopingDistance, stopping is required
    - If ControlState is set to stop in the last iteration, then wait for the
    signal from the manipulator to move. After receiving the signal set
    required to false
    - If all the destination is covered, stop is required 
Example Call:
```
local com_position_vector = sim.getObjectPosition(COMFollower, Path)
local heading_error = get_heading_error(com_position_vector)
local required = require_stop(heading_error)
```
--]]
function require_stop(heading_error)
    local required = false

    -- When close to the destination
    if (#Destinations ~= 0) then
        local destination = Destinations[1]
        if math.abs(heading_error) <= ReverseHeadingErrorThreshold then
            local distance_along_path = get_distance_along_path(RearFollower, destination)
            if distance_along_path < ThresholdStopingDistance then
                required = true
            end
        else
            local distance_along_path = get_distance_along_path(FrontFollower, destination)
            if distance_along_path < ThresholdStopingDistance then
                required = true
            end
        end
    end

    -- If in stop position wait for the signal to move
    if ControlState == "Stop" then
        if (sim.waitForSignal(MoveSignal) == 1) then
            required = false
            sim.clearIntegerSignal(MoveSignal)
        else
            print("Error")
        end
        required = false
    end

    -- If completed all the destinations
    if #Destinations == 0 then
        required = true
    end

    return required
end

--[[
Function Name: require_reverse_brake
Input:
    - heading_error:
        - The difference in the bike's heading direction and the tangent to the
        path at the point closest to the bike
Output:
    - Returns:
        - required:
            - Boolean value
            - Tells whether the reverse control is required
Logic:
    - If heading error is greater than the ReverseHeadingErrorThreshold, and
    the distance between the FrontFollower and destination is less than the
    BrakingDistance, then reverse brake is requireds
Example Call:
```
local com_position_vector = sim.getObjectPosition(COMFollower, Path)
local heading_error = get_heading_error(com_position_vector)
local required = require_reverse_brake(heading_error)
```
--]]
function require_reverse_brake(heading_error)
    local required = false

    -- If heading error is more than the ReverseHeadingErrorThreshold
    if math.abs(heading_error) >= ReverseHeadingErrorThreshold then
        if (#Destinations ~= 0) then
            local destination = Destinations[1]
            local distance_along_path = get_distance_along_path(FrontFollower, destination)
            if distance_along_path < BrakingDistance then
                required = true
            end
        end
    end

    return required
end

--[[
Function Name: require_forward_brake
Input:
    - heading_error:
        - The difference in the bike's heading direction and the tangent to the
        path at the point closest to the bike
Output:
    - Returns:
        - required:
            - Boolean value
            - Tells whether the reverse control is required
Logic:
    - If heading error is less than the ReverseHeadingErrorThreshold and
    distance between the RearFollower and the destination is less than the
    BrakingDistance then forward brake is required
Example Call:
```
local com_position_vector = sim.getObjectPosition(COMFollower, Path)
local heading_error = get_heading_error(com_position_vector)
local required = require_forward_brake(heading_error)
```
--]]
function require_forward_brake(heading_error)
    local required = false

    if math.abs(heading_error) <= ReverseHeadingErrorThreshold then
        if (#Destinations ~= 0) then
            local destination = Destinations[1]
            local distance_along_path = get_distance_along_path(RearFollower, destination)
            if distance_along_path < BrakingDistance then
                required = true
            end
        end
    end

    return required
end

--[[
Function Name: predictive_controller
Input:
Output:
    - Returns:
        - predictive_desired_angular_speed:
            - Desired angular speed of the rear motor
        - predictive_desired_steering_angle:
            - Desired steering angle of the steering motor
Logic:
    - Let us say states of bike is
        - Position of center of mass
        - Heading vector
        - Steering angle
        - Bike speed
    - The predictive controller uses the states to compute the next states of
    the bike and at each step it also computes the desired angular speed and
    desired steering angle
    - After the completion of predictions, the desired angular speeds and the
    desired steering angles are used to calculate the predictive angular speed
    and the predictive steering angle respectivey
    - Research paper
        - "A path-tracking algorithm using predictive Stanley lateral controller"
        - DOI:10.1177/1729881420974852
Example Call:
```
predictive_controller()
```
--]]
function predictive_controller()
    -- States
    local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower, Path))
    local heading_vector, steering_angle
    local bike_speed
    local next_com_position, next_heading_vector
    -- Errors
    local cross_track_error, heading_error
    -- Intermediate results
    local desired_steering_angle, desired_angular_speed
    -- Predictive results
    local predictive_desired_steering_angle = 0
    local predictive_desired_angular_speed = 0

    -- Intialize the variables with the current state
    heading_vector = get_bike_heading_vector()
    steering_angle = get_steering_angle()
    bike_speed = get_bike_speed()
    -- End point flag to handle the predictive control near end points
    local reached_end = has_reached_end()

    -- Predictions
    for prediction_index = 1, NumberOfPredictions, 1 do
        -- Errors
        cross_track_error = get_cross_track_error(com_position_vector)
        heading_error = get_heading_error(com_position_vector)

        -- Basic level controller
        desired_angular_speed = longitudinal_controller(steering_angle)
        desired_steering_angle = lateral_controller(cross_track_error, heading_error, bike_speed)

        -- Update prediction values
        predictive_desired_angular_speed =
            predictive_desired_angular_speed +
            KPredictions[prediction_index] *
            desired_angular_speed
        predictive_desired_steering_angle =
            predictive_desired_steering_angle +
            KPredictions[prediction_index] *
            desired_steering_angle

        -- Update states
        -- Guard for the predictions when we reach the end of the path
        if not reached_end then
            next_com_position = get_next_com_position(com_position_vector, heading_vector, bike_speed)
            next_heading_vector = get_next_heading_vector(heading_vector, steering_angle, bike_speed)
            com_position_vector = next_com_position
            heading_vector = next_heading_vector
            steering_angle = desired_steering_angle
            bike_speed = desired_angular_speed * WheelRadius
        end
    end

    return predictive_desired_angular_speed, predictive_desired_steering_angle
end

--[[
Function Name: has_reached_end
Input:
Output:
    - Returns:
        - reached_end:
            - Tells if the bike has reached end
Logic:
    - Checks if the distance between the front follower and the end
    point is less than the LFront
    - Required to manage the predictions of the predictive controller
    in the end point of the path
Example Call:
```
local com_position = sim.getObjectPosition(COMFollower)
local com_position_vector = Vector:New(com_position)
local heading_vector = get_bike_heading_vector()
reached_end = has_reached_end(com_position_vector, heading_vector)
```
--]]
function has_reached_end()
    local reached_end = false
    local front_position_vector = Vector:new(sim.getObjectPosition(FrontFollower, Path))

    -- Get_closest point, on the path, to the follower position
    local follower_position = {
        front_position_vector.x,
        front_position_vector.y,
        front_position_vector.z
    }
    local closest_point_length = sim.getClosestPosOnPath(PathPositions, PathLengths, follower_position)
    local closest_point = sim.getPathInterpolatedConfig(PathPositions, PathLengths, closest_point_length)
    local end_point = {
        PathPositions[(#PathLengths - 1) * 3 + 1],
        PathPositions[(#PathLengths - 1) * 3 + 2],
        PathPositions[(#PathLengths - 1) * 3 + 3]
    }

    local distance =
        (
            (closest_point[1] - end_point[1])^2 +
            (closest_point[2] - end_point[2])^2 +
            (closest_point[3] - end_point[3])^2
        )^0.5

    if distance < LFront then
        reached_end = true
    end

    return reached_end
end

--[[
Function Name: get_next_heading_error
Input:
    - heading_vector:
        - The vector in the heading direction of the bike
    - steering_angle:
        - Steering angle of the steering motor
    - speed:
        - Speed of the bike
Output:
    - Returns:
        - next_heading_vector:
            - The vector in the direction of the heading of the bike after the time TPredict
Logic:
    - next_heading_vector is computed using the heading vector and steering_angle
Example Call:
```
local heading_vector = get_bike_heading_vector()
local steering_angle = get_steering_angle()
local speed = get_bike_speed()
local next_heading_vector = get_next_heading_vector(heading_vector, steering_angle, speed)
```
--]]
function get_next_heading_vector(heading_vector, steering_angle, speed)
    local next_heading_vector = Vector:new()

    -- Normalize
    heading_vector:normalize()

    -- Vector perpendicular to the bike
    local vertical_vector = Vector:new({0, 0, 1})
    local bike_perpendicular_vector = heading_vector:cross(vertical_vector)
    bike_perpendicular_vector:normalize()

    -- Calculate next heading vector
    local angular_speed = speed * math.tan(steering_angle) / (LFront + LRear)
    next_heading_vector.x = heading_vector.x + angular_speed * TPredict * bike_perpendicular_vector.x
    next_heading_vector.y = heading_vector.y + angular_speed * TPredict * bike_perpendicular_vector.y
    next_heading_vector.z = heading_vector.z + angular_speed * TPredict * bike_perpendicular_vector.z

    return next_heading_vector
end

--[[
Function Name: get_next_com_position
Input:
    - heading_vector:
        - The vector in the heading direction of the bike
    - steering_angle:
        - Steering angle of the steering motor
    - speed:
        - Speed of the bike
Output:
    - Returns:
        - next_com_vector:
            - The vector of the position of the center of mass of the DiaryBikeChassis
            after the time TPredict
Logic:
    - next_com_vector is computed using the heading vector and steering_angle
Example Call:
```
local heading_vector = get_bike_heading_vector()
local steering_angle = get_steering_angle()
local speed = get_bike_speed()
local next_com_vector = get_next_com_position(heading_vector, steering_angle, speed)
```
--]]
function get_next_com_position(com_position, heading_vector, speed)
    local next_com_position

    -- Normalize
    heading_vector:normalize()

    -- Calculate next com position
    next_com_position = Vector:new()
    next_com_position.x = com_position.x + speed * TPredict * heading_vector.x
    next_com_position.y = com_position.y + speed * TPredict * heading_vector.y
    next_com_position.z = com_position.z + speed * TPredict * heading_vector.y

    return next_com_position
end

--[[
Function Name: get_steering_angle
Input:
Output:
    - Returns:
        - steering_angle:
            - The current steering angle of the steering motor joint
Logic:
    - Retreives the joint's positon
Example Call:
```
local steering_angle = get_steering_angle()
```
--]]
function get_steering_angle()
    local steering_angle = -sim.getJointPosition(SteeringMotor)
    return steering_angle
end

--[[
Function Name: longitudinal_controller
Input:
    - steering_angle:
        - The joint angle of the steering motor
Output:
    - Returns
        - desired_angular_speed:
            - The required angular speed of the rear motor joint
Logic:
    - The angular speed of the rear motor is calculated according the equation menitoned
    below
    - The value of desired angular speed is less for the higher steering angles, as we
    want lower speed while turning
    - The value of desired angular speed is more for the lower steering angles, as
    we want to traverse faster in the straight paths
Example Call:
```
local steering_angle = get_steering_angle()
local desired_angular_speed = longitudinal_controller(steering_angle)
```
--]]
function longitudinal_controller(steering_angle)
    local scaled_steering_angle = steering_angle / MaxSteerAngle
    local desired_angular_speed =
        ((MaxAngularSpeed - MinAngularSpeed) * (1 - math.abs(scaled_steering_angle)) ^ 2 + MinAngularSpeed)

    return desired_angular_speed
end

--[[
Function Name: get_distance_along_path
Input:
    - follower:
        - The dummy object used to represent either rear, center of mass
        or front part of the bike
    - destination:
        - The destination where we want to reach
Output:
    - distance_along_path
        - Distance between the follower and destination along the path
Logic:
    - Get the length of the closest point on the path from the follower
    - Get the length of the closest point on the path from the destination
    - The difference between them would be the distance along the path
Example Call:
```
local destination = sim.getObjectHandle(Destination[1])
local distance_along_path = get_distance_along_path(COMFollower, destination)
```
--]]
function get_distance_along_path(follower, destination)
    local destination_position = sim.getObjectPosition(destination, Path)
    local destination_length = sim.getClosestPosOnPath(PathPositions, PathLengths, destination_position)
    local follower_position = sim.getObjectPosition(follower, Path)
    local follower_length = sim.getClosestPosOnPath(PathPositions, PathLengths, follower_position)
    local distance_along_path = math.abs(destination_length - follower_length)

    return distance_along_path
end

--[[
Function Name: reverse_lateral_controller
Input:
    - cross_track_error:
        - The distance of bike from the path
    - heading_error:
        - The difference in the angle of heading vector of the bike and
        the tangent to the path at the point where the bike is present
    - bike_speed:
        - The speed of the bike
Output:
    - desired_steering_angle:
        - The desired steering angel of the steering motor
Logic:
    - We have used the Hoffmann Stanley controller with a little bit modification
    for the reverse direction
    - It tries to minimize the cross track error and heading error
    - Research paper
        - "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing"
        - DOI:10.1109/ACC.2007.4282788
Example Call:
```
local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower))
local cross_track_error = get_cross_track_error(com_position_vector)
local heading_error = get_heading_error(com_position_vector)
local bike_speed = get_bike_speed()
local desired_steering_angle = reverse_lateral_controller(cross_track_error, heading_error, bike_speed)
```
--]]
function reverse_lateral_controller(cross_track_error, heading_error, bike_speed)
    -- Reverse Stanley controller
    local desired_steer_angle =
        KReverseHeading * heading_error - math.atan(KReverseCrossTrack * cross_track_error / (KSoft + bike_speed))

    -- Limit steer angle
    if math.abs(desired_steer_angle) > MaxSteerAngle then
        desired_steer_angle = math.abs(desired_steer_angle) / desired_steer_angle * MaxSteerAngle
    end

    return desired_steer_angle
end

--[[
Function Name: lateral_controller
    - cross_track_error:
        - The distance of bike from the path
    - heading_error:
        - The difference in the angle of heading vector of the bike and
        the tangent to the path at the point where the bike is present
    - bike_speed:
        - The speed of the bike
Output:
    - desired_steering_angle:
        - The desired steering angel of the steering motor
Logic:
    - We have used the Hoffmann Stanley controller
    - It tries to minimize the cross track error and heading error
    - Research paper
        - "Autonomous Automobile Trajectory Tracking for Off-Road Driving: Controller Design, Experimental Validation and Racing"
        - DOI:10.1109/ACC.2007.4282788
Example Call:
```
local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower))
local cross_track_error = get_cross_track_error(com_position_vector)
local heading_error = get_heading_error(com_position_vector)
local bike_speed = get_bike_speed()
local desired_steering_angle = lateral_controller(cross_track_error, heading_error, bike_speed)
```
--]]
function lateral_controller(cross_track_error, heading_error, bike_speed)
    -- Stanley controller
    local desired_steer_angle =
        -KHeading * heading_error - (2 - KHeading) * math.atan(KCrossTrack * cross_track_error / (KSoft + bike_speed))

    -- Limit steer angle
    if math.abs(desired_steer_angle) > MaxSteerAngle then
        desired_steer_angle = math.abs(desired_steer_angle) / desired_steer_angle * MaxSteerAngle
    end

    return desired_steer_angle
end

--[[
Function Name: get_bike_speed
Input:
Output:
    - bike_speed:
Logic:
    - Get the bike's velocity and compute its magnitude
Example Call:
```
local bike_speed = get_bike_speed()
```
--]]
function get_bike_speed()
    local bike_velocity = sim.getObjectVelocity(Body)
    bike_velocity = Vector:new(bike_velocity)
    local bike_speed = bike_velocity:mag()

    return bike_speed
end

--[[
Function Name: get_cross_track_error
Input:
    - follower_position_vector
        - Position vector of the follower
Output:
    - cross_track_error:
        - The distance between the follower and closest point on the path
        in the plane of the path
Logic:
    - The distance between follower and closest point is to calculated on the
    plane of the path
    - Distance vector is projected on the path plane and then its magnitude is
    computed
    - Distance is necessary to be projected on the path plane otherwise on the
    slope we would get the wrong cross track error
Example Call:
```
local com_position_vector = Vector:new(sim.getObjectPosition(COMFollower))
local cross_track_error = get_cross_track_error(com_position_vector)
```
--]]
function get_cross_track_error(follower_position_vector)
    -- Get_closest point, on the path, to the follower position
    local follower_position = {
        follower_position_vector.x,
        follower_position_vector.y,
        follower_position_vector.z
    }
    local closest_point_length = sim.getClosestPosOnPath(PathPositions, PathLengths, follower_position)
    local closest_point_position = sim.getPathInterpolatedConfig(PathPositions, PathLengths, closest_point_length)

    local cross_track_error_mag = calculate_distance_on_path_plane(follower_position, closest_point_position)
    local cross_track_error_sign = get_cross_track_error_sign(follower_position, closest_point_position)
    local cross_track_error = cross_track_error_sign * cross_track_error_mag

    return cross_track_error
end

--[[
Function Name: get_cross_track_error_sign
Input:
    - follower_position
        - The position of the follower
    - closest_point_position
        - The position of the closest point to the follower, on the path
Output:
    - sign
        - The sign of the cross track error
Logic:
    - Get the vector from closest point to the follower
    - Get the tangent to the path near the follower
    - Perform the cross product of the tangent to path and vector
    from closest point to the follower
    - Based upon the sign of z component of the cross product decide
    the sign of the cross track error
Example Call:
```
local follower_position = sim.getObjectPosition(COMFollower)
local closest_point_length = sim.getClosestPosOnPath(PathPositions, PathLengths, follower_position)
local closest_point_position = sim.getPathInterpolatedConfig(PathPositions, PathLengths, closest_point_length)
local cross_track_error_sign = get_cross_track_error_sign(follower_position, closest_point_position)
```
--]]
function get_cross_track_error_sign(follower_position, closest_point_position)
    local tangent_path_vector = get_tangent_path(follower_position)

    local closest_point_follower_vector =
        Vector:new(
        {
            follower_position[1] - closest_point_position[1],
            follower_position[2] - closest_point_position[2],
            follower_position[3] - closest_point_position[3]
        }
    )
    closest_point_follower_vector:normalize()
    local cross_product = tangent_path_vector:cross(closest_point_follower_vector)
    cross_product:normalize()

    local sign
    if cross_product.z < 0 then
        sign = 1
    else
        sign = -1
    end

    return sign
end

--[[
Function Name: get_tangent_path
Input:
    - follower_position
        - The position of the follower
Output:
    - tangent_vector
        - The tangent to the path near the follower
Logic:
    - Get the closest points on the path near the follower
    - The vector between their position will give the tangent to the path
Example Call:
```
local follower_position = sim.getObjectPosition(COMFollower)
local tangent = get_tangent_path(follower_position)
```
--]]
function get_tangent_path(follower_position)
    local point1_length = sim.getClosestPosOnPath(PathPositions, PathLengths, follower_position)
    local point2_length

    if point1_length < PathLengths[#PathLengths - 2] then
        point2_length = point1_length + 2 * DeltaL
    else
        point2_length = point1_length
        point1_length = point2_length - 2 * DeltaL
    end

    local point1_position_wrt_path = sim.getPathInterpolatedConfig(PathPositions, PathLengths, point1_length)
    local point2_position_wrt_path = sim.getPathInterpolatedConfig(PathPositions, PathLengths, point2_length)
    local tangent_vector =
        Vector:new(
        {
            point2_position_wrt_path[1] - point1_position_wrt_path[1],
            point2_position_wrt_path[2] - point1_position_wrt_path[2],
            point2_position_wrt_path[3] - point1_position_wrt_path[3]
        }
    )
    tangent_vector:normalize()

    return tangent_vector
end

--[[
Function Name: calculate_distance_on_path_plane
Input:
    - follower_position
    - closest_point_position
        - Point on the path closest to the follower
Output:
    - distance
        - Distance between the follower and its closest point on the
        plane of the path
Logic:
    - Get the distance vector between the follower and its closest point
    - Project the vector on the plane of the path
    - Get its magnitude
Example Call:
```
local distance = calculate_distance_on_path_plane(follower_position, closest_point_position)
```
--]]
function calculate_distance_on_path_plane(follower_position, closest_point_position)
    local distance
    local distance_vector =
        Vector:new(
        {
            follower_position[1] - closest_point_position[1],
            follower_position[2] - closest_point_position[2],
            follower_position[3] - closest_point_position[3]
        }
    )

    -- Take projection of distance vector on path's plane
    local tangent_path_vector = get_tangent_path(follower_position)
    local vertical_vector = Vector:new({0, 0, 1})
    local path_plane_vector = vertical_vector:cross(tangent_path_vector)
    local path_perpendicular_vector = tangent_path_vector:cross(path_plane_vector)
    path_perpendicular_vector:normalize()

    -- Projection of distance vector on to the path's plane
    local dot_product = path_perpendicular_vector:dot(distance_vector)
    local distance_projection_vector =
        Vector:new(
        {
            distance_vector.x - dot_product * path_perpendicular_vector.x,
            distance_vector.y - dot_product * path_perpendicular_vector.y,
            distance_vector.z - dot_product * path_perpendicular_vector.z
        }
    )
    distance = distance_projection_vector:mag()

    return distance
end

--[[
Function Name: get_heading_error
Input:
    - follower_position_vector
        - Position vector of the follower
Output:
    - heading_error
        - The angle difference between the heading vector of the bike and the
        tangent to the path
Logic:
    - Get the heading vector
    - Get the tangent to the path near the follower
    - Get the angle between the two vectors
Example Call:
```
local heading_error = get_heading_error(follower_position_vector)
```
--]]
function get_heading_error(follower_position_vector)
    local follower_position = {
        follower_position_vector.x,
        follower_position_vector.y,
        follower_position_vector.z
    }
    local bike_heading_vector = get_bike_heading_vector()
    local tangent_path_vector = get_tangent_path(follower_position)

    -- Get vector perpendicular to path's plane
    local vertical_vector = Vector:new({0, 0, 1})
    local path_plane_vector = vertical_vector:cross(tangent_path_vector)
    local path_perpendicular_vector = tangent_path_vector:cross(path_plane_vector)
    path_perpendicular_vector:normalize()

    -- Projection of bike's heading vector on to the path's plane
    local dot_product = path_perpendicular_vector:dot(bike_heading_vector)
    local bike_heading_projection_vector =
        Vector:new(
        {
            bike_heading_vector.x - dot_product * path_perpendicular_vector.x,
            bike_heading_vector.y - dot_product * path_perpendicular_vector.y,
            bike_heading_vector.z - dot_product * path_perpendicular_vector.z
        }
    )
    bike_heading_projection_vector:normalize()

    -- Error calculation
    local heading_error_mag = tangent_path_vector:angle(bike_heading_projection_vector)
    local cross_product = tangent_path_vector:cross(bike_heading_projection_vector)
    local heading_error_sign
    if cross_product.z < 0 then
        heading_error_sign = 1
    else
        heading_error_sign = -1
    end
    local heading_error = heading_error_sign * heading_error_mag

    return heading_error
end

--[[
Function Name: get_bike_heading_vector
Input:
Output:
    - heading_vector
        - The vector in the heading direction of the bike
Logic:
    - Get the matrix of the bike
    - Get the vector in the direction of its heading from that matrix
Example Call:
```
local heading_vector = get_bike_heading_vector()
```
--]]
function get_bike_heading_vector()
    local tf_body_wrt_path = sim.getObjectMatrix(Body, Path)
    local heading_vector = Vector:new({tf_body_wrt_path[1], tf_body_wrt_path[5], tf_body_wrt_path[9]})

    return heading_vector
end

--[[
Function Name: generate_angular_speed
Input:
    - desired_angular_speed
        - The required angular speed of the rear motor
Output:
    - Set the rear motor's joint velocity
Logic:
    - Set the rear motor's joint velocity
Example Call:
```
local desired_angular_speed = 2
generate_angular_speed(desired_angular_speed)
```
--]]
function generate_angular_speed(desired_angular_speed)
    if math.abs(desired_angular_speed) > MaxAngularSpeed then
        print('Controller has asked for more than the allowed speed')
        sim.stopSimulation()
    end

    sim.setJointTargetVelocity(RearMotor, desired_angular_speed)
end

--[[
Function Name: rotate_steering
Input:
    - desired_steer_angle
        - The required steering angle of the steering motor
Output:
    - Rotate the steering motor joint by the required angle
Logic:
    - Set the joint target position
Example Call:
```
local desired_steer_angle = math.rad(30)
rotate_steering(desired_steer_angle)
```
--]]
function rotate_steering(desired_steer_angle)
    if math.abs(desired_steer_angle) > MaxSteerAngle then
        print('Controller has asked for more than the allowed steering angle')
        sim.stopSimulation()
    end

    sim.setJointTargetPosition(SteeringMotor, -desired_steer_angle)
end

-- Vector
-- Meta class
Vector = {x = 0, y = 0, z = 0}

--[[
Function Name: Vector:new
    - Derived class
    - Constructor for Vector class
Input:
    - point
        - The table of three values which represents a vector or position
Output:
    - Returns:
        - obj:
            - The vector object
Logic:
    - Update the attributes of the vector class
Example Call:
```
local vector = Vector:new({1, 0, 2})
```
--]]
function Vector:new(point)
    local obj = {}
    setmetatable(obj, {__index = self})

    point = point or {0, 0, 0}
    obj.x = point[1]
    obj.y = point[2]
    obj.z = point[3]

    return obj
end

--[[
Function Name: Vector:angle
    -- Function to compute angle between two vectors
    -- The returned angle is in radians
Input:
    - vec
        - Another object of Vector class
Output:
    - theta
        - Angle between the two vectors
Logic:
    - Use the dot product to compute the angle between the two vectors
Example Call: 
```
local vector1 = Vector:new({1, 0, 2})
local vector2 = Vector:new({2, 4, 1})
local theta = vector1:angle(vector2)
```
--]]
function Vector:angle(vec)
    local cos_theta = self:dot(vec) / (self:mag() * vec:mag())
    local theta = math.acos(cos_theta)

    return theta
end

-- Function to perform dot product between two vectors
--[[
Function Name: dot
Input:
    - vec
        - Another object of Vector class
Output:
    - result
        - The dot product of the two vectors
Logic:
    - Compute the dot product between the two vectors
Example Call:
```
local vector1 = Vector:new({1, 0, 2})
local vector2 = Vector:new({2, 4, 1})
local dot_product = vector1:dot(vector2)
```
--]]
function Vector:dot(vec)
    local result = self.x * vec.x + self.y * vec.y + self.z * vec.z

    return result
end

--[[
Function Name: Vector:cross
Input:
    - vec
        - Another object of Vector class
Output:
    - result
        - A vector
        - The cross product of the two vectors
Logic:
    - Compute the cross product between the two vectors
Example Call:
```
local vector1 = Vector:new({1, 0, 2})
local vector2 = Vector:new({2, 4, 1})
local cross_product = vector1:cross(vector2)
```
--]]
function Vector:cross(vec)
    local result = Vector:new({0, 0, 0})
    result.x = self.y * vec.z - self.z * vec.y
    result.y = self.z * vec.x - self.x * vec.z
    result.z = self.x * vec.y - self.y * vec.x

    return result
end

--[[
Function Name: normalize
Input:
Output:
    - Normalize the vector
Logic:
    - Normalize the vector
Example Call:
```
local vector1 = Vector:new({1, 0, 2})
vector1:normalize()
```
--]]
function Vector:normalize()
    local mag = self:mag()
    self.x = self.x / mag
    self.y = self.y / mag
    self.z = self.z / mag
end

--[[
Function Name: Vector:mag()
Input:
Output:
    - Returns:
        - mag
            - The magnitude of the vector
Logic:  
Example Call:
```
local vector1 = Vector:new({1, 0, 2})
local magnitude = vector1:mag()
```
--]]
function Vector:mag()
    local mag = (self.x ^ 2 + self.y ^ 2 + self.z ^ 2) ^ 0.5
    return mag
end

--[[
Function Name: Vector:update
Input:
    - values
        - The table of 3 values
Output:
    - Updates the attributes of the vector
Logic:
Example Call:
```
local vector1 = Vector:new({1, 0, 2})
vector1:update({2, 4, 1})
```
--]]
function Vector:update(values)
    self.x = values[1]
    self.y = values[2]
    self.z = values[3]
end