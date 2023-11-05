# EYRC

## MTB_Axis1

- `sysCall_init` : Initalize object handles
    - get object handles
    - initialize 
        - pickup object locations
        - Tolerance
    - Has local str = waitForSignal("original_conf")
        - It partially acts like semaphores
        - Once it gets signal it continues
    - at end once all orders are recived coroutine is created

- `sysCall_actuation` : runs on every step of simulation
    - if coroutine is dead then prints error else runs coroutine

- `coroutineMain` : Main funtions that runs the script
    - This function waits for destination to be reached on `"destinationSignal"` from `steeringMotor` 
    - this means this code will be executed when any pickup points is reached and the MTB(bike) is at hault
    - Depending on the denstination reached it will pickup or drop the objects
    - once all the pickup from current bench are completed then `"readyToMove"` signal is set to true hence control will be shifted to `steeringMotor`

- Handle destinations `handleLoadingPoint`,`handleDairyFarm`,`handleDeliveryPoint`
    - Pick required items or  Drop item
        - for all the objects to be picked
            - calls `putObjectInStorageArea` 
            - Mark the space in bucket to 1 or 0  
            - Bucket[Bucket_id] = object or `NULL`

- putObjectInStorageArea
    - Picks object from table and put it in bucket or vice-verse
    - goes to desired object `goToObject`
    - wait for pickup to get connected to arm `MTB_PickUpOrDropReady`
    - go to bucket `goToPlacePos`
    - drop object `MTB_PickUpOrDropReady`
    - Move arm to default position

## MTB_PickLink
- CoroutineMain
    - Listens for "MTB_PickUpOrDropReady" (set by MTB_Axis1)
    - Attaches/detaches object from PickLink according to the signal
    - Sets "PickUpConnected" after picking the object
    - Sets "PickUpDropped" after dropping the object

## ReactionWheelMotor
- threads
	1. CoroutineMain 
	2. UpdateExtraMassOnBody
	3. UpdateArmHasPkg 
- UpdateExtraMassOnBody
    - Update the total mass of body
    - modifies the parameter for LQR
- UpdateArmHasPkg
    - If arm has package then set it to 1
    - waits on `ArmHasPackage` semaphore
- coroutine_main
    - Updates states (body orientation,angular vel, reaction wheel orientation and angular vel)
    - update_lqr_weights depending on states
    - generate torque accordingly

## SteeringMotor
- Predictive algo params for forward and reverse
    1. heading error constant
    2. CorssTrack error constant
    3. Soft error constant
- Limits 
    - max steering angle
    - max steering angular val
    - min strring angluar val

- Main
    - `Update` function
        - Runs only when any destination is reached 
    - `hybrid_controller` function
        - Update desired angular speed and desired angle


- `Update` function 
    - `delete` function
        - if desination is reached so delete currnet path
    - `update_destination` function
        - Update current denstination value for manuplator
    - `update_path_state` function
        - Create new path to be followed
    
- `hybrid_controller` function
    - Depending on state of motion controller is selected
    - Reason dynamics of each motion changes
    - Contollers
        1. Forward : `predictive_controller`
        2. Reverse : `reverse_controller`
        3. Slope : `slope_controller`
        4. ForwardBrake : `forward_brake_controller`
        5. ReverseBrake : `reverse_brake_controller`
        6. Stop : `stop_controller`

- `predictive_controller` function
    - from current state steering angle and velocity are found 
    - next 5 predictions are found
    - Their average is taken

- `reverse_controller` function
    - normal stanley controller is used with negative velocity output

- `slope_controller` function
    - Predictive contoller fails for slope
    - So normal stanley contoller is used

- `forward_brake_controller`, `reverse_brake_controller` function
    - Linearly decreasing angular velocity is used
 
- `stop_controller` function
    - Stop linear_vel and steering_vel to zero

### LUA
- Array : 
    ```lua
        arr = {"asd","sadasd","dasfa"}
    ```
- For loop
    ```lua
        for i = 2 : 10 : 1
        do
            -- print 2 to 10
        end
    ```

    ```lua
        for key,value in arr
        do
            -- iterate over array
        end
    ```

- While
    ```lua
        while( condition )
        do
            -- loop
        end
    ```
- Repeat untill
    ```lua
        repeat
        
        until(condition)
    ```
- Function
    - Here output type is not defined
    ```lua
        function fun(var1,var2)
            local a
            -- do something
            return a
        end
    ```
- Tables
    - Dictionary
    ```lua
        tableA = {}
        tableA['name'] = 'ohm'
    ```
- Coroutines
    - coroutine.create(function)
        - Create coroutine and returns variable pointing that coroutine 
    - coroutin.resume(coroutine_var)
        - resume coroutine
    - coroutine.running()
        - return if coroutine is runnning or main thread is runnnig
    - coroutine.status (coroutine_var)
        - Status of coroutine_var
    - coroutine.yield (coroutine_var)
        - Stops coroutine