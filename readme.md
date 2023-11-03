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




<h2> SteeringMotor.lua : Stanley Predictive Controller  </h2>
<h2> ReactionWheelMotor.lua : LQR controller </h2>
<h2> MTB_Axis1.lua : LQR controller  </h2>
<h2> MTB_PickLink.lua : LQR controller  </h2>

