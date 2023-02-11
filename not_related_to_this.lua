
function sysCall_init()
    corout=coroutine.create(coroutineMain)
    m1a = sim.getObject('./M1a')
    m1b = sim.getObject('./M1b')
    m2a = sim.getObject('./M2a')
    m2b = sim.getObject('./M2b')
    m3a = sim.getObject('./M3a')
    m3b = sim.getObject('./M3b')
    m4a = sim.getObject('./M4a')
    m4b = sim.getObject('./M4b')
    sim.setJointTargetVelocity(m1a,-1)-- -1
    sim.setJointTargetVelocity(m1b,-1)-- -1
    sim.setJointTargetVelocity(m2a,-1)-- -1
    sim.setJointTargetVelocity(m2b,-1)-- -1
    sim.setJointTargetVelocity(m3a,-1)-- -1
    sim.setJointTargetVelocity(m3b,-1)-- -1
    sim.setJointTargetVelocity(m4a,-1)-- -1
    sim.setJointTargetVelocity(m4b,-1)-- -1
    
    --pairs
    pair = 1 -- 1 --> joint 1,4 || 0 --> joint 2,3

    -- Parameters
    joint1a_stat = 0 -- 0-start | 1-end | 2-trans
    joint1b_stat = 0 -- 0-start | 1-end | 2-trans
    joint2a_stat = 0 -- 0-start | 1-end | 2-trans
    joint2b_stat = 0 -- 0-start | 1-end | 2-trans
    joint3a_stat = 0 -- 0-start | 1-end | 2-trans
    joint3b_stat = 0 -- 0-start | 1-end | 2-trans
    joint4a_stat = 0 -- 0-start | 1-end | 2-trans
    joint4b_stat = 0 -- 0-start | 1-end | 2-trans
    forward_motion = 0 -- 1-> going forward | 0-> going reverse | -1-> stop
    angle1a = 0 
    angle1b = 0
    angle2a = 0 
    angle2b = 0
    angle3a = 0 
    angle3b = 0
    angle4a = 0 
    angle4b = 0
    TControl = 5e-2
    LastTime = sim.getSimulationTime()
    CurrentTime = sim.getSimulationTime()
    sim.setJointTargetVelocity(m1a,-1)
    sim.setJointTargetVelocity(m1b,-1)
    sim.setJointTargetVelocity(m2a,-1)
    sim.setJointTargetVelocity(m2b,-1)
    sim.setJointTargetVelocity(m3a,-1)
    sim.setJointTargetVelocity(m3b,-1)
    sim.setJointTargetVelocity(m4a,-1)
    sim.setJointTargetVelocity(m4b,-1)
    
    -- leg motions
    leg1 = 0
    leg2 = 0
    leg3 = 0
    leg4 = 0
end

function sysCall_actuation()
    if coroutine.status(corout)~='dead' then
        local ok,errorMsg=coroutine.resume(corout)
        if errorMsg then
            error(debug.traceback(corout,errorMsg),2)
        end
    end
end

function coroutineMain()

    while true do
        CurrentTime = sim.getSimulationTime()
        if (CurrentTime - LastTime > (TControl - 1e-4)) then
            update_motion_status(1,'a')
            update_motion_status(1,'b')
            update_motion_status(2,'a')
            update_motion_status(2,'b')
            update_motion_status(3,'a')
            update_motion_status(3,'b')
            update_motion_status(4,'a')
            update_motion_status(4,'b')
            perform_motion()
            LastTime = CurrentTime 
        end
    end
end

function update_motion_status(leg,joint_ab)
    local motor = sim.getObject('./M'..leg..joint_ab)
    local angle = sim.getJointPosition(motor)
    local angle_limit
    if joint_ab == 'a' then
        angle_limit = 30
    elseif joint_ab == 'b' then
        angle_limit = 20
    end

    motor_name = 'm'..leg..joint_ab
    if motor_name == 'm1a' then
        joint1a_stat = get_joint_stat(angle,angle_limit) 
    elseif motor_name == 'm1b' then
        joint1b_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm2a' then
        joint2a_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm2b' then
        joint2b_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm3a' then
        joint3a_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm3b' then
        joint3b_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm4a' then
        joint4a_stat = get_joint_stat(angle,angle_limit)
    elseif motor_name == 'm4b' then
        joint4b_stat = get_joint_stat(angle,angle_limit)
    end

end

function get_joint_stat(angle,angle_limit)
    
    local joint_stat = 0
    if angle > math.rad(angle_limit) then
        joint_stat = 1
    elseif angle < math.rad(0) then
        joint_stat = 0
    end
    return joint_stat
end


function perform_motion()

    if pair == 1 then
        -- motor 1
        if joint1a_stat == 1 then
            sim.setJointTargetVelocity(m1a,-(30*0.01) )-- -1
        elseif joint1a_stat == 0 then
            sim.setJointTargetVelocity(m1a,(30*0.01) )-- 1
        end                

        if joint1b_stat == 1 then
            sim.setJointTargetVelocity(m1b,-(20*0.01) )-- -1
        elseif joint1b_stat == 0 then
            sim.setJointTargetVelocity(m1b,(20*0.01) )-- 1
        end                

        -- motor 4
        if joint4a_stat == 1 then
            sim.setJointTargetVelocity(m4a,-(30*0.01) )-- -1
        elseif joint4a_stat == 0 then
            sim.setJointTargetVelocity(m4a,(30*0.01) )-- 1
        end                

        if joint4b_stat == 1 then
            sim.setJointTargetVelocity(m4b,-(20*0.01) )-- -1
        elseif joint4b_stat == 0 then
            sim.setJointTargetVelocity(m4b,(20*0.01) )-- 1
        end                
        sim.setJointTargetVelocity(m2a, 0 )-- -1
        sim.setJointTargetVelocity(m2b, 0 )-- -1
        sim.setJointTargetVelocity(m3b, 0 )-- -1
        sim.setJointTargetVelocity(m3a, 0 )-- -1

    elseif pair == 0 then
        -- motor 1
        if joint2a_stat == 1 then
            sim.setJointTargetVelocity(m2a,-(30*0.01) )-- -1
        elseif joint1a_stat == 0 then
            sim.setJointTargetVelocity(m2a,(30*0.01) )-- 1
        end                

        if joint2b_stat == 1 then
            sim.setJointTargetVelocity(m2b,-(20*0.01) )-- -1
        elseif joint1b_stat == 0 then
            sim.setJointTargetVelocity(m2b,(20*0.01) )-- 1
        end                

        -- motor 4
        if joint3a_stat == 1 then
            sim.setJointTargetVelocity(m3a,-(30*0.01) )-- -1
        elseif joint4a_stat == 0 then
            sim.setJointTargetVelocity(m3a,(30*0.01) )-- 1
        end                

        if joint3b_stat == 1 then
            sim.setJointTargetVelocity(m3b,-(20*0.01) )-- -1
        elseif joint4b_stat == 0 then
            sim.setJointTargetVelocity(m3b,(20*0.01) )-- 1
        end                
        sim.setJointTargetVelocity(m1a, 0 )-- -1
        sim.setJointTargetVelocity(m1b, 0 )-- -1
        sim.setJointTargetVelocity(m4b, 0 )-- -1
        sim.setJointTargetVelocity(m4a, 0 )-- -1
    end

    if joint1a_stat == 1 then
        pair = 0
    elseif joint3a_stat == 1 then
        pair = 1
    end
end