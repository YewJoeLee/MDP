def run_task_2():
    # State 0
    robot = RobotState(x=1, y=1, heading='N')
    
    # State 1
    dist_to_scan_1 = calculate_forward_dist(robot.y, OBS_1_Y - CAMERA_OFFSET)
    send_to_STM(f"FW{dist_to_scan_1}")
    wait_for_ack()
    robot.update("FW", dist_to_scan_1)
    
    cv_result_1 = send_snap_and_wait()
    
    # State 2
    if cv_result_1 == "LEFT":
        send_to_STM("UL00") #Small curve around the obstacle 
        robot.apply_macro_shift("UL00")
    else:
        send_to_STM("UR00")
        robot.apply_macro_shift("UR00")
    wait_for_ack()
        
    # State 3
    # ... align if needed based on robot.x ...
    dist_to_scan_2 = calculate_forward_dist(robot.y, OBS_2_Y - CAMERA_OFFSET)
    send_to_STM(f"FW{dist_to_scan_2}")
    wait_for_ack()
    robot.update("FW", dist_to_scan_2)
    
    cv_result_2 = send_snap_and_wait()
    
    # State 4
    if cv_result_2 == "LEFT":
        send_to_STM("WL00") #Wide curve around the obstacle which is bigger than the previous one
        robot.apply_macro_shift("WL00")
    else:
        send_to_STM("WR00")
        robot.apply_macro_shift("WR00")
    wait_for_ack()
        
    # State 5: The Coarse Return
    # Do NOT route to (1, 1). Route to the Entry Node just outside it (e.g., X=1, Y=3).
    entry_node_x = 1
    entry_node_y = 3

    return_commands = a_star_to_coordinate(robot.x, robot.y, robot.heading, entry_node_x, entry_node_y)
    for cmd in return_commands:
        send_to_STM(cmd)
        wait_for_ack()


    # State 6: The Fine Parking
    # The robot is now roughly outside the carpark. Hand over to STM IR logic.
    send_to_STM("PARK") 
    wait_for_ack() # Algorithm waits while STM dynamically adjusts using IR sensors
    
    print("Task 2 Complete")

    
