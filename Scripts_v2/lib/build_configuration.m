function config = build_configuration(robot_model, array_joint_values)
    config = homeConfiguration(robot_model);
    %config = randomConfiguration(robot_model);
    if length(config) ~= length(array_joint_values)
        disp("Error! array_joints_value too small")
        return
    end
    
    for i=1:length(config)
        config(i).JointPosition = array_joint_values(i);
    end
end