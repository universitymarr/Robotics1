function robot = build_robot_model(joints_string, DHTable)
    n_joints = length(joints_string);

    array_bodies = rigidBody.empty(n_joints,0);
    array_joints = rigidBodyJoint.empty(n_joints,0);
    
    prefix_joint_name = "joint_";
    prefix_body_name = "body_";
    
    robot = rigidBodyTree;
    
    for i = 1:n_joints
        if joints_string(i) == "R"
            joint_type = "revolute";
        elseif joints_string(i) == "P"
            joint_type = "prismatic";
        else
            joint_type = "Error";
        end
        array_bodies(i) = rigidBody(prefix_body_name+i);
        array_joints(i) = rigidBodyJoint(prefix_joint_name+i, joint_type);
        
        setFixedTransform(array_joints(i), DHTable(i,:),'dh');
        array_bodies(i).Joint = array_joints(i);
        
        if i == 1
            parent_name = 'base';
        else
            parent_name = prefix_body_name + (i-1);
        end
        addBody(robot, array_bodies(i), parent_name);
    end
end
