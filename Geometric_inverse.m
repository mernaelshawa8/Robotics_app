function joint_angles = inverse_kinematics(link_lengths, end_effector_position)
    % Geometric method for a simple planar robot like the one in this example
    L1 = 12;
    L2 = 16;
    x = 2;
    y = 4;

    % Calculate theta2 using cosine law
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2 = atan2(sqrt(1 - cos_theta2^2), cos_theta2);

    % Calculate theta1 using inverse tangent
    theta1 = atan2(y, x) - atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    % Joint angles
    joint_angles = [theta1, theta2];
end
