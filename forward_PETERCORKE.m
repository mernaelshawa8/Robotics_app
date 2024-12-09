function ForwardKinematics(app)
  % Get DH parameters from input fields
      
      a=[0 20 30 0 ];
      
      alpha= [pi/2 0 0 pi/2 ];
      
      d= [10 0 0 0 ];
      %joint angles 
      theta= [0 0 0 0 ];
      n = length(a);
      if length(alpha) ~= n || length(theta) ~= n || length(d) ~= n
        error('Input dimensions do not match');
        return;
      end
     
      
      for i = 1:n
          % Assuming a simple revolute joint robot
       L(i)= [ Link( [0, d(i), a(i), alpha(i)])];
      end 
      %links
      robot = SerialLink(L);
      T = robot.fkine(theta) % Calculate end-effector pose based on given joint angles
      
end

