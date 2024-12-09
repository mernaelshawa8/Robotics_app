function InverseKinematicss(app)
      a=[0 7 10];
      alpha= [pi/2 0 0];
      d = [8 0 0 ];
      n = length(a);
      if length(alpha) ~= n || length(d) ~= n ||length(a) ~= n
        error('Input dimensions do not match');
      end
      for i = 1:n
          % Assuming a simple revolute joint robot
       L(i)= [Link( [0, d(i), a(i), alpha(i)])];
      end 
      %links
      robot = SerialLink(L);
      desiredpose = transl(10, 8, 7); 
      disp(desiredpose)
      q0=[0 0 0];
    %inverse kinematics using robotics toolbox
    jointAngles = robot.ikine(desiredpose,q0,'mask',[1 1 1 0 0 0]);
    disp(jointAngles)
end