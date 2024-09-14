function joint_states = inverse_kinematics(x, y, z, theta_ee)
  % Initialize joint_states vector
  joint_states = zeros(4, 1);  % [theta_base, theta_finger, d_prismatic, theta_ee]

  % Compute the base rotation angle from (x, y)
  theta = atan2(y, x);
  joint_states(1) = theta; % atan2(sin(theta), cos(theta));  % Base joint angle

  % Compute the distance in the XY plane from the base
  r = sqrt(x^2 + y^2);

  % Define parameters
  base_height = 0.0375;  % Base height from the URDF
  finger_length = 0.5;   % Length of the finger link

  % Compute the effective height (Z position) from the base height
  z_eff = z - base_height;

  % Angle for the finger joint (rotation around Y-axis), offset by pi/2
  distance_3d = sqrt(r^2 + z_eff^2);  % 3D distance to the target
  if distance_3d > finger_length
    distance_3d = finger_length;  % Limit to max reach of the finger link
  end

  theta_finger = atan2(z_eff, r) - pi/2;  % Updated finger angle calculation
  joint_states(2) = atan2(sin(theta_finger), cos(theta_finger));

  % Prismatic joint (d_prismatic): Compute the prismatic joint extension
  joint_states(3) = max(0.0, distance_3d - 0.2); % The remaining Z distance after extending the prismatic length

  % End-effector joint (theta_ee): End-effector rotation is given directly
  joint_states(4) = theta_ee;

end
