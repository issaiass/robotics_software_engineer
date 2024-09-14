function [T_total] = forwardKimenatics(theta_base, theta_finger, d_prismatic, theta_ee)
    # Base joint (rotation around Z)
    T_base = [cos(theta_base), -sin(theta_base), 0, 0;
              sin(theta_base), cos(theta_base),  0, 0;
              0,               0,                1, 0.075;
              0,               0,                0, 1];

    # Finger joint (rotation around Y)
    T_finger =  [cos(theta_finger), 0, sin(theta_finger), 0;
                 0,                1, 0,                0;
                 -sin(theta_finger), 0, cos(theta_finger), 0.5; # length of the finger link
                 0,                0, 0,                1];

    # Prismatic joint (translation along Z)
    T_prismatic =  [1, 0, 0, 0;
                    0, 1, 0, 0;
                    0, 0, 1, d_prismatic; # Prismatic joint extension
                    0, 0, 0, 1];

    # End-effector joint (rotation around Z)
    T_ee =  [cos(theta_ee), -sin(theta_ee), 0, 0;
             sin(theta_ee), cos(theta_ee),  0, 0;
             0,             0,              1, 0.1; # length of the EE base
             0,             0,              0, 1];

    # Multiply the transformations
    T_total= T_base * T_finger * T_prismatic * T_ee;

end
