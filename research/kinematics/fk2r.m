function output = fk2r(theta1, theta2,l1,l2)

T1 = [cos(theta1),  0, sin(theta1), l1*cos(theta1);
      0,            1, 0,           0;
      -sin(theta1), 0, cos(theta1), l1*sin(theta1);
      0,            0, 0,           1];

T2 = [cos(theta2),  0, sin(theta2), l2*cos(theta2);
      0,            1, 0,           0;
      -sin(theta2), 0, cos(theta2), l2*sin(theta2);
      0,            0, 0,           1];

output = T1;
end

