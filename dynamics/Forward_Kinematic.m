function [xa, ya] = Forward_Kinematic(theta1_a, theta2_a, theta3_a, theta4_a)
    l1 = 1;
    l2 = 1;
    l3 = 1.5;
    l4 = 1;

    xa = l1 * cos(theta1_a) + l2 * cos(theta1_a + theta2_a) + ...
         l3 * cos(theta1_a + theta2_a + theta3_a) + ...
         l4 * cos(theta1_a + theta2_a + theta3_a + theta4_a);

    ya = l1 * sin(theta1_a) + l2 * sin(theta1_a + theta2_a) + ...
         l3 * sin(theta1_a + theta2_a + theta3_a) + ...
         l4 * sin(theta1_a + theta2_a + theta3_a + theta4_a);
end
