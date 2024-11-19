function drE = velocity_foot(in1,in2)
%VELOCITY_FOOT
%    drE = VELOCITY_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.2.
%    19-Nov-2024 14:34:01

dth1 = in1(4,:);
dth2 = in1(5,:);
dth3 = in1(6,:);
l_AC = in2(18,:);
l_DE = in2(19,:);
l_OB = in2(17,:);
th1 = in1(1,:);
th2 = in1(2,:);
th3 = in1(3,:);
t2 = th1+th3;
t3 = cos(t2);
t4 = sin(t2);
t5 = t2+th2;
t6 = cos(t5);
t7 = sin(t5);
t8 = l_DE.*t3;
t9 = l_OB.*t3;
t10 = l_DE.*t4;
t11 = l_OB.*t4;
t12 = l_AC.*t6;
t13 = l_AC.*t7;
t14 = t8+t9+t12;
t15 = t10+t11+t13;
drE = [dth2.*t12+dth1.*t14+dth3.*t14;dth2.*t13+dth1.*t15+dth3.*t15];
end
