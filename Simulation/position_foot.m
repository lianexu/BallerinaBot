function rE = position_foot(in1,in2)
%POSITION_FOOT
%    rE = POSITION_FOOT(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 24.1.
%    14-Nov-2024 14:47:42

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
rE = [l_DE.*t4+l_OB.*t4+l_AC.*sin(t5);-l_DE.*t3-l_OB.*t3-l_AC.*cos(t5)];
end
