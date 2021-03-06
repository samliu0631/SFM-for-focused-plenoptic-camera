function [jrt, jst]=jacprojRTS(j, i, rt, xyz, r0, a)
% symbolic projection function Jacobian
% code automatically generated with maple
qr0=r0(j*4+1:(j+1)*4);
t11 = rt(1)^2;
t21 = rt(2)^2;
t31 = rt(3)^2;
t51 = sqrt(1 - t11 - t21 - t31);
t6  = qr0(1)*rt(1);
t8  = qr0(3)*rt(3);
t9  = qr0(4)*rt(2);
t101 = qr0(2)*t51 + t6 - t8 + t9;
t13 = qr0(1)*rt(2);
t14 = qr0(2)*rt(3);
t17 = qr0(4)*rt(1);
t161 = qr0(3)*t51 + t13 + t14 - t17;
t18 = qr0(1)*rt(3);
t19 = qr0(2)*rt(2);
t20 = qr0(3)*rt(1);
t221 = qr0(4)*t51 + t18 - t19 + t20;
t27 = qr0(2)*rt(1);
t28 = qr0(3)*rt(2);
t29 = qr0(4)*rt(3);
t301 = qr0(1)*t51 - t27 - t28 - t29;
t241 = -t101*xyz(1) - t161*xyz(2) - t221*xyz(3);
t341 = t161*xyz(3) - t221*xyz(2) + t301*xyz(1);
t391 = -t101*xyz(3) + t221*xyz(1) + t301*xyz(2);
t441 = t101*xyz(2) - t161*xyz(1) + t301*xyz(3);
t581 = t101*t391 - t161*t341 - t221*t241 + t301*t441 + rt(6);
t12 = t11;
t23 = t21;
t32 = t31;
t53 = sqrt(1 - t12 - t23 - t32);
t102 = qr0(2)*t53 + t6 - t8 + t9;
t162 = qr0(3)*t53 + t13 + t14 - t17;
t222 = qr0(4)*t53 + t18 - t19 + t20;
t302 = qr0(1)*t53 - t27 - t28 - t29;
t242 = -t102*xyz(1) - t162*xyz(2) - t222*xyz(3);
t342 = t162*xyz(3) - t222*xyz(2) + t302*xyz(1);
t392 = -t102*xyz(3) + t222*xyz(1) + t302*xyz(2);
t442 = t102*xyz(2) - t162*xyz(1) + t302*xyz(3);
t582 = t102*t392 - t162*t342 - t222*t242 + t302*t442 + rt(6);
t5 = sqrt(1 - t12 - t23 - t32);
t10 = qr0(2)*t5 + t6 - t8 + t9;
t16 = qr0(3)*t5 + t13 + t14 - t17;
t22 = qr0(4)*t5 + t18 - t19 + t20;
t30 = qr0(1)*t5 - t27 - t28 - t29;
t24 = -t10*xyz(1) - t16*xyz(2) - t22*xyz(3);
t34 = t16*xyz(3) - t22*xyz(2) + t30*xyz(1);
t39 = -t10*xyz(3) + t22*xyz(1) + t30*xyz(2);
t44 = t10*xyz(2) - t16*xyz(1) + t30*xyz(3);
df  = double(zeros(1,47));
dfr0 = double(zeros(1,47));
dfr1 = double(zeros(1,47));
df(25) = -(a(1)*(-t101*t241 + t161*t441 - t221*t391 + t301*t341 + rt(4)) + a(2)*(-t101*t441 - t161*t241 + t221*t341 + t301*t391 + rt(5)) + a(3)*t581)/t581^2 + a(3)/t581;
df(24) = df(25)*t301 + (a(1)*t161 - a(2)*t101)/t581;
df(23) = df(25)*t101 + (-a(1)*t221 + a(2)*t301)/t581;
df(22) = -df(25)*t161 + (a(1)*t301 + a(2)*t221)/t581;
df(21) = -df(25)*t221 + (-a(1)*t101 - a(2)*t161)/t581;
df(20) = df(25)*t441 + df(22)*xyz(1) + df(23)*xyz(2) + df(24)*xyz(3) + (a(1)*t341 + a(2)*t391)/t581;
df(19) = -df(20);
df(18) = -df(20);
df(17) = -df(20);
df(16) = -df(25)*t241 + df(23)*xyz(1) - df(22)*xyz(2) - df(21)*xyz(3) + (-a(1)*t391 + a(2)*t341)/t581;
df(15) = df(16);
df(14) = -df(16);
df(13) = df(16);
df(12) = -df(25)*t341 - df(24)*xyz(1) - df(21)*xyz(2) + df(22)*xyz(3) + (a(1)*t441 - a(2)*t241)/t581;
df(11) = -df(12);
df(10) = df(12);
df(9) = df(12);
df(8) = df(25)*t391 - df(21)*xyz(1) + df(24)*xyz(2) - df(23)*xyz(3) + (-a(1)*t241 - a(2)*t441)/t581;
df(7) = df(8);
df(6) = -df(8);
df(5) = df(8);
df(4) = qr0(1)*df(20) + qr0(2)*df(8) + qr0(3)*df(12) + qr0(4)*df(16);
df(3) = -1/2*df(4)/(1 - t11 - t21 - t31)^(1/2);
df(2) = -1/2*df(4)/(1 - t11 - t21 - t31)^(1/2);
df(1) = -1/2*df(4)/(1 - t11 - t21 - t31)^(1/2);
dfr0(38) = -(a(4)*(-t102*t442 - t162*t242 + t222*t342 + t302*t392 + rt(5)) + a(5)*t582)/t582^2 + a(5)/t582;
dfr0(37) = dfr0(38)*t302 - a(4)*t102/t582;
dfr0(36) = dfr0(38)*t102 + a(4)*t302/t582;
dfr0(35) = -dfr0(38)*t162 + a(4)*t222/t582;
dfr0(34) = -dfr0(38)*t222 - a(4)*t162/t582;
dfr0(33) = dfr0(38)*t442 + dfr0(35)*xyz(1) + dfr0(36)*xyz(2) + dfr0(37)*xyz(3) + a(4)*t392/t582;
dfr0(32) = -dfr0(38)*t242 + dfr0(36)*xyz(1) - dfr0(35)*xyz(2) - dfr0(34)*xyz(3) + a(4)*t342/t582;
dfr0(31) = -dfr0(38)*t342 - dfr0(37)*xyz(1) - dfr0(34)*xyz(2) + dfr0(35)*xyz(3) - a(4)*t242/t582;
dfr0(30) = dfr0(38)*t392 - dfr0(34)*xyz(1) + dfr0(37)*xyz(2) - dfr0(36)*xyz(3) - a(4)*t442/t582;
dfr0(29) = qr0(1)*dfr0(33) + qr0(2)*dfr0(30) + qr0(3)*dfr0(31) + qr0(4)*dfr0(32);
dfr0(28) = -1/2*dfr0(29)/(1 - t12 - t23 - t32)^(1/2);
dfr0(27) = -1/2*dfr0(29)/(1 - t12 - t23 - t32)^(1/2);
dfr0(26) = -1/2*dfr0(29)/(1 - t12 - t23 - t32)^(1/2);
dfr0(19) = -dfr0(33);
dfr0(18) = -dfr0(33);
dfr0(17) = -dfr0(33);
dfr0(15) = dfr0(32);
dfr0(14) = -dfr0(32);
dfr0(13) = dfr0(32);
dfr0(11) = -dfr0(31);
dfr0(10) = dfr0(31);
dfr0(9) = dfr0(31);
dfr0(7) = dfr0(30);
dfr0(6) = -dfr0(30);
dfr0(5) = dfr0(30);
dfr0(3) = dfr0(28);
dfr0(2) = dfr0(27);
dfr0(1) = dfr0(26);
dfr1(47) = 1/2*a(8)*a(7)*t30/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(46) = 1/2*a(8)*a(7)*t10/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(45) = -1/2*a(8)*a(7)*t16/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(44) = -1/2*a(8)*a(7)*t22/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(43) = dfr1(45)*xyz(1) + dfr1(46)*xyz(2) + dfr1(47)*xyz(3) + 1/2*a(8)*a(7)*t44/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(42) = dfr1(46)*xyz(1) - dfr1(45)*xyz(2) - dfr1(44)*xyz(3) - 1/2*a(8)*a(7)*t24/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(41) = -dfr1(47)*xyz(1) - dfr1(44)*xyz(2) + dfr1(45)*xyz(3) - 1/2*a(8)*a(7)*t34/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(40) = -dfr1(44)*xyz(1) + dfr1(47)*xyz(2) - dfr1(46)*xyz(3) + 1/2*a(8)*a(7)*t39/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(39) = qr0(1)*dfr1(43) + qr0(2)*dfr1(40) + qr0(3)*dfr1(41) + qr0(4)*dfr1(42);
dfr1(28) = -1/2*dfr1(39)/(1 - t12 - t23 - t32)^(1/2);
dfr1(27) = -1/2*dfr1(39)/(1 - t12 - t23 - t32)^(1/2);
dfr1(26) = -1/2*dfr1(39)/(1 - t12 - t23 - t32)^(1/2);
dfr1(19) = -dfr1(43);
dfr1(18) = -dfr1(43);
dfr1(17) = -dfr1(43);
dfr1(15) = dfr1(42);
dfr1(14) = -dfr1(42);
dfr1(13) = dfr1(42);
dfr1(11) = -dfr1(41);
dfr1(10) = dfr1(41);
dfr1(9) = dfr1(41);
dfr1(7) = dfr1(40);
dfr1(6) = -dfr1(40);
dfr1(5) = dfr1(40);
dfr1(3) = dfr1(28);
dfr1(2) = dfr1(27);
dfr1(1) = dfr1(26);
jrt = zeros(1,18);
jrt(1) = qr0(1)*df(5) + qr0(2)*df(17) + qr0(3)*df(15) + qr0(4)*df(11) + 2*rt(1)*df(1);
jrt(2) = qr0(1)*df(9) + qr0(2)*df(14) + qr0(3)*df(18) + qr0(4)*df(7) + 2*rt(2)*df(2);
jrt(3) = qr0(1)*df(13) + qr0(2)*df(10) + qr0(3)*df(6) + qr0(4)*df(19) + 2*rt(3)*df(3);
jrt(4) = a(1)/t581;
jrt(5) = a(2)/t581;
jrt(6) = df(25);
jrt(7) = qr0(1)*dfr0(5) + qr0(2)*dfr0(17) + qr0(3)*dfr0(15) + qr0(4)*dfr0(11) + 2*rt(1)*dfr0(1);
jrt(8) = qr0(1)*dfr0(9) + qr0(2)*dfr0(14) + qr0(3)*dfr0(18) + qr0(4)*dfr0(7) + 2*rt(2)*dfr0(2);
jrt(9) = qr0(1)*dfr0(13) + qr0(2)*dfr0(10) + qr0(3)*dfr0(6) + qr0(4)*dfr0(19) + 2*rt(3)*dfr0(3);
jrt(10) = 0.0;
jrt(11) = a(4)/t582;
jrt(12) = dfr0(38);
jrt(13) = qr0(1)*dfr1(5) + qr0(2)*dfr1(17) + qr0(3)*dfr1(15) + qr0(4)*dfr1(11) + 2*rt(1)*dfr1(1);
jrt(14) = qr0(1)*dfr1(9) + qr0(2)*dfr1(14) + qr0(3)*dfr1(18) + qr0(4)*dfr1(7) + 2*rt(2)*dfr1(2);
jrt(15) = qr0(1)*dfr1(13) + qr0(2)*dfr1(10) + qr0(3)*dfr1(6) + qr0(4)*dfr1(19) + 2*rt(3)*dfr1(3);
jrt(16) = 0.0;
jrt(17) = 0.0;
jrt(18) = 1/2*a(8)*a(7)/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;

  
  





t11 = rt(1)^2;
t21 = rt(2)^2;
t31 = rt(3)^2;
t51 = sqrt(1 - t11 - t21 - t31);
t6 = qr0(1)*rt(1);
t8 = qr0(3)*rt(3);
t9 = qr0(4)*rt(2);
t101 = qr0(2)*t51 + t6 - t8 + t9;
t13 = qr0(1)*rt(2);
t14 = qr0(2)*rt(3);
t17 = qr0(4)*rt(1);
t161 = qr0(3)*t51 + t13 + t14 - t17;
t18 = qr0(1)*rt(3);
t19 = qr0(2)*rt(2);
t20 = qr0(3)*rt(1);
t221 = qr0(4)*t51 + t18 - t19 + t20;
t27 = qr0(2)*rt(1);
t28 = qr0(3)*rt(2);
t29 = qr0(4)*rt(3);
t301 = qr0(1)*t51 - t27 - t28 - t29;
t241 = -t101*xyz(1) - t161*xyz(2) - t221*xyz(3);
t341 = t161*xyz(3) - t221*xyz(2) + t301*xyz(1);
t391 = -t101*xyz(3) + t221*xyz(1) + t301*xyz(2);
t441 = t101*xyz(2) - t161*xyz(1) + t301*xyz(3);
t581 = t101*t391 - t161*t341 - t221*t241 + t301*t441 + rt(6);
t12 = t11;
t23 = t21;
t32 = t31;
t53 = sqrt(1 - t12 - t23 - t32);
t102 = qr0(2)*t53 + t6 - t8 + t9;
t162 = qr0(3)*t53 + t13 + t14 - t17;
t222 = qr0(4)*t53 + t18 - t19 + t20;
t302 = qr0(1)*t53 - t27 - t28 - t29;
t242 = -t102*xyz(1) - t162*xyz(2) - t222*xyz(3);
t342 = t162*xyz(3) - t222*xyz(2) + t302*xyz(1);
t392 = -t102*xyz(3) + t222*xyz(1) + t302*xyz(2);
t442 = t102*xyz(2) - t162*xyz(1) + t302*xyz(3);
t582 = t102*t392 - t162*t342 - t222*t242 + t302*t442 + rt(6);
t5 = sqrt(1 - t12 - t23 - t32);
t10 = qr0(2)*t5 + t6 - t8 + t9;
t16 = qr0(3)*t5 + t13 + t14 - t17;
t22 = qr0(4)*t5 + t18 - t19 + t20;
t30 = qr0(1)*t5 - t27 - t28 - t29;
t24 = -t10*xyz(1) - t16*xyz(2) - t22*xyz(3);
t34 = t16*xyz(3) - t22*xyz(2) + t30*xyz(1);
t39 = -t10*xyz(3) + t22*xyz(1) + t30*xyz(2);
t44 = t10*xyz(2) - t16*xyz(1) + t30*xyz(3);
df =zeros(1,14);
dfr0 = zeros(1,14);
dfr1 = zeros(1,14);
df(5) = -(a(1)*(-t101*t241 + t161*t441 - t221*t391 + t301*t341 + rt(4)) + a(2)*(-t101*t441 - t161*t241 + t221*t341 + t301*t391 + rt(5)) + a(3)*t581)/t581^2 + a(3)/t581;
df(4) = df(5)*t301 + (a(1)*t161 - a(2)*t101)/t581;
df(3) = df(5)*t101 + (-a(1)*t221 + a(2)*t301)/t581;
df(2) = -df(5)*t161 + (a(1)*t301 + a(2)*t221)/t581;
df(1) = -df(5)*t221 + (-a(1)*t101 - a(2)*t161)/t581;
dfr0(10) = -(a(4)*(-t102*t442 - t162*t242 + t222*t342 + t302*t392 + rt(5)) + a(5)*t582)/t582^2 + a(5)/t582;
dfr0(9) = dfr0(10)*t302 - a(4)*t102/t582;
dfr0(8) = dfr0(10)*t102 + a(4)*t302/t582;
dfr0(7) = -dfr0(10)*t162 + a(4)*t222/t582;
dfr0(6) = -dfr0(10)*t222 - a(4)*t162/t582;
dfr1(14) = 1/2*a(8)*a(7)*t30/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(13) = 1/2*a(8)*a(7)*t10/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(12) = -1/2*a(8)*a(7)*t16/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;
dfr1(11) = -1/2*a(8)*a(7)*t22/(t10*t39 - t16*t34 - t22*t24 + t30*t44 + rt(6))^2;

jst = zeros(1,9);
jst(1) = -t101*df(1) - t161*df(4) + t221*df(3) + t301*df(2);
jst(2) = t101*df(4) - t161*df(1) - t221*df(2) + t301*df(3);
jst(3) = -t101*df(3) + t161*df(2) - t221*df(1) + t301*df(4);
jst(4) = -t102*dfr0(6) - t162*dfr0(9) + t222*dfr0(8) + t302*dfr0(7);
jst(5) = t102*dfr0(9) - t162*dfr0(6) - t222*dfr0(7) + t302*dfr0(8);
jst(6) = -t102*dfr0(8) + t162*dfr0(7) - t222*dfr0(6) + t302*dfr0(9);
jst(7) = -t10*dfr1(11) - t16*dfr1(14) + t22*dfr1(13) + t30*dfr1(12);
jst(8) = t10*dfr1(14) - t16*dfr1(11) - t22*dfr1(12) + t30*dfr1(13);
jst(9) = -t10*dfr1(13) + t16*dfr1(12) - t22*dfr1(11) + t30*dfr1(14);

  
  
  
  
  
  
  
  
  
  
end

