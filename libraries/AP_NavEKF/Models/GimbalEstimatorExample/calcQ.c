  t3 = q0*q0;
  t4 = q1*q1;
  t5 = q2*q2;
  t6 = q3*q3;
  t2 = t3+t4+t5+t6;
  t7 = t2*t2;
  t11 = q0*q3*2.0;
  t12 = q1*q2*2.0;
  t8 = t11-t12;
  t13 = q0*q2*2.0;
  t14 = q1*q3*2.0;
  t9 = t13+t14;
  t10 = t3+t4-t5-t6;
  t15 = q0*q1*2.0;
  t16 = t11+t12;
  t17 = dvxNoise*t10*t16;
  t18 = t3-t4+t5-t6;
  t19 = q2*q3*2.0;
  t20 = t15-t19;
  t21 = t15+t19;
  t22 = t3-t4-t5+t6;
  t23 = t13-t14;
  t24 = dvzNoise*t9*t22;
  t25 = t24-dvxNoise*t10*t23-dvyNoise*t8*t21;
  t26 = dvyNoise*t18*t21;
  t27 = t26-dvxNoise*t16*t23-dvzNoise*t20*t22;
  A0[0][0] = daxNoise*t7;
  A0[1][1] = dayNoise*t7;
  A0[2][2] = dazNoise*t7;
  A0[3][3] = dvxNoise*(t10*t10)+dvyNoise*(t8*t8)+dvzNoise*(t9*t9);
  A0[3][4] = t17-dvzNoise*t9*(t15-q2*q3*2.0)-dvyNoise*t8*t18;
  A0[3][5] = t25;
  A0[4][3] = t17-dvyNoise*t8*t18-dvzNoise*t9*t20;
  A0[4][4] = dvxNoise*(t16*t16)+dvyNoise*(t18*t18)+dvzNoise*(t20*t20);
  A0[4][5] = t27;
  A0[5][3] = t25;
  A0[5][4] = t27;
  A0[5][5] = dvxNoise*(t23*t23)+dvyNoise*(t21*t21)+dvzNoise*(t22*t22);
