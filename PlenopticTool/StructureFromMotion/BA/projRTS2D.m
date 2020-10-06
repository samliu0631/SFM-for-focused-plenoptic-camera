function m=projRTS2D(j, i, rt, xyz, r0, a)
% symbolic projection function
% code automatically generated with maple

% 输入：
% j:   表示相机的序号。
% i:   表示3D点的序号。这里并没有使用。
% rt： 表示当前相机在初始姿态下的局部旋转四元数。
% xyz：表示3D点的坐标。 这里注意其只表示1个3D点的坐标。
% r0:  表示传递的初始旋转矩阵。
% a:   表示相机内参。 共5个。

% 输出：
% m:   表示通过当前3D点和当前相机姿态计算出的投影观测向量。



  qr0=r0(j*4+1:(j+1)*4); % 表示第j个相机对应的初始旋转四元数。

  t1 = (rt(1) ^ 2);   
  t2 = (rt(2) ^ 2);
  t3 = (rt(3) ^ 2);
  t5 = sqrt((1 - t1 - t2 - t3));   % 计算单位四元数的第一个元素。
  
  % 计算四元数 qr0*rt
  t10 = t5 * qr0(2) + qr0(1) * rt(1) + rt(2) * qr0(4) - rt(3) * qr0(3);
  t16 = t5 * qr0(3) + qr0(1) * rt(2) + rt(3) * qr0(2) - rt(1) * qr0(4);
  t22 = t5 * qr0(4) + qr0(1) * rt(3) + rt(1) * qr0(3) - rt(2) * qr0(2);
  t30 = t5 * qr0(1) - rt(1) * qr0(2) - rt(2) * qr0(3) - rt(3) * qr0(4);
  
  % 计算 qr0*rt*[0,xyz]
  t24 = -t10 * xyz(1) - t16 * xyz(2) - t22 * xyz(3);
  t34 = t30 * xyz(1) + t16 * xyz(3) - t22 * xyz(2);
  t39 = t30 * xyz(2) + t22 * xyz(1) - t10 * xyz(3);
  t44 = t30 * xyz(3) + t10 * xyz(2) - t16 * xyz(1);
  
  % 计算 qr0*rt*[0,xyz]*(qr0*rt)^-1
  t100 = ( -t24 * t10 + t30 * t34 - t39 * t22 + t44 * t16 + rt(4) ); % 旋转后的坐标。
  t52 = -t24 * t16 + t30 * t39 - t44 * t10 + t34 * t22 + rt(5); % 旋转后的Y坐标
  t58 = -t24 * t22 + t30 * t44 - t34 * t16 + t39 * t10 + rt(6); %旋转后的Z坐标
  t61 = 0.1e1 / t58;
  
  % 根据内参计算投影。
  m(1) = (     a(1) * t100    +      a(2) * t52    +    a(3) * t58   ) * t61;
  % u=（fx*X+r*Y+u0*Z）/Z
  m(2) = (     a(4) * t52      +    a(5) * t58     ) * t61;
  % v=（fy*Y+v0*Z）/Z

