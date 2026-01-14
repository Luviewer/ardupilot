clear; clc;
syms m g pie real;

%% 机体到地理坐标系的旋转矩阵
syms yaw roll pitch real;

Cyaw = [
        cos(yaw), sin(yaw), 0;
        -sin(yaw), cos(yaw), 0;
        0, 0, 1;
        ]; %z

Cpitch = [
          cos(pitch), 0, -sin(pitch);
          0, 1, 0;
          sin(pitch), 0, cos(pitch);
          ]; %y

Croll = [
         1, 0, 0;
         0, cos(roll), sin(roll);
         0, -sin(roll), cos(roll);
         ]; %x

Cbn = Croll * Cpitch * Cyaw;
Cnb = Cbn';

%% 第i个旋转坐标系1到机体坐标系的旋转矩阵
% 旋转角度
syms angle_1 real;

%旋转矩阵
b_C_i1 = subs(Cpitch', pitch, angle_1);

%% 第i个机体系到旋转坐标系3的位置，在机体系
% lfront_x x轴距离，lfront_y y轴距离
syms lfront_x lfront_y lrear real;


i1_P1 = [lfront_x, lfront_y, 0]';
b_Pi = Cyaw * i1_P1;

%% 合力计算
syms Fi real;
i3_Fi = [0, 0, -Fi]';
b_Fi = b_C_i1 * i3_Fi;

%% 合力矩计算
syms Qi real;
b_Mi = cross(b_Pi, b_Fi);

%% 代入第i组旋转机构等到总合力
% a1前右，a2后置，a3前左
syms a1 a2 a3 real;
syms F1 F2 F3 real;

b_F = subs(b_Fi, [Fi, angle_1, yaw], [F1, a1, 0]) + subs(b_Fi, [Fi, angle_1, yaw, lfront_x, lfront_y], [F2, a2, pi, lrear, 0]) + subs(b_Fi, [Fi, angle_1, yaw], [F3, a3, pi]);
b_F = simplify(b_F);

%% 代入第i组旋转机构等到总合力矩
b_M = subs(b_Mi, [Fi, angle_1, yaw], [F1, a1, 0]) + subs(b_Mi, [Fi, angle_1, yaw, lfront_x, lfront_y], [F2, a2, pi, lrear, 0]) + subs(b_Mi, [Fi, angle_1, yaw], [F3, a3, pi]);
b_M = simplify(b_M);

%% 控制分配
b_FM = [b_F; b_M];
b_FM = [b_FM(1, :); b_FM(3:6, :)];

%% 输出控制分配矩阵
disp('========================================');
disp('控制分配矩阵（力和力矩）:');
disp('========================================');
disp('b_FM = ');
disp(b_FM);
disp('========================================');
disp('矩阵说明：');
disp('第1行：Fx (X方向力) = -Fi*(sin(a1) + sin(a2) + sin(a3))');
disp('第2行：Fz (Z方向升力) = -Fi*(cos(a1) + cos(a2) + cos(a3))');
disp('第3行：Mx (Roll力矩) = -Fi*lfront_y*(cos(a1) - cos(a3))');
disp('第4行：My (Pitch力矩) = -Fi*lfront_x*(cos(a3) - cos(a1) + cos(a2))');
disp('第5行：Mz (Yaw力矩) = Fi*lfront_y*(sin(a1) - sin(a3))');
disp('========================================');
disp('其中：');
disp('  a1 = 前右倾转角度');
disp('  a2 = 后置倾转角度');
disp('  a3 = 前左倾转角度');
disp('  lfront_x = X轴方向距离');
disp('  lfront_y = Y轴方向距离');
disp('  Fi = 第i个电机的推力');
disp('========================================');
disp('混控因子推导说明：');
disp('1. Roll因子：前左和前右受cos(θ)影响，后置lfront_y=0不贡献');
disp('2. Pitch因子：三个电机都贡献，前左cos(a3)为负，前右cos(a1)为正，后置cos(a2)为正');
disp('3. Yaw因子：只有前左和前右贡献（后置lfront_y=0），受sin(θ)影响');
disp('4. Throttle因子：三个电机都受cos(θ)影响（升力损失）');
disp('========================================');

syms f1_s1 f2_s2 f3_s3 real;
syms f1_c1 f2_c2 f3_c3 real;

n = [
    f1_s1, f1_c1, f2_s2, f2_c2, f3_s3, f3_c3
    ];

b_FM_alloc = subs(b_FM, [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)], n);

F_alloc = [
    diff(b_FM_alloc, f1_s1), diff(b_FM_alloc, f1_c1), diff(b_FM_alloc, f2_s2), diff(b_FM_alloc, f2_c2), diff(b_FM_alloc, f3_s3), diff(b_FM_alloc, f3_c3)
    ];


% F_alloc =
 
% [      -1,         0, -1,      0,        -1,         0]
% [       0,        -1,  0,     -1,         0,        -1]
% [       0, -lfront_y,  0,      0,         0,  lfront_y]
% [       0,  lfront_x,  0, -lrear,         0, -lfront_x]
% [lfront_y,         0,  0,      0, -lfront_y,         0]


% >> simplify(pinv(F_alloc))
 
% ans =
 
% [-1/3,    0,                                     0,           0,  1/(2*lfront_y)]
% [   0, -1/2, (lfront_x - lrear)/(2*lfront_y*lrear), 1/(2*lrear),               0]
% [-1/3,    0,                                     0,           0,               0]
% [   0,    0,            -lfront_x/(lfront_y*lrear),    -1/lrear,               0]
% [-1/3,    0,                                     0,           0, -1/(2*lfront_y)]
% [   0, -1/2, (lfront_x + lrear)/(2*lfront_y*lrear), 1/(2*lrear),               0]