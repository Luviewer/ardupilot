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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 第i个旋转坐标系1到机体坐标系的旋转矩阵
% 旋转角度
syms angle_1 real;

%旋转矩阵
b_C_i1 = subs(Cpitch', pitch, angle_1);

%% 第i个机体系到旋转坐标系3的位置，在机体系
% lfront_x x轴距离，lfront_y y轴距离
syms lfront_x lfront_y lrear_x real;

i_Front_R_Position = [lfront_x, lfront_y, 0]';
i_Front_L_Position = [-lfront_x, lfront_y, 0]';
i_Rear_Position = [0, -lrear_x, 0]';

% a1前右，a2后置，a3前左
syms a1 a2 a3 real;
syms F1 F2 F3 real;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 合力计算
syms Fi real;
i3_Fi = [0, 0, -Fi]';
b_Fi = b_C_i1 * i3_Fi;

%% 代入第i组旋转机构等到总合力
b_F = subs(b_Fi, [Fi, angle_1], [F1, a1]) + subs(b_Fi, [Fi, angle_1], [F2, a2]) + subs(b_Fi, [Fi, angle_1], [F3, a3]);
b_F = simplify(b_F);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 合力矩计算
syms Qi real;

%% 代入第i组旋转机构等到总合力矩
b_M = subs(cross(i_Front_R_Position, b_Fi), [Fi, angle_1], [F1, a1]) + subs(cross(i_Rear_Position, b_Fi), [Fi, angle_1], [F2, a2]) + subs(cross(i_Front_L_Position, b_Fi), [Fi, angle_1], [F3, a3]);
b_M = simplify(b_M);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 控制分配
b_FM = [b_F; b_M];
b_FM = [b_FM(1, :); b_FM(3:6, :)];

b_FM_var = [Cpitch'*b_F; Cpitch'*b_M];
b_FM_var = expand(b_FM_var);
b_FM_var = [b_FM_var(1, :); b_FM_var(3:6, :)];


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

b_FM_alloc = subs(b_FM_var, [F1*sin(a1), F1*cos(a1), F2*sin(a2), F2*cos(a2), F3*sin(a3), F3*cos(a3)], n);

F_alloc = [
    diff(b_FM_alloc, f1_s1), diff(b_FM_alloc, f1_c1), diff(b_FM_alloc, f2_s2), diff(b_FM_alloc, f2_c2), diff(b_FM_alloc, f3_s3), diff(b_FM_alloc, f3_c3)
    ];

disp('F_alloc = ');
disp(F_alloc);
disp('========================================');

F_pinv = simplify(pinv(F_alloc));
disp('F_pinv = ');
disp(F_pinv);
disp('========================================');

% ========================================
% F_alloc = 
% [      -1,         0,       -1,       0,       -1,         0]
% [       0,        -1,        0,      -1,        0,        -1]
% [       0, -lfront_y,        0, lrear_x,        0, -lfront_y]
% [       0,  lfront_x,        0,       0,        0, -lfront_x]
% [lfront_y,         0, -lrear_x,       0, lfront_y,         0]
 
% ========================================
% F_pinv = 
% [-lrear_x/(2*(lfront_y + lrear_x)),                                 0,                           0,               0, 1/(2*(lfront_y + lrear_x))]
% [                                0, -lrear_x/(2*(lfront_y + lrear_x)), -1/(2*(lfront_y + lrear_x)),  1/(2*lfront_x),                          0]
% [   -lfront_y/(lfront_y + lrear_x),                                 0,                           0,               0,    -1/(lfront_y + lrear_x)]
% [                                0,    -lfront_y/(lfront_y + lrear_x),      1/(lfront_y + lrear_x),               0,                          0]
% [-lrear_x/(2*(lfront_y + lrear_x)),                                 0,                           0,               0, 1/(2*(lfront_y + lrear_x))]
% [                                0, -lrear_x/(2*(lfront_y + lrear_x)), -1/(2*(lfront_y + lrear_x)), -1/(2*lfront_x),                          0]

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 新的控制分配矩阵（包含pitch）
F_alloc_new = [
    -cos(pitch), -sin(pitch), -cos(pitch), -sin(pitch), -cos(pitch), -sin(pitch);
     sin(pitch), -cos(pitch),  sin(pitch), -cos(pitch),  sin(pitch), -cos(pitch);
    lfront_y*sin(pitch), -lfront_y*cos(pitch), -lrear_x*sin(pitch), lrear_x*cos(pitch), lfront_y*sin(pitch), -lfront_y*cos(pitch);
    0, lfront_x, 0, 0, 0, -lfront_x;
    lfront_y*cos(pitch), lfront_y*sin(pitch), -lrear_x*cos(pitch), -lrear_x*sin(pitch), lfront_y*cos(pitch), lfront_y*sin(pitch)
];

disp('========================================');
disp('新的控制分配矩阵（包含pitch）:');
disp('========================================');
disp('F_alloc_new = ');
disp(F_alloc_new);
disp('========================================');

%% 提取pitch：将矩阵分解为 F_alloc_new = T_pitch * F_alloc_base
%% 方法1：提取基础矩阵（将pitch设为0）
F_alloc_base = subs(F_alloc_new, pitch, 0);
F_alloc_base = simplify(F_alloc_base);

disp('基础矩阵（pitch=0）:');
disp('F_alloc_base = ');
disp(F_alloc_base);
disp('========================================');

%% 方法2：通过矩阵分解提取pitch相关的变换
%% 观察矩阵结构，可以写成 F_alloc_new = T_pitch * F_alloc_base 的形式
%% 其中T_pitch是依赖于pitch的变换矩阵

% 定义基础矩阵（不包含pitch）
F_alloc_base_manual = [
    -1, 0, -1, 0, -1, 0;
     0, -1, 0, -1, 0, -1;
     0, -lfront_y, 0, lrear_x, 0, -lfront_y;
     0, lfront_x, 0, 0, 0, -lfront_x;
     lfront_y, 0, -lrear_x, 0, lfront_y, 0
];

% 尝试通过求解 T_pitch = F_alloc_new / F_alloc_base 来提取变换矩阵
% 但由于矩阵不是方阵，需要使用伪逆
T_pitch = F_alloc_new * pinv(F_alloc_base_manual);
T_pitch = simplify(T_pitch);

disp('提取的pitch变换矩阵:');
disp('T_pitch = F_alloc_new * pinv(F_alloc_base)');
disp('T_pitch = ');
disp(T_pitch);
disp('========================================');

%% 方法3：直接提取每列的cos(pitch)和sin(pitch)系数
% 将F_alloc_new写成 F_alloc_new = F_alloc_cos * cos(pitch) + F_alloc_sin * sin(pitch) + F_alloc_const

% 提取常数项（不包含pitch的项）
F_alloc_const = simplify(subs(F_alloc_new, [cos(pitch), sin(pitch)], [0, 0]));

% 提取cos(pitch)的系数：令cos(pitch)=1, sin(pitch)=0，然后减去常数项
F_alloc_with_cos = subs(F_alloc_new, [cos(pitch), sin(pitch)], [1, 0]);
F_alloc_cos = simplify(F_alloc_with_cos - F_alloc_const);

% 提取sin(pitch)的系数：令cos(pitch)=0, sin(pitch)=1，然后减去常数项
F_alloc_with_sin = subs(F_alloc_new, [cos(pitch), sin(pitch)], [0, 1]);
F_alloc_sin = simplify(F_alloc_with_sin - F_alloc_const);

disp('矩阵分解为: F_alloc_new = F_alloc_cos*cos(pitch) + F_alloc_sin*sin(pitch) + F_alloc_const');
disp('F_alloc_cos (cos(pitch)的系数矩阵) = ');
disp(F_alloc_cos);
disp('F_alloc_sin (sin(pitch)的系数矩阵) = ');
disp(F_alloc_sin);
disp('F_alloc_const (常数项矩阵) = ');
disp(F_alloc_const);
disp('========================================');

%% 验证分解
F_alloc_reconstructed = F_alloc_cos*cos(pitch) + F_alloc_sin*sin(pitch) + F_alloc_const;
F_alloc_reconstructed = simplify(F_alloc_reconstructed);
disp('验证分解正确性:');
disp('F_alloc_reconstructed = F_alloc_cos*cos(pitch) + F_alloc_sin*sin(pitch) + F_alloc_const');
disp('是否相等:');
disp(isequal(simplify(F_alloc_new - F_alloc_reconstructed), zeros(5, 6)));
disp('========================================');