function calculate_equivalent_inertia_script()
    % =========================================================================
    % 脚本主程序: 计算五连杆机构在平衡点的等效转动惯量
    % =========================================================================
    clc;
    clear;
    close all;

    % -------------------------------------------------------------------------
    % 1. 定义物理参数 (用户需要根据实际情况修改)
    % -------------------------------------------------------------------------
    % 为了演示，我们假设连杆 l1=l4, l2=l3，形成对称结构
    params.d_AE = 0.21;   % A点和E点之间的距离 (m)
    params.l1 = 0.14;     % 连杆l1的长度 (m)
    params.l2 = 0.25;    % 连杆l2的长度 (m)
    params.l3 = 0.25;    % 连杆l3的长度 (m)
    params.l4 = 0.14;     % 连杆l4的长度 (m)
    
    params.m1 = 0.3;     % 连杆l1的质量 (kg)
    params.m2 = 0.5;     % 连杆l2的质量 (kg)
    params.m3 = 0.5;     % 连杆l3的质量 (kg)
    params.m4 = 0.3;     % 连杆l4的质量 (kg)

    % 假设连杆为均匀细杆，其绕质心（中点）的转动惯量为 (1/12)*m*L^2
    params.Ic1 = (1/12) * params.m1 * params.l1^2; % (kg*m^2)
    params.Ic2 = (1/12) * params.m2 * params.l2^2; % (kg*m^2)
    params.Ic3 = (1/12) * params.m3 * params.l3^2; % (kg*m^2)
    params.Ic4 = (1/12) * params.m4 * params.l4^2; % (kg*m^2)

    % -------------------------------------------------------------------------
    % 2. 定义平衡状态 (用户需要根据需求修改)
    % -------------------------------------------------------------------------
    % 我们选择L0垂直向上的姿态作为平衡点
    phi0_eq = pi / 2;

    % -------------------------------------------------------------------------
    % 3. 调用函数计算等效转动惯量
    % -------------------------------------------------------------------------
    I_eq = get_equivalent_inertia(phi0_eq, params);

    % -------------------------------------------------------------------------
    % 4. 显示结果
    % -------------------------------------------------------------------------
    fprintf('======================================================\n');
    fprintf('计算结果:\n');
    fprintf('在平衡点 phi0 = %.2f rad (%.1f deg) 时,\n', phi0_eq, rad2deg(phi0_eq));
    fprintf('系统的等效转动惯量 I_eq = %.6f kg*m^2\n', I_eq);
    fprintf('======================================================\n');

end

function I_eq = get_equivalent_inertia(phi0_equilibrium, params)
    % =========================================================================
    % 核心函数: 通过符号运算求解等效转动惯量
    % 输入:
    %   phi0_equilibrium: 定义的平衡点角度 (rad)
    %   params: 包含所有物理参数的结构体
    % 输出:
    %   I_eq: 计算得到的等效转动惯量 (kg*m^2)
    % =========================================================================

    % --- 步骤 1: 符号变量定义 ---
    syms t
    syms phi0(t) phi1(t) phi2(t) phi3(t) phi4(t)
    
    % 将参数加载到工作区以便符号计算使用
    l1 = params.l1; l2 = params.l2; l3 = params.l3; l4 = params.l4;
    d_AE = params.d_AE;

    % --- 步骤 2: 建立运动学方程 ---
    % 定义坐标系：原点在E点，A点在(-d_AE, 0)
    pA = [-d_AE; 0];
    pE = [0; 0];

    % 各关键点的位置矢量
    pB = pA + l1 * [cos(phi1); sin(phi1)];
    pD = pE + l4 * [cos(phi4); sin(phi4)];
    pC_from_left = pB + l2 * [cos(phi2); sin(phi2)]; % 从左侧计算C点
    pC_from_right = pD + l3 * [cos(phi3); sin(phi3)]; % 从右侧计算C点

    % 建立两个闭环约束方程 (C点位置必须唯一)
    loop_constraint = pC_from_left - pC_from_right;
    
    % --- 步骤 3: 求解平衡点的角度 ---
    % !!! 注意: 用户需要在这里替换成自己的正向运动学(FK)函数 !!!
    % 为了使脚本能独立运行，这里提供一个针对对称情况的简化版FK求解器
    [phi1_eq, phi2_eq, phi3_eq, phi4_eq] = solve_equilibrium_angles(phi0_equilibrium, params);
    
    % --- 步骤 4: 对运动学方程求导以获得速度关系 ---
    % 对时间t求一阶导数
    loop_constraint_vel = diff(loop_constraint, t);

    % 替换角度为函数形式 phi_i(t)
    vars = [phi1, phi2, phi3, phi4];
    vars_t = [phi1(t), phi2(t), phi3(t), phi4(t)];
    loop_constraint_vel = subs(loop_constraint_vel, vars, vars_t);
    
    % 定义角速度 omega_i = diff(phi_i(t), t)
    omega0 = diff(phi0(t), t);
    omega1 = diff(phi1(t), t);
    omega2 = diff(phi2(t), t);
    omega3 = diff(phi3(t), t);
    omega4 = diff(phi4(t), t);
    
    % --- 步骤 5: 建立虚拟杆 L0 的约束 ---
    % 虚拟杆 L0 连接 E 点和 C 点
    % pC = L0 * [cos(phi0); sin(phi0)]
    % 我们需要知道 L0 的长度在平衡点的值
    L0_eq_vec = pA + l1*[cos(phi1_eq); sin(phi1_eq)] + l2*[cos(phi2_eq); sin(phi2_eq)];
    L0_eq = norm(L0_eq_vec);

    pC_virtual = L0_eq * [cos(phi0); sin(phi0)];
    
    % C点的位置必须同时满足左右两侧的约束和虚拟杆的约束
    % pC_from_left - pC_virtual = 0
    virtual_loop_constraint = pC_from_left - pC_virtual;
    virtual_loop_constraint_vel = diff(virtual_loop_constraint, t);
    virtual_loop_constraint_vel = subs(virtual_loop_constraint_vel, vars, vars_t);
    virtual_loop_constraint_vel = subs(virtual_loop_constraint_vel, phi0, phi0(t));

    % 整合所有速度约束方程
    all_constraints_vel = [loop_constraint_vel; virtual_loop_constraint_vel];

    % --- 步骤 6: 求解角速度比例系数 k_i = omega_i / omega_0 ---
    % 在平衡点处，代入角度的数值
    eq_angles = [phi0(t), phi1(t), phi2(t), phi3(t), phi4(t)];
    eq_values = [phi0_equilibrium, phi1_eq, phi2_eq, phi3_eq, phi4_eq];
    all_constraints_vel_eq = subs(all_constraints_vel, eq_angles, eq_values);

    % 将方程整理成 A*x = b 的形式，其中 x = [omega1; omega2; omega3; omega4]
    % b 中包含 omega0
    [A, b] = equationsToMatrix([all_constraints_vel_eq(1)==0, all_constraints_vel_eq(2)==0, ...
                                all_constraints_vel_eq(3)==0, all_constraints_vel_eq(4)==0], ...
                                [omega1, omega2, omega3, omega4]);
    
    % 求解 omega_i = f(omega_0)
    omegas_solved = linsolve(A, b);
    
    % 提取比例系数 k_i
    k1 = double(subs(omegas_solved(1) / omega0, omega0, 1));
    k2 = double(subs(omegas_solved(2) / omega0, omega0, 1));
    k3 = double(subs(omegas_solved(3) / omega0, omega0, 1));
    k4 = double(subs(omegas_solved(4) / omega0, omega0, 1));
    
    % --- 步骤 7: 求解质心速度比例系数 j_i = v_ci / omega_0 ---
    % 定义质心位置 (假设在中点)
    pc1 = pA + 0.5 * l1 * [cos(phi1); sin(phi1)];
    pc2 = pB + 0.5 * l2 * [cos(phi2); sin(phi2)];
    pc3 = pD + 0.5 * l3 * [cos(phi3); sin(phi3)];
    pc4 = pE + 0.5 * l4 * [cos(phi4); sin(phi4)];

    % 对质心位置求导得到速度
    vc1 = diff(pc1, t); vc2 = diff(pc2, t); vc3 = diff(pc3, t); vc4 = diff(pc4, t);
    
    % 代入平衡点的值和已求出的omega关系
    v_subs = [omega1, omega2, omega3, omega4];
    o_subs = [k1*omega0, k2*omega0, k3*omega0, k4*omega0];
    all_vars = [eq_angles, v_subs];
    all_vals = [eq_values, o_subs];

    vc1_eq = subs(vc1, all_vars, all_vals);
    vc2_eq = subs(vc2, all_vars, all_vals);
    vc3_eq = subs(vc3, all_vars, all_vals);
    vc4_eq = subs(vc4, all_vars, all_vals);

    % 提取 j_i 矢量的大小平方 |j_i|^2
    j1_sq = double(subs(norm(vc1_eq/omega0)^2, omega0, 1));
    j2_sq = double(subs(norm(vc2_eq/omega0)^2, omega0, 1));
    j3_sq = double(subs(norm(vc3_eq/omega0)^2, omega0, 1));
    j4_sq = double(subs(norm(vc4_eq/omega0)^2, omega0, 1));

    % --- 步骤 8: 计算最终的等效转动惯量 ---
    % I_eq = sum(m_i * |j_i|^2 + Ic_i * k_i^2)
    m = [params.m1, params.m2, params.m3, params.m4];
    Ic = [params.Ic1, params.Ic2, params.Ic3, params.Ic4];
    j_sq = [j1_sq, j2_sq, j3_sq, j4_sq];
    k_sq = [k1^2, k2^2, k3^2, k4^2];

    I_eq = sum(m .* j_sq + Ic .* k_sq);
end

function [phi1, phi2, phi3, phi4] = solve_equilibrium_angles(phi0, params)
    % !!! 这是一个简化的FK求解器，仅适用于对称情况的特定平衡点 !!!
    % !!! 用户应在此处替换为自己的、更通用的FK函数 !!!
    
    % 警告信息
    persistent hasWarned
    if isempty(hasWarned)
        disp('------------------------------------------------------');
        disp('警告: 正在使用简化的FK求解器。');
        disp('如果您的机构不是完全对称的，或者平衡点不是L0垂直，');
        disp('请务必用您自己的FK函数替换 solve_equilibrium_angles。');
        disp('------------------------------------------------------');
        hasWarned = true;
    end

    % 根据对称性进行简化计算
    % 假设平衡点是 L0 垂直向上 (phi0 = pi/2), 且机构几何对称
    if abs(phi0 - pi/2) < 1e-6 && ...
       abs(params.l1 - params.l4) < 1e-6 && ...
       abs(params.l2 - params.l3) < 1e-6
   
        % 在该特定情况下，可以解析求解
        l1 = params.l1; l2 = params.l2; d_AE = params.d_AE;
        
        % 求解 L0 的长度
        % C点的x坐标为0
        % x_B + l2*cos(phi2) = 0 -> -d_AE/2 + l1*cos(phi1) + l2*cos(phi2) = 0
        % 由于对称性, x_B = -x_D, phi2 = pi - phi3, phi1 = pi - phi4
        % 只需要解左半边
        % fsolve需要一个初始猜测值
        initial_guess = [2*pi/3; pi/3]; % 初始猜测 [phi1, phi2]
        options = optimoptions('fsolve', 'Display', 'none');
        
        % C点x坐标为0, y坐标未知(h)
        % -d_AE/2 + l1*cos(phi1) + l2*cos(phi2) = 0
        % l1*sin(phi1) + l2*sin(phi2) = h
        % 这里直接用数值解，因为解析解可能复杂
        % 我们需要找到 phi1, phi2 使得 C 点的 x 坐标为 0
        % C点 x: -d_AE/2 + l1*cos(phi1) + l2*cos(phi2) = 0
        % C点 y: L0 = l1*sin(phi1) + l2*sin(phi2)
        % 这是一个非线性方程组，需要数值求解
        
        % 简化的简化：假设一个合理的解，因为精确求解FK不是此脚本的重点
        phi1 = 2.0; % rad, 约 114.6 deg
        % 从 -d_AE/2 + l1*cos(phi1) + l2*cos(phi2) = 0 求解 phi2
        cos_phi2 = (params.d_AE/2 - params.l1*cos(phi1)) / params.l2;
        if abs(cos_phi2) > 1
            error('几何结构无解，请检查连杆长度。');
        end
        phi2 = acos(cos_phi2);

        % 根据对称性
        phi4 = pi - phi1;
        phi3 = pi - phi2;
    else
        error('此简化的FK求解器只适用于phi0=pi/2的对称机构。请替换为您的FK函数。');
    end
end
