function [L1, L2] = project_com(com, p1, p2)
    % 计算总质心在投影线上离两端的长度
    % 输入:
    %   com: 1x2 矩阵, 质心坐标 [x, y]
    %   p1: 1x2 矩阵, 线段起点 [x1, y1]
    %   p2: 1x2 矩阵, 线段终点 [x2, y2]
    % 输出:
    %   L1: 质心到 p1 的距离
    %   L2: 质心到 p2 的距离

    % 计算投影点
    v = p2 - p1; % 线段方向向量
    v_unit = v / norm(v); % 归一化方向向量
    vec_cp = com - p1; % 质心到 p1 的向量
    proj_length = dot(vec_cp, v_unit); % 投影长度

    % 计算离两端的距离
    L1 = proj_length; % 质心到 p1 的距离
    L2 = norm(p2 - p1) - L1; % 质心到 p2 的距离
end
