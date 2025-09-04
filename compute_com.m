function com = compute_com(centers, masses)
    % 计算总体质心位置
    % 输入:
    %   centers: 4x2 矩阵, 每行是一个质心 [x, y]
    %   masses: 1x4 或 4x1 矩阵, 代表每个质点的质量
    % 输出:
    %   com: 1x2 矩阵, [x_com, y_com]

    total_mass = sum(masses); % 总质量
    com_x = sum(centers(:,1) .* masses) / total_mass;
    com_y = sum(centers(:,2) .* masses) / total_mass;
    
    com = [com_x, com_y];
end
