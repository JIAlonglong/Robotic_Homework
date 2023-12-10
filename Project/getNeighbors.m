function neighbors = getNeighbors(pos, obstacles)
    % 生成相邻节点的坐标矩阵
    % 输入参数：
    %   pos: 当前节点的坐标 [x, y, z]
    %   obstacles: 障碍物坐标矩阵，每行表示一个障碍物的坐标 [x, y, z]
    % 输出参数：
    %   neighbors: 相邻节点的坐标矩阵，每行表示一个相邻节点的坐标 [x, y, z]

    % 定义相邻节点的偏移量
    offsets = [1, 0, 0; -1, 0, 0; 0, 1, 0; 0, -1, 0; 0, 0, 1; 0, 0, -1];

    % 初始化相邻节点坐标矩阵
    neighbors = repmat(pos, 6, 1) + offsets;

    % 过滤掉在障碍物中的相邻节点
    validIdx = ~ismember(neighbors, obstacles, 'rows');
    neighbors = neighbors(validIdx, :);
end