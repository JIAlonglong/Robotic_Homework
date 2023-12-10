function path = Astar(start, goal, obstacles)
    % A*算法函数封装
    % 输入参数：
    %   start: 起始点坐标 [x, y, z]
    %   goal: 目标点坐标 [x, y, z]
    %   obstacles: 障碍物坐标矩阵，每行表示一个障碍物的坐标 [x, y, z]
    % 输出参数：
    %   path: 路径，每行表示路径上的一个点的坐标 [x, y, z]

    % 定义启发式函数
    heuristic = @(pos) sqrt((pos(1) - goal(1))^2 + (pos(2) - goal(2))^2 + (pos(3) - goal(3))^2);

    % 初始化起始节点和开放列表
    startNode = struct('pos', start, 'g', 0, 'h', heuristic(start), 'f', 0, 'parent', []);
    openList = startNode;
    closedList = [];

    % 开始搜索
    while ~isempty(openList)
        % 选择f值最小的节点作为当前节点
        [~, idx] = min([openList.f]);
        currentNode = openList(idx);

        % 判断是否到达目标节点
        if isGoal(currentNode.pos, goal)
            path = reconstructPath(currentNode);
            return;
        end

        % 将当前节点从开放列表移至关闭列表
        openList(idx) = [];
        closedList = [closedList, currentNode];

        % 扩展当前节点的相邻节点
        neighborsa = getNeighbors(currentNode.pos, obstacles);
        for i = 1:5
            neighbor = neighborsa(i,:);
            if isNodeInList(neighbor, closedList)
                continue;
            end

            % 计算相邻节点的g值和h值
            g = currentNode.g + distance(currentNode.pos, neighbor);
            h = heuristic(neighbor);

            % 判断是否已经在开放列表中
            [isInOpenList,openIdx] = isNodeInList(neighbor, openList);
            if isInOpenList
                if g < openList(openIdx).g
                    % 更新相邻节点的g值和f值
                    openList(openIdx).g = g;
                    openList(openIdx).f = g + h;
                    openList(openIdx).parent = currentNode;
                end
            else
                % 添加相邻节点到开放列表
                newNode = struct('pos', neighbor, 'g', g, 'h', h, 'f', g + h, 'parent', currentNode);
                openList = [openList, newNode];
            end
        end
    end

    % 未找到路径
    path = [];
    disp('No path found.');
end