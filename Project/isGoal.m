    % 判断节点是否为目标节点
    function isGoal = isGoal(pos, goal)
        isGoal = norm(pos - goal) < 1e-6;
    end