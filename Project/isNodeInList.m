function [result, openIdx] = isNodeInList(node, nodeList)
    result = false;
    openIdx = -1;
    for i = 1:numel(nodeList)
        if isequal(node, nodeList(i).pos)
            result = true;
            openIdx = i;
            break;
        end
    end
end