function path_s = calc_s(path)
    path_s = [0];
    for i = 1:length(path(:,1)) - 1
        s = hypot(path(i + 1, 1) - path(i, 1), path(i + 1, 2) - path(i, 2));
        path_s = [path_s; s];
    end
    path_s = cumsum(path_s);
end