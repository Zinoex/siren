function xk1 = StateFn(xk, uk, ...
    conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, minimum_green_vector, num_signals)

    xk1 = xk;

    % Times
    g_idx = index(0, num_signals);
    r_idx = index(1, num_signals);
    y_idx = index(2, num_signals);
    a_idx = index(3, num_signals);
    ng_idx = index(4, num_signals);

    mv1 = uk(index(0, num_signals));
    mv2 = uk(index(1, num_signals));
    
    g = mv1 < 0.5 & mv2 < 0.5;
    r = mv1 >= 0.5 & mv2 >= 0.5;
    y = mv1 < 0.5 & mv2 >= 0.5;
    a = mv1 >= 0.5 & mv2 < 0.5;

    xk1(g_idx) = (xk(g_idx) + 1) .* g;
    xk1(r_idx) = (xk(r_idx) + 1) .* r;
    xk1(y_idx) = (xk(y_idx) + 1) .* y;
    xk1(a_idx) = (xk(a_idx) + 1) .* a;
    xk1(ng_idx) = (xk(ng_idx) + 1) .* (1 - g);

    % Queue params
    c_a = uk(index(2, num_signals));
    c_d = uk(index(3, num_signals));
    q_idx = index(5, num_signals);
    s_idx = index(6, num_signals);
    tq_idx = index(7, num_signals);

    % Replace c_a and c_d with uk(end idxs)
    xk1(q_idx) = rectifier(xk(q_idx) + c_a - c_d .* (g + y));
    xk1(s_idx) = xk(s_idx) + c_a .* (r + a);
    xk1(tq_idx) = (xk(tq_idx) + 1) .* indicator((1 - g) .* xk1(q_idx));

function idx = index(i, num_signals)
    idx = i * num_signals + 1:(i + 1) * num_signals;

function y = indicator(x)
    y = x >= 1;

function y = rectifier(x)
    y = max(x, zeros(size(x)));