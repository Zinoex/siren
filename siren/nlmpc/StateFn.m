function xk1 = StateFn(xk, uk)

    xk1 = xk;

    % Times
    g_idx = index(0);
    r_idx = index(1);
    y_idx = index(2);
    a_idx = index(3);
    ng_idx = index(4);

    g = uk(g_idx);
    r = uk(r_idx);
    y = uk(y_idx);
    a = uk(a_idx);

    xk1(g_idx) = (xk(g_idx) + 1) .* g;
    xk1(r_idx) = (xk(r_idx) + 1) .* r;
    xk1(y_idx) = (xk(y_idx) + 1) .* y;
    xk1(a_idx) = (xk(a_idx) + 1) .* a;
    xk1(ng_idx) = (xk(ng_idx) + 1) .* (1 - g);

    % Queue params
    c_a = uk(index(4));
    c_d = uk(index(5));
    q_idx = index(5);
    s_idx = index(6);
    tq_idx = index(7);

    % Replace c_a and c_d with uk(end idxs)
    xk1(q_idx) = rectifier(xk(q_idx) + c_a - c_d .* (g + y));
    xk1(s_idx) = xk(s_idx) + c_a .* (r + a);
    xk1(tq_idx) = (xk(tq_idx) + 1) .* indicator((1 - g) .* xk1(q_idx));

function idx = index(i)
    idx = i * num_signals + 1:(i + 1) * num_signals;

function y = indicator(x)
    y = x >= 1;

function y = rectifier(x)
    y = max(x, zeros(size(x)));