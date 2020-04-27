function y = OutputFn(xk1, uk, ...
    Ts, conflict_matrix, green_interval_matrix, yellow_time_vector, amber_time_vector, min_green_time_vector, num_signals)
    
%     r = uk(index(1, num_signals));
%     a = uk(index(3, num_signals));
%     
%     c_a = uk(index(4, num_signals));

    y = xk1(5 * num_signals + 1:end);
end


function idx = index(i, num_signals)
    idx = i * num_signals + 1:(i + 1) * num_signals;
end