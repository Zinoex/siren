function Jacobian = OutputJacobian(x, u, conflict_matrix, green_interval, yellow_time, amber_time, minimum_green_vector, num_signals)
    Jacobian = [zeros(3 * num_signals, 5 * num_signals) eye(3 * num_signals)];
end