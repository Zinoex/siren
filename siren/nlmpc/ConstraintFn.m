function ceq = ConstraintFn(X, U, data)

% Binary
U_stacked = reshape(U, [], 1);
U_size = size(U_stacked, 1);

ceq(1:U_size) = U_stacked == 0 | U_stacked == 1;