function [arr_cars, dep_cars] = simulate_traffic(r, g, y, a, arr_rates, dep_rates)
% Uses Poisson Distribution to simulate cars arriving and departing intersection
%     Inputs:
%         r, g, y, a - column vector of boolean values representing the light in each lane
%         arr_rates, dep_rates - arrival and departure rates of cars (for each lane)
%             **Rates should be specified with units of cars/timestep
%     Outputs:
%           arr_cars, dep_cars - number of cars arriving and departing each
%           lane (whole numbers)
    sz = size(r);
    input_sz_match = all(size(g) == sz & size(y) == sz & size(a) == sz & size(arr_rates) == sz & size(dep_rates) == sz);
    assert(input_sz_match & sz(1)==1, "Error, inputs vectors must have same length.");   
    arr_cars = poissrnd(arr_cars_mean);
    dep_cars = poissrnd(dep_cars_mean);
    dep_cars = dep_cars .* (g + y);
