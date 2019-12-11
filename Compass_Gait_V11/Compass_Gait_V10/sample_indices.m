function [x_i_inds, dt_i_inds] = sample_indices(i, nx)
%SAMPLE_INDICES(i, nx) calculates the indices of z such that z(x_i_inds) = 
%x_i and z(dt_i_inds) = dt_i.
%
%   @param i: sample number; scalar
%   @param nx: dimension of state vector, x; scalar
%
%   @output x_i_inds: indices such that z(x_i_inds) = x_i; 1 by nx vector
%   @output dt_i_inds: indices such that z(dt_i_inds) = dt_i; scalar

    x_i_inds = (1:nx) + nx * (i - 1) + 1 * (i - 1);
    dt_i_inds = nx * i + i;

end

