function [deriv,tangent,rho] = giveMeChocolate(vec)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    syms t
    assume(t, 'real')
    deriv=simplify(diff(vec, t));
    tangent=simplify(deriv/norm(deriv));
    rho=simplify(norm(deriv)/(norm(diff(tangent, t))));
end

