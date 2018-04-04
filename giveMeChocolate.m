function [deriv,tangent,rho] = giveMeChocolate(vec)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    syms t
    deriv=simplify(diff(vec, t));
    tangent=simplify(deriv/norm(deriv));
    rho=simplify(norm(diff(tangent, t))/norm(deriv));
end

