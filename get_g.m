function g = get_g(p, theta)
g = [EULERXYZ(theta), p;[0 0 0 1]];
end