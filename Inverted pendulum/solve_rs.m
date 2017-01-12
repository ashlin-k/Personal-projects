function [rdd, sdd] = solve_rs()

syms r r_d r_dd s s_d s_dd x_dd y_dd z_dd g L

%r_dd = (1/((L^2 - s^2)*zeta)) * ( -r^4*x_dd - (L^2 - s^2)^2*x_dd - 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd) + r^3*(s_d^2 + s*s_dd - zeta*(g+z_dd)) + r*(-L^2*s*s_dd + s^3*s_dd + s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))) );

zeta = sqrt(L^2 - r^2 - s^2);
f = (1/((L^2 - s^2)*zeta));
A = r^4*x_dd;
B = (L^2 - s^2)^2*x_dd;
C = 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd);
D = r*(s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd)));

sdd_temp = ((r_dd/f) + A + B + C - D - r^3*(s_d^2 - zeta*(g+z_dd))) / (r^3*s - r*L*s + r*s^3);

rdd_temp = subs(((1/((L^2 - s^2)*zeta)) * ( -r^4*x_dd - (L^2 - s^2)^2*x_dd - 2*r^2*(s*r_d*s_d + (-L^2 + s^2)*x_dd) + r^3*(s_d^2 + s*s_dd - zeta*(g+z_dd)) + r*(-L^2*s*s_dd + s^3*s_dd + s^2*(r_d^2 - zeta*(g+z_dd)) + L^2*(-r_d^2 - s_d^2 + zeta*(g+z_dd))))), s_dd, (((r_dd/f) + A + B + C - D - r^3*(s_d^2 - zeta*(g+z_dd))) / (r^3*s - r*L*s + r*s^3)));

rdd = solve(r_dd == rdd_temp, r_dd);

sdd = ((rdd/f) + A + B + C - D - r^3*(s_d^2 - zeta*(g+z_dd))) / (r^3*s - r*L*s + r*s^3);
