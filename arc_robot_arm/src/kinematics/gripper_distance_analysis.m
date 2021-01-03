theta = pi()/4:-pi()/32:0;
dist = [0.1,.094-.016, .075-.007, .05,0.045,0.04,0.032,0.022,0];
dist_pred = 0.02;

coefs = polyfit(theta, dist, 1);

b = coefs(2) * -1
m = 1 / coefs(1)

pred = m * dist_pred + b;