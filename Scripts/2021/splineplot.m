function splineplot(tvals,qvals)

% tvals = [1 2 2.5 4];
% qvals = [45 90 -45 45];

coeffs = fliplr(splines(tvals, qvals));

syms tau t real;
pols = cell(3);
N = 4

% Plot Joint Values
subplot(1, 3, 1);
hold on
for i = 1:N-1
    taut = (t - tvals(i)) / (tvals(i+1) - tvals(i));
    pol = poly2sym(coeffs(i, :), tau);
    pol = subs(pol, tau, taut);
    pols{i} = pol;
    fplot(pol, tvals(i:i+1));
end
hold off

% Plot Velocity
subplot(1, 3, 2);
hold on
vels = cell(3);
for i = 1:N-1
    vel = diff(pols{i}, t);
    vels{i} = vel;
    fplot(vel, tvals(i:i+1));
end
hold off


% Plot Acceleration
subplot(1, 3, 3);
hold on
accs = cell(3);
for i = 1:N-1
    acc = diff(vels{i}, t);
    accs{i} = pol;
    fplot(acc, tvals(i:i+1));
end
hold off

