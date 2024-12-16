function spline_plot_norm(tvals, coeffs)

    % NOTE:
    % It works only with normalized splines


    % tvals = [1 2 2.5 4];
    % qvals = [45 90 -45 45];

    %[coeffs, v] = splines(tvals, qvals);
    coeffs = fliplr(coeffs);

    syms tau t real;
    pols = cell(3);
    N = 4

    % Plot Joint Values
    subplot(1, 3, 1); title('Joint Values');
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
    subplot(1, 3, 2); title('Velocity');
    hold on
    vels = cell(3);
    for i = 1:N-1
        vel = diff(pols{i}, t);
        vels{i} = vel;
        fplot(vel, tvals(i:i+1));
    end
    hold off


    % Plot Acceleration
    subplot(1, 3, 3);  title('Acceleration');
    hold on
    accs = cell(3);
    for i = 1:N-1
        acc = diff(vels{i}, t);
        accs{i} = pol;
        fplot(acc, tvals(i:i+1));
    end
    hold off

