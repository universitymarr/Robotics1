function plot_multiple_functions(functions, interval_start, interval_end)
    % Input:
    %   - functions: cell array of function handles, e.g., {@sin, @cos, @(x) x.^2}
    %   - interval: intervallo su cui fare il plot, e.g., [0, 2*pi]
    
    % Creazione del vettore x sull'intervallo specificato
    x = linspace(interval_start, interval_end, 1000);

    % Inizializzazione della figura
    figure;

    % Iterazione attraverso le funzioni e plot
    for i = 1:length(functions)
        y = arrayfun(functions{i}, x);
        plot(x, y, 'DisplayName', func2str(functions{i}));
        hold on;
    end

    % Aggiunta di legenda, etichette degli assi, e titolo
    legend('show');
    xlabel('X-axis');
    ylabel('Y-axis');
    title('Plot di Funzioni Multiple');

    % Opzionale: aggiunta di una griglia
    grid on;

    % Rilascio del "hold"
    hold off;
end