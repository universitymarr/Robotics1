function[] = plot_function(f,x_i,x_f,rate)  
    %funzione che genera il plot di una qualsisasi funzione continua o
    %discontinua
    %
    %input:
    %-f = funzione definita come f=@(x) ...
    %-x_i = valore minimo della variabile
    %-x_f = valore massimo della variabile
    %-rate = rate di sampling nell'intervallo indicato
    %
    %nota: la visulaizzazione può essere migliorata modificando manualmente
    %i margini del ylim
    %
    %Esempio di funzione continua:
    %f=@(x)  x.^2
    %Esempio di funzione a tratti:
    %f=@(x) (x < 0.75) .*0 + (0.75 <= x & x < 1.5) .* x.^2  + (x >= 1.5) .* x;
    
    % Genera valori x nella gamma desiderata
    %MODIFICA L'INTERVALLO A PIACIMENTO
    x = linspace(0, 2, 1000);
    x = linspace(x_i,x_f,rate);

    % Calcola i valori della funzione per ciascun valore x
    y = f(x);

    % Fai il plot della funzione
    figure;
    plot(x, y, 'LineWidth', 2);
    title('Plot of f');
    xlabel('x');
    ylabel('f(x)');

    %MODIFICA LA GRANDEZZA DEL PLOT A PIACIMENTO
    %ylim([-1, 2]);
    grid on;