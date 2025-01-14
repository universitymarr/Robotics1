import matplotlib.pyplot as plt 

def plotter(X: list, Y: list, marker: str=None,
            labels: list=None, title: str="Plot", 
            xlabel: str="X-axis", ylabel: str="Y-axis"):
    """
    Plots provided curves based on x and y data

    @param X: The list of X values
    @param Y: The list of Y values
    @param marker: The data points shape 
    @param labels: The list of labels
    @param title: The title of the plot
    @param xlabel: The X-axis label
    @param ylabel: The Y-axis label
    """

    for i, y in enumerate(Y):
        label = labels[i] if labels and len(labels) > i else f"Curve {i+1}"
        plt.plot(X, y, marker=marker, label=label)
    
    plt.title(title)
    plt.xlabel(xlabel)
    plt.ylabel(ylabel)
    plt.legend()

    plt.grid()
    plt.show()