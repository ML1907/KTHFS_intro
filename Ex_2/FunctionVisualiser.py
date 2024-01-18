#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import math

class FunctionVisualiser(object):
    """
    class to plot and visualise the function statically within a specified interval [a, b]
    """
    def __init__(self, function, a, b):
        self.function = function
        self.time = np.linspace(a, b, int((b - a) * 200))
        self.fig, self.ax = plt.subplots(figsize=(15, 8), constrained_layout=True)
        self.ax.set_xlabel('time')
        self.ax.set_ylabel('value')
        self.ax.set_title('Exercise 2 kthfs_intro')
        self.plot, = self.ax.plot([], [], label='function')  # create an empty plot
        self.ax.legend()

    def main(self):
        times = []
        values = []
        for t in self.time:
            function_value = self.function(t)
            times.append(t)
            values.append(function_value)
        self.update_plot(times, values)

    def update_plot(self, times, values):
        self.plot.set_data(times, values)
        self.ax.relim()           # adjust axis limits with new data
        self.ax.autoscale_view()  # ensure that updated data range is visible
        plt.draw()                # draw the plot

class FunctionVisualiser_RealTime(FunctionVisualiser):
    """
    class to plot and visualise the function evolution within a specified interval [a, b]
    """
    def __init__(self, function, a, b):
        plt.ion()                       # Enable interactive mode
        super(FunctionVisualiser_RealTime, self).__init__(function, a, b)  # Call the constructor of the parent class

    def main(self):
        times = []
        values = []
        for t in self.time:
            function_value = self.function(t)
            times.append(t)
            values.append(function_value)
            self.update_plot(times, values)

    def update_plot(self, times, values):
        self.plot.set_data(times, values)
        self.ax.relim()           # adjust axis limits with new data
        self.ax.autoscale_view()  # ensure that updated data range is visible
        plt.draw()                # redraw the plot
        plt.pause(0.001)          # give time to the plot to update before re-drawing


class PeriodicFunctionVisualiser_RealTime(FunctionVisualiser_RealTime):
    """
    class to plot and visualise the function evolution within a specified interval [a, b]. The class allows to visualise non-redundant information about this function
    """
    def __init__(self, function, a, b, tolerance=1):
        super(PeriodicFunctionVisualiser_RealTime, self).__init__(function, a, b)
        self.tolerance = tolerance
        self.periodic_count = 0

    def is_periodic(self, values):
        n = len(values)
        if n > 2:
            # Check if the last value is close to the first value. It has to happen 2 times
            if abs(values[-1] - values[0]) < self.tolerance:
                self.periodic_count += 1
                if self.periodic_count == 2:
                    self.periodic_count = 0
                    return True
        return False

    def main(self):
        times = []
        values = []
        for t in self.time:
            function_value = self.function(t)
            times.append(t)
            values.append(function_value)
            self.update_plot(times, values)

            # Check for periodicity
            if self.is_periodic(values):
                print("Function is periodic. Stopping.")
                break



# function declaration
def my_function(t):
    l = 5 * np.sin(2 * math.pi * 1 * t)
    h = 3 * math.pi * np.exp(-l)
    return h

def my_function2(t):
    return t


a, b = 0 , 5  # Set the visualisation time interval [a, b]

## Uncomment a visualiser of your choice

#visualiser = FunctionVisualiser(my_function, a, b)
#visualiser = FunctionVisualiser_RealTime(my_function, a, b)
visualiser = PeriodicFunctionVisualiser_RealTime(my_function, a, b)
visualiser.main()
plt.show(block=True)   # Until you close the plot window, it does not disappear


