#!/usr/bin/env python3

import statistics
import warnings
import csv

import numpy as np
from scipy import optimize, integrate
from scipy.signal import resample, savgol_filter

#from src.graph_output.graph_2d import graph_2d
#from src.graph_output.html_output import html_output


class MotherWavelet:
    def __init__(self, wavelet_length=60, wavelet_shift=0, std_bound_mult=1):
        self.wavelet_length = wavelet_length
        self.wavelet_shift = wavelet_shift
        self.std_bound_multiplier = std_bound_mult

        self.stored_signals = []

        self.mother_wavelet_raw = []
        self.mother_stdev = []

        self.mother_wavelet_pfit_eq = None
        self.polyfit_graph_output = []
        self.polyfit_graph_string = ''
        self.mother_wavelet = []

        self.f_s = 50 # sampling frequency


    ##### Scales and returns mother wavelet. Scaled using (1/scale) (larger scale is shorter wave)
    def get_mother_wavelet(self, scale_in, scale_type="length"):
        # scale_type can be ["length", "scale"]
        if scale_type == "scale":
            # if scale value input, convert to length before resampling
            scale = 1/(scale_in/self.f_s)
        else:
            scale = scale_in

        #current_length = len(self.mother_wavelet)
        #goal_length = round(current_length / scale)
        goal_length = round(self.f_s * scale)
        scaled_mother_wav = resample(self.mother_wavelet, goal_length)
        return list(scaled_mother_wav)



    ##### Function to add a new signal to the database
    def add(self, sig, perturbation_frame=None, process=True, cut_frames=[]):
        if not cut_frames:
            min_idx = sig.index(min(sig[perturbation_frame-10:perturbation_frame+40])) + self.wavelet_shift
            self.stored_signals.append(sig[min_idx-round(self.wavelet_length/2):min_idx+round(self.wavelet_length/2)])
        else:
            self.stored_signals.append(
                resample(
                    sig[cut_frames[0]:cut_frames[1]], self.wavelet_length
                )
            )

        if process:
            self.regenerate()
        pass


    ##### Generate mother wavelet
    def regenerate(self):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", np.RankWarning)

            self.mother_wavelet_raw = []
            self.mother_stdev = []
            for i in range(0, self.wavelet_length):
                signal_vars = [signal[i] for signal in self.stored_signals]
                self.mother_wavelet_raw.append(np.mean(signal_vars))
                try:
                    self.mother_stdev.append(statistics.stdev(signal_vars))
                except statistics.StatisticsError:
                    self.mother_stdev.append(0)

            self.polyfitting(self.mother_wavelet_raw)

            self.output_to_file()

            pass


    #####
    def polyfitting(self, y_input):
        ##### Create X-array from 0-1 with proper number of values
        x = np.arange(0, 1, 1 / len(y_input))

        ##### Normalize input signal in X and Y
        # set bottom of graph to 0
        y_shift = min(y_input)
        y = y_input - y_shift
        # Normalize y-values to a scale of 0-1
        y_max = max(y)
        y_shift = y_shift / y_max
        y = [(_ / y_max) + y_shift for _ in y]

        ##### Smooth the y-array
        y = list(savgol_filter(y, int(len(y)/10), 3))

        ##### get best polynomial fit to start optimization from
        # get euclidean distance for each degree polynomial
        distances = []
        for _ in range(1, 30):
            pfit_coeffs = np.polyfit(x, y, deg=_)
            pfit_eq = np.poly1d(pfit_coeffs)
            pfit_y = pfit_eq(x)
            # metric_distance = sum([abs(x1 - x2)**2 for x1, x2 in zip(y, pfit_y)])
            metric_distance = 1 / sum(np.convolve(y, pfit_y))
            distances.append(metric_distance)
        # find lowest distance (best fit) and get degree number
        best_degree = distances.index(min(distances)) + 1
        pfit_coeffs = np.polyfit(x, y, deg=best_degree)


        ##### Optimize polynomial coefficients to get admissibility and better match
        optimized_coeffs = optimize.minimize(self.optimize_polyfit_coeffs, pfit_coeffs.copy(), args=(x, y))

        # determine equation and curve for initial optimization
        pfit_eq = np.poly1d(pfit_coeffs)
        pfit_y = pfit_eq(x)

        # determine equation for optimized coefficients
        pfit_eq_opt = np.poly1d(optimized_coeffs.x)

        # use equation to get points of optimized curve
        # lots of points so it can be resampled to anything
        x2 = np.arange(0,1,0.001)
        pfit_y_opt = pfit_eq_opt(x2)
        pfit_y_opt_zero = pfit_y_opt - np.mean(pfit_y_opt)
        pfit_y_opt_zero_integral = integrate.cumtrapz(pfit_y_opt_zero, x2)

        self.polyfit_graph_string = f"Number of Signals: {len(self.stored_signals)}<br>Optimal Degree: {best_degree}<br>Mean: {np.mean(pfit_y_opt_zero)}<br>Integral: {pfit_y_opt_zero_integral[-1]-pfit_y_opt_zero_integral[0]}"
        self.polyfit_graph_output = [y, pfit_y, pfit_eq_opt(x)-np.mean(pfit_eq_opt(x))]

        self.mother_wavelet_pfit_eq = pfit_eq_opt

        self.mother_wavelet = pfit_y_opt_zero


    ##### Function to output a single value representative of how close the polyfit is to the input signal
    # Optimally used with scipy.optimize.minimize
    def optimize_polyfit_coeffs(self, coeffs, *data):
        x, y = data

        pfit_y = np.poly1d(coeffs)
        pfit_y_zeroed = pfit_y - np.mean(pfit_y)

        ##### get measures of success
        # euclidean distance
        metric_distance_arr = [abs(x1 - x2) ** 10 for x1, x2 in zip(y, pfit_y)]
        metric_distance = sum(metric_distance_arr)
        # metric_distance = 1/abs(sum(np.correlate(y, pfit_y)))

        # integral (last-first)
        metric_integral = integrate.cumtrapz(pfit_y_zeroed)
        metric_integral = abs(metric_integral[-1] - metric_integral[0])

        # print(metric_integral, metric_distance)
        return (metric_distance * metric_integral) ** abs(metric_distance_arr[-1])



    ##### Call to output an overview of the wavelet to an html file
    def output_to_file(self, filename="generated_wavelet"):
        html_graphs = []

        html_graphs.append(self.graph_polyfit())
        html_graphs.append(self.graph_mother_wavelet())
        html_graphs.append(self.graph_input_signals())

        try:
            html_output(
                filename,
                html_graphs,
                top_matter=self.polyfit_graph_string,
                output_dir="../../data_processed"
            )
        except:
            try:
                html_output(
                    filename,
                    html_graphs,
                    top_matter=self.polyfit_graph_string,
                    output_dir="../data_processed"
                )
            except:
                html_output(
                    filename,
                    html_graphs,
                    top_matter=self.polyfit_graph_string,
                    output_dir="data_processed"
                )

    ##### Function to output html graph object with all input signals on the same chart
    def graph_input_signals(self):
        g = graph_2d(
            f"{len(self.stored_signals)} signals with {self.wavelet_length} frames used to form Mother Wavelet",
            self.stored_signals
        )
        return g


    ##### Function to output html graph object with all input signals on the same chart
    def graph_mother_wavelet(self):
        g = graph_2d(
            f"Averaged Mother Wavelet with {self.std_bound_multiplier} STD bounds",
            [self.mother_wavelet_raw, [x + y for x, y in zip(self.mother_wavelet_raw, self.mother_stdev)], [x - y for x, y in zip(self.mother_wavelet_raw, self.mother_stdev)]],
            legend_names=["Average", f"Upper {self.std_bound_multiplier}-std", f"Lower {self.std_bound_multiplier}-std"]
        )
        return g


    ##### Function to output html graph object for fitted wavelet with admissibility conditions
    def graph_polyfit(self):
        g = graph_2d(
            f"Curve Fitted Mother Wavelet with Admissibility Conditions",
            self.polyfit_graph_output,
            legend_names=['Input', 'Initial Fit', 'Optimized Fit'],
            x=np.arange(0,1,1/len(self.polyfit_graph_output[0]))
        )
        return g


    def save_mother(self, filename):
        # what is the minimal amount of different variables that can be saved?
        # write wavelet_length on line 1
        # write wavelet_shift on line 2
        # write std_bound_multiplier on line 3
        # write 1 stored signal per line after that

        with open(filename+".csv", "w+", newline='') as f:
            csvwriter = csv.writer(f, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)

            csvwriter.writerow([self.wavelet_length])
            csvwriter.writerow([self.wavelet_shift])
            csvwriter.writerow([self.std_bound_multiplier])
            for r in self.stored_signals:
                csvwriter.writerow(r)


    def load_mother(self, filename):
        with open(filename+".csv", "r", newline='') as f:
            csvreader = csv.reader(f, delimiter=' ', quotechar='|')

            # reset object
            self.__init__()

            # read wavelet_length from file
            for row in csvreader:
                self.wavelet_length = int(row[0])
                break

            # read wavelet_shift from file
            for row in csvreader:
                self.wavelet_shift = int(row[0])
                break

            # read std_bound_multiplier from file
            for row in csvreader:
                self.wavelet_shift = int(row[0])
                break

            # iterate over the rest of the file and save all signals to array
            for row in csvreader:
                self.stored_signals.append([float(_) for _ in row])

        self.regenerate()
