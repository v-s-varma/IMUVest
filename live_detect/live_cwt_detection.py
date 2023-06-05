#!/usr/bin/env python3

# contains class for CWT live detection
# includes built in filters and derivatives so gyroscope values can be passed in directly
# If run directly, this file will use the live CWT class to analyze the files below

from live_detect.store_MotherWavelet import *
from scipy import signal
import numpy as np



class detect_cwt():
    def __init__(self, mother_wavelet_obj, scale=2, scale_type="length", gy_store_frames=20, detect_thresh=300, f_s=45, derivative=True, filter=True):
        # argument inputs
        self.mother_wavelet = mother_wavelet_obj.get_mother_wavelet(scale, scale_type=scale_type)
        self.detect_thresh = detect_thresh
        self.derivative = derivative
        self.filter = filter

        self.gy_store_frames = max(len(self.mother_wavelet)+1, gy_store_frames)

        # variable storage arrays
        self.time_store_gy = [0 for _ in range(0, self.gy_store_frames)]
        self.window_store_gy = [0 for _ in range(0, self.gy_store_frames)]

        # butterworth filter setup
        butterworth_order = 3
        butterworth_cutoff = 3  # Hz

        #self.b_gy, self.a_gy = signal.butter(butterworth_order, butterworth_cutoff, fs=f_s)
        self.b_gy, self.a_gy = signal.iirfilter(2, 2, btype="lowpass", ftype="bessel", output="ba", fs=50)
        self.z_gy = signal.lfilter_zi(self.b_gy, self.a_gy) * 0

        self.test_store_arr = []


    def update(self, gy_val, timestamp, output_type="value"):
        # main function to insert a new value for detection
        # input values for lower back gyroscope and timestamp
        # output_type can be "value" or "boolean"

        # get time since last run, update time
        self.time_store_gy.append(timestamp)
        self.time_store_gy.pop(0)

        if self.filter:
            # filter gyroscope values as they come in, save to sliding window
            gy_filt, self.z_gy = signal.lfilter(self.b_gy, self.a_gy, [gy_val], zi=self.z_gy)
            self.window_store_gy.append(gy_filt[-1])
        else:
            self.window_store_gy.append(gy_val)
        self.window_store_gy.pop(0)

        # compute the acceleration over time
        acc_arr = np.diff(self.window_store_gy) / np.diff(self.time_store_gy)

        # compute the convolution of the stored signal with the mother wavelet
        cwt_arr = np.convolve(self.mother_wavelet, acc_arr, "same")

        # grab last value of convolution to output for testing
        out_val = cwt_arr[-1]

        # store testing values
        self.test_store_arr.append(acc_arr[-1])

        # get T/F value for detection
        out_bool = int(out_val > self.detect_thresh)

        # check output type and return relevant values
        if output_type == "value":
            return out_val
        elif output_type == "boolean":
            return out_bool
        else:
            return out_val, out_bool





if __name__ == "__main__":

    ##### IMPORT AND SETUP

    import numpy as np

    from src.data_processing.remove_standing import *
    from src.file_io.import_csv import *
    from src.graph_output.graph_2d import *
    from src.graph_output.html_output import *
    from src.detect_postprocess.cwt import *
    from src.data_processing.get_analysis_signal import *

    # import source signal from file, initialize detector with src
    mwav_obj = MotherWavelet()
    mwav_obj.load_mother("../../cwt_mother_save")

    # save and filter filenames for analysis
    #filenames = ['../../data/chadi_trip_full/algDump_chadi_2_12_lateNorm1.txt',
    #             '../../data/chadi_trip_full/algDump_chadi_2_12_lateSlow1.txt',
    #             '../../data/chadi_trip_full/algDumpChadi_earlyTrip_1.txt',
    #             '../../data/chadi_trip_full/algDumpChadi_earlyTrip_3.txt',
    #             '../../data/chadi_comparison/chadi_trips_2_12_21_walk_bend_walk_shoe2.csv',
    #             '../../data/duncan_activities/20220428_sitting1.txt']
    filenames = [
        'chadi_trips_2_12_21_walk_bend_walk_fall1.csv',
        'chadi_trips_2_12_21_walk_bend_walk_fall2.csv',
        'chadi_trips_2_12_21_walk_bend_walk_fall3.csv',
        'chadi_tripTesting_2_1_21_Early1.csv',
        'chadi_tripTesting_2_1_21_Late1.csv',
        'chadi_tripTesting_2_1_21_Mid1.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Early2.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Early3.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Early4.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Early5.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Late2.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Late3.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Mid2.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Walk1.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Walk2.csv',
        'chadi_tripTesting_2_1_21_Early_Mid_Late_Walk3.csv',
        'chadi_trips_2_12_21_walk_bend_walk_shoe1.csv',
        'chadi_trips_2_12_21_walk_bend_walk_shoe2.csv'
    ]
    filenames = ["../../data/chadi_comparison" + "/" + _ for _ in filenames]

    test_scales = [0.5, 1, 2, 5]
    test_scales_chadi = [np.round(50/_) for _ in test_scales]


    ##### PROCESS

    # Iterate through filenames and store output graphs in g array
    g = []
    for name in filenames:

        # import and process data
        file_dict, unprocessed_lines = import_csv(name)
        file_dict = remove_standing_imudict(file_dict)

        output_vals = []
        output_legend = []

        for _x in range(0, len(test_scales)):
            # get trip indicator from DTW over time
            val_arr = []
            detector = detect_cwt(mwav_obj, scale=test_scales[_x], gy_store_frames=50, filter=False, detect_thresh=2000)
            for _ in range(1, len(file_dict["gy z lowback"])):
                val = detector.update(file_dict["gy z lowback"][_], file_dict["time"][_], output_type="value")
                val_arr.append( val)#*1000 )
            output_vals.append(val_arr)
            output_legend.append(f"a={test_scales_chadi[_x]}")



        ##### OUTPUT


        g.append("<hr><hr>")
        g.append(f"<h2>{name}</h2>")

        #scale_idx = 0
        #anal_sig = get_analysis_signal("lowback", file_dict)
        #cwt_post = cwt(anal_sig, return_type="data", wavelet_object=mwav_obj, scale_values=test_scales)
        #output_vals.append(cwt_post[scale_idx])
        #output_legend.append(f"PP-Scale: {test_scales[scale_idx]}")

        # graph output value alongside loadcell reading
        try:
            output_vals.append(np.abs(file_dict["loadcell"]))
            output_legend.append(f"Loadcell")
        except:
            pass



        g.append(
            graph_2d(
                "Various scales from live object",
                output_vals,
                legend_names=output_legend,
            )
        )

        #g.append(
        #    graph_2d(
        #        "Filtered signals comparison",
        #        [detector.test_store_arr, anal_sig]
        #    )
        #)


    ##### SAVE

    html_output(
        f"cwt_live_demo",
        g,
        output_dir="../../data_processed"
    )
