import numpy
import math
import datetime
import json
import os.path


class Carlosddd_Logmodule:
    """
    """

    def __init__(self, name="defaultLog", test_environment=False, max_slices=10):
        self.lst = []
        self.lst_ts = [] # timestamps
        self.test_environment = test_environment
        self.idx = 0
        self.max_slices = max_slices
        self.startup_ts = self.get_ts_now(with_date=True)
        if not self.test_environment:
            self.abs_path = "/data/openpilot/carlosddd/"    # in OP environment (C2) we need to use absolut path
        else:
            self.abs_path = ""  # for testing use this folder here (windows)
        self.log_path = self.abs_path + "logs/"
        self.name = name
        self.filename = self.name + "_" + self.startup_ts + "_log"
        self.log_filename_octave = self.log_path + "octave/" + self.filename + ".m"
        self.log_filename_csv = self.log_path + "csv/" + self.filename + ".csv"
        self.first_run_csv = True
        self.first_run_octave = True
        self.json_path = self.abs_path + "config.json"
        self.cnt = 0    # index of measurement-sets (= all variables to capture), multiple (max_slices) measurement-sets are collected to one slice 
        self.slice_cnt = 0  # number of slices (holdin n=max_slices measurement-sets)
        json_data = self.load_json(self.json_path)
        self.parse_json(json_data)
        #print(self.config_print_output, self.config_output_octave, self.config_output_csv)
        self.inhibit_log = not any([self.config_print_output, self.config_output_octave, self.config_output_csv])
        if self.inhibit_log:
            print(">>> carlosddd_logmodule.py: no log activity conducted")
        for i in range(self.max_slices):
            d = {}
            self.lst.append(d)
            self.lst_ts.append(0)
        #self.print_lst()

    def load_json(self, json_path):
        with open(json_path, 'r') as f:
            json_data = json.load(f)
        return json_data

    def parse_json(self, json_data):
        self.config_print_output =  bool(json_data['output_prints'])
        self.config_output_octave =  bool(json_data['output_octave'])
        self.config_output_csv =  bool(json_data['output_csv'])
        self.config_csv_seperator = str(json_data['csv_seperator'])

    def update(self, key, value, convert=False):
        if not self.inhibit_log:
            #print("Adding", value, "to key", key)
            if convert:
                if isinstance(value, bool):
                    value = 1 if value else 0
            self.lst[self.idx][key] = value
            self.lst_ts[self.idx] = self.get_ts_now()

    def slice_done(self):
        if not self.inhibit_log:
            self.idx += 1
            if self.idx >= self.max_slices:
                #print("Paket complete")
                #self.print_lst()
                self.output()
                self.idx = 0
                self.cnt += self.max_slices # do this after outputting!
                self.slice_cnt += 1

    def print_lst(self):
        for idx, d in enumerate(self.lst):
            ts = self.lst_ts[idx]
            print(ts, d)

    def get_ts_now(self, with_date=False):
        if with_date:
            return datetime.datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
        else:
            return datetime.datetime.now().strftime("%H:%M:%S.%f")

    def output(self):
        if self.config_print_output:
            self.print_output()
        if self.config_output_octave:
            self.log_output_octave()
        if self.config_output_csv:
            self.log_output_csv()

    def get_known_keys(self):
        known_keys = []
        for d in self.lst:
            for key in d:
                if key not in known_keys:
                    known_keys.append(key)
        return known_keys

    def print_output(self):
        known_keys = self.get_known_keys()
        #print("final key list", known_keys)
        out_str = ""
        out_str += "k2-plot:"
        out_str += str(self.slice_cnt) + ":"
        for idx, known_key in enumerate(known_keys):
            out_str += str(known_key) + ","
        out_str = out_str[:-1] # remove last comma
        out_str += ":"
        for key in known_keys:
            for idx, d in enumerate(self.lst):
                if key in d:
                    out_str += str(d[key])
                else:
                    out_str += 'nan'
                out_str += ","
            out_str = out_str[:-1] # remove last comma
            out_str += ";"
        out_str += ":::"
        print(out_str)

    def log_output_octave(self):
        log_file = open(self.log_filename_octave, 'a')
        known_keys = self.get_known_keys()
        # write header if necessary
        if self.first_run_octave:
            self.first_run_octave = False
            log_file.write("# this m-file (GNU Octave) contains data organized to overwrite the contents of a NaN-matrix of appropriate size" + "\n")
            log_file.write("n_row = " + str(len(known_keys)) + "\n")
            log_file.write("n_col = ?" + " # <<<--- please see index in last lines and enter here!" + "\n")
            log_file.write("inp_data = NaN(n_col, n_row);" + " # prepare matrix of appropriate dimensions" + "\n")
            log_file.write("# inp_data = " + str(known_keys) + "\n")
            for idx, known_key in enumerate(known_keys):
                log_file.write("#" + "data_" + str(known_key) + " = " + "inp_data(:," + str(idx+1) + ")" + ";" + "\n")
        # write data (GNU octave output)
        for idx_key, key in enumerate(known_keys):
            for idx_d, d in enumerate(self.lst):
                out_str = "inp_data(" + str(self.cnt+idx_d+1) + "," + str(idx_key+1) + ")" + " = "  # octave indexing starts with 1! (non-zero-based)
                if key in d:
                    out_str += str(d[key])
                else:
                    out_str += 'NaN'
                log_file.write(out_str + ";" + "\n")
        log_file.close()

    def log_output_csv(self):
        log_file = open(self.log_filename_csv, 'a')
        known_keys = self.get_known_keys()
        # write header if necessary
        if self.first_run_csv:
            self.first_run_csv = False
            out_str = "idx,timestamp,"
            for idx, known_key in enumerate(known_keys):
                out_str += str(known_key) + ","
            log_file.write(out_str[:-1] + "\n") # remove last comma
        # write data
        for idx_d, d in enumerate(self.lst):
            out_str = str(self.cnt+idx_d) + "," + self.lst_ts[idx_d] + ","
            for idx_key, key in enumerate(known_keys):
                if key in d:
                    out_str += str(d[key])
                else:
                    out_str += 'NaN'
                out_str += ","
            out_str = out_str[:-1] # remove last comma
            log_file.write(out_str + "\n")
        log_file.close()
