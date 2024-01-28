import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType
from fpcmci.preprocessing.subsampling_methods.Static import Static

SELECTED_AGENT = 'h'

DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
# Replace 'file1.csv' and 'file2.csv' with the paths to your CSV files
file1_path = DATA_DIR + '/raw20240128_162414.csv'
file2_path = DATA_DIR + '/raw20240128_164122.csv'

# Load CSV files into pandas DataFrames
df1 = pd.read_csv(file1_path)
df2 = pd.read_csv(file2_path)

# Concatenate DataFrames along rows (axis=0)
result_df = pd.concat([df1, df2], axis=0, ignore_index=True)

# df = Data(DATA_DIR + '/' + CSV_NAME + ".csv")
df = Data(result_df, subsampling=Static(2))
variables = ["_v", "_{d_g}", r"_{risk}"]
# variables = ["_v", "_{d_g}", r"_{risk}", r"_{\omega}", r"_{\theta_{gr}}"]
# variables = ["_v", r"_{\theta}", r"_{\theta_{g}}", "_{d_g}", "_{risk}", r"_{\omega}", r"_{d_{obs}}"]
variables = [SELECTED_AGENT + v for v in variables]
df.shrink(variables)

df.plot_timeseries()
        
cdm = FPCMCI(df, 
             f_alpha = 0.05,
             pcmci_alpha = 0.05,
             min_lag = 1, 
             max_lag = 5, 
             sel_method = TE(TEestimator.Gaussian), 
             val_condtest = GPDC(significance = 'analytic', gp_params = None),
             verbosity = CPLevel.DEBUG,
             neglect_only_autodep = False,
             resfolder = "PUTTANA")

feature, causalmodel = cdm.run()
# feature, causalmodel = cdm.run_pcmci()

if len(feature) > 0:   
    cdm.dag(label_type = LabelType.Lag)
    cdm.timeseries_dag()