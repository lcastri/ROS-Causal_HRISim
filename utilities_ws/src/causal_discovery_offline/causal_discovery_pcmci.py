from datetime import timedelta
from time import time
from matplotlib import pyplot as plt
import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType, ImageExt


# SELECTED_AGENT = 'h'
DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/ppdata'
CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240225_230735"]
# CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240225_221217"]
# CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240219_122333"]
# CSV_NAME = ["data_20240219_145325", "data_20240219_145555"]
CSV_NAME = ["data_20240311_121511"]

ldf = list()
for csv in CSV_NAME:
    ldf.append(pd.read_csv(DATA_DIR + '/' + csv + '.csv'))
df = pd.concat(ldf, axis=0, ignore_index=True)

# df.dropna(inplace=True)

df = Data(df)
# variables = ["_v", "_{d_g}", r"_{risk}"]
# variables = ["_v", "_{d_g}", r"_{risk}", r"_{\theta_{g}}", r"_{\omega}"]
# variables = ["g_r", "v", "d_g", "r", r"d_{obs}", r"\theta_{g}", r"\omega"]
# variables = ["g_r", "v", "d_g", "r", r"\theta_{g}", r"\omega"]
variables=["d_g", "v", "r"]
# variables=["v", "d_g", "r", r"d_{obs}", r"\theta_{g}", r"\omega"]

# variables = [SELECTED_AGENT + v for v in variables]
df.shrink(variables)

# df.plot_timeseries()

cdm = FPCMCI(df, 
             f_alpha = 0.75,
             pcmci_alpha = 0.05,
             min_lag = 1, 
             max_lag = 1, 
             sel_method = TE(TEestimator.Kraskov), 
             val_condtest = GPDC(),
             verbosity = CPLevel.DEBUG,
             neglect_only_autodep = False,
             resfolder = "results/" + '__'.join(CSV_NAME))

start = time()
feature, causalmodel = cdm.run_pcmci()
elapsed_fpcmci = time() - start
fpcmci_time = str(timedelta(seconds = elapsed_fpcmci))
print(fpcmci_time)

if len(feature) > 0:
    cdm.dag(skip_autodep=True, node_size=5, font_size=14, label_type = LabelType.NoLabels, img_ext = ImageExt.PDF)
    cdm.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PDF, node_proximity=3)
    cdm.dag(skip_autodep=True, node_size=5, font_size=14, label_type = LabelType.NoLabels, img_ext = ImageExt.PNG)
    cdm.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PNG, node_proximity=3)