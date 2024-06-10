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
DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/Traj_pp'
# CSV_NAME = ["causalhri_1", "causalhri_2"] # Causal-HRI
CSV_NAME = ["A1"] # A1 real world exp

ldf = list()
for csv in CSV_NAME:
    ldf.append(pd.read_csv(DATA_DIR + '/' + csv + '.csv'))
df = pd.concat(ldf, axis=0, ignore_index=True)

# df.dropna(inplace=True)

df = Data(df)
variables=["v", "d_g", "r"]

df.shrink(variables)

# df.plot_timeseries()

cdm = FPCMCI(df, 
             f_alpha = 0.5,
             pcmci_alpha = 0.05,
             min_lag = 1, 
             max_lag = 1, 
             sel_method = TE(TEestimator.Gaussian), 
             val_condtest = GPDC(),
             verbosity = CPLevel.DEBUG,
             neglect_only_autodep = False,
             resfolder = "results/" + 'new__'.join(CSV_NAME))

start = time()
feature, causalmodel = cdm.run()
# feature, causalmodel = cdm.run_pcmci()
elapsed_fpcmci = time() - start
fpcmci_time = str(timedelta(seconds = elapsed_fpcmci))
print(fpcmci_time)

if len(feature) > 0:
    cdm.dag(skip_autodep=True, node_size=5, font_size=16, label_type = LabelType.NoLabels, img_ext = ImageExt.PDF)
    cdm.timeseries_dag(node_size=5, font_size=16, img_ext = ImageExt.PDF, node_proximity=4)
    cdm.dag(skip_autodep=True, node_size=5, font_size=16, label_type = LabelType.NoLabels, img_ext = ImageExt.PNG)
    cdm.timeseries_dag(node_size=5, font_size=16, img_ext = ImageExt.PNG, node_proximity=4)