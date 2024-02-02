from datetime import timedelta
from time import time
import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType, ImageExt

SELECTED_AGENT = 'h'

DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
# CSV_NAME = ["data_20240131_182712", "data_20240131_182942"]
CSV_NAME = ["data_20240131_234259", "data_20240131_234529"]
ldf = list()
for csv in CSV_NAME:
    ldf.append(pd.read_csv(DATA_DIR + '/' + csv + '.csv'))
df = pd.concat(ldf, axis=0, ignore_index=True)

df = Data(df)
# variables = ["_myv", "_my{d_g}", r"_{risk}"]
variables = ["_v", "_{d_g}", r"_{risk}"]
variables = [SELECTED_AGENT + v for v in variables]
df.shrink(variables)

df.plot_timeseries()

cdm = FPCMCI(df, 
             f_alpha = 0.05,
             pcmci_alpha = 0.05,
             min_lag = 1, 
             max_lag = 1, 
             sel_method = TE(TEestimator.Gaussian), 
             val_condtest = GPDC(significance = 'analytic', gp_params = None),
             verbosity = CPLevel.DEBUG,
             neglect_only_autodep = False,
             resfolder = "results/CIAO")
            #  resfolder = "results/" + '__'.join(CSV_NAME))

start = time()
feature, causalmodel = cdm.run()
elapsed_fpcmci = time() - start
fpcmci_time = str(timedelta(seconds = elapsed_fpcmci))
print(fpcmci_time)
# feature, causalmodel = cdm.run_pcmci()

if len(feature) > 0:   
    cdm.dag(label_type = LabelType.Lag, img_ext = ImageExt.PDF)
    cdm.timeseries_dag(node_size=6, font_size=16, img_ext = ImageExt.PDF)