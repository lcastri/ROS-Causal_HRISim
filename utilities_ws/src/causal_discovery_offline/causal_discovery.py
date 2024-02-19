from datetime import timedelta
from time import time
from matplotlib import pyplot as plt
import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from tigramite.independence_tests.cmiknn import CMIknn
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType, ImageExt

from tigramite.pcmci import PCMCI
from tigramite import plotting as tp
from tigramite import data_processing as pp

SELECTED_AGENT = 'h'

DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/ppdata'
CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240219_145325", "data_20240219_145555"]
# CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240219_122333"]
# CSV_NAME = ["data_20240219_145325", "data_20240219_145555"]

ldf = list()
for csv in CSV_NAME:
    ldf.append(pd.read_csv(DATA_DIR + '/' + csv + '.csv'))
df = pd.concat(ldf, axis=0, ignore_index=True)

df = Data(df)
# variables = ["_v", "_{d_g}", r"_{risk}"]
variables = ["_v", "_{d_g}", r"_{risk}", r"_{\theta_{g}}", r"_{\omega}", r"_{d_{obs}}"]
variables = [SELECTED_AGENT + v for v in variables]
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
             resfolder = "results/" + '__'.join(CSV_NAME))

start = time()
feature, causalmodel = cdm.run()
elapsed_fpcmci = time() - start
fpcmci_time = str(timedelta(seconds = elapsed_fpcmci))
print(fpcmci_time)

if len(feature) > 0:   
    cdm.dag(label_type = LabelType.NoLabels, img_ext = ImageExt.PDF)
    cdm.timeseries_dag(node_size=6, font_size=16, img_ext = ImageExt.PDF, node_proximity=3)
    cdm.dag(label_type = LabelType.NoLabels, img_ext = ImageExt.PNG)
    cdm.timeseries_dag(node_size=6, font_size=16, img_ext = ImageExt.PNG, node_proximity=3)

# dataframe = pp.DataFrame(df.d.values, var_names=df.features)
# pcmci = PCMCI(dataframe = dataframe, 
#               cond_ind_test = GPDC(significance = 'analytic', gp_params = None),
#               verbosity=2)

# results = pcmci.run_pcmciplus(tau_min=0, tau_max=1)


# tp.plot_graph(
#     val_matrix=results['val_matrix'],
#     graph=results['graph'],
#     var_names=variables,
#     link_colorbar_label='cross-MCI (edges)',
#     node_colorbar_label='auto-MCI (nodes)',
#     ); plt.show()

# # Plot time series graph
# tp.plot_time_series_graph(
#     figsize=(8, 8),
#     node_size=0.05,
#     val_matrix=results['val_matrix'],
#     graph=results['graph'],
#     var_names=variables,
#     link_colorbar_label='MCI',
#     ); plt.show()