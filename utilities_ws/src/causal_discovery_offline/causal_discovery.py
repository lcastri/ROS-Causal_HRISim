from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType

SELECTED_AGENT = 'h'

DATA_DIR = '/home/lucacastri/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
csv_name = 'raw20240125_154606'

df = Data(DATA_DIR + '/' + csv_name + ".csv")

variables = ["_v", "_{d_g}", r"_{d_{obs}}", r"_{\theta_{gr}}", r"_{\omega}"]
# variables = ["_v", r"_{\theta}", r"_{\theta_{g}}", "_{d_g}", "_{risk}", r"_{\omega}", r"_{d_{obs}}"]
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
             neglect_only_autodep = True,
             resfolder = csv_name)

feature, causalmodel = cdm.run()
# feature, causalmodel = cdm.run_pcmci()

if len(feature) > 0:   
    cdm.dag(label_type = LabelType.NoLabels)
    cdm.timeseries_dag()