import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from matplotlib import pyplot as plt
from tigramite import data_processing as pp

DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/ppdata'
CSV_NAME = ["data_20240131_234259", "data_20240131_234529","data_20240219_122333", "data_20240219_122603"]
SELECTED_AGENT = 'h'
variables = ["_v", "_{d_g}", r"_{risk}", r"_{\theta_{g}}", r"_{\omega}", r"_{d_{obs}}"]
variables = ['$' + SELECTED_AGENT + v + '$' for v in variables]

ldf = list()
for csv in CSV_NAME:
    ldf.append(pd.read_csv(DATA_DIR + '/' + csv + '.csv'))
df = pd.concat(ldf, axis=0, ignore_index=True)

dataframe = pp.DataFrame(df.values, var_names=variables)


gpdc = GPDC(significance='analytic', gp_params=None)
gpdc.dataframe = dataframe


array, dymmy, dummy, dummy = gpdc._get_array(X=[(3, -1)], Y=[(0, 0)], Z=[(1, -1)], tau_max=1)
x, meanx = gpdc._get_single_residuals(array, target_var=0, return_means=True, return_likelihood=False)
y, meany = gpdc._get_single_residuals(array, target_var=1, return_means=True, return_likelihood=False)

fig, axes = plt.subplots(nrows=1, ncols=3, figsize=(8,3))
axes[0].scatter(array[1], array[0], color='grey', alpha=0.3)
axes[0].scatter(array[1], meanx, color='black')
axes[0].set_title("GP of %s on %s" % (variables[0], variables[1]) )
axes[0].set_xlabel(variables[1]); axes[0].set_ylabel(variables[0])
axes[1].scatter(array[2], array[1], color='grey', alpha=0.3)
axes[1].scatter(array[2], meany, color='black')
axes[1].set_title("GP of %s on %s" % (variables[2], variables[1]) )
axes[1].set_xlabel(variables[1]); axes[1].set_ylabel(variables[2])
axes[2].scatter(x, y, color='red', alpha=0.3)
axes[2].set_title("DC of residuals:" "\n val=%.3f / p-val=%.3f" % (gpdc.run_test(
            X=[(3, -1)], Y=[(0, 0)], Z=[(1, -1)], tau_max=1)) )
axes[2].set_xlabel("resid. "+variables[0]); axes[2].set_ylabel("resid. "+variables[2])
plt.tight_layout()
plt.show()