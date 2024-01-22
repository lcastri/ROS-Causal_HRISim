import numpy
import pandas as pd
from pyparsing import col
import tigramite.data_processing as pp
from tigramite.pcmci import PCMCI
from tigramite.independence_tests import GPDC, GPDCtorch
from constants import *
from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning
from log import log
import utils


class causal_model:
    def __init__(self, filename, vars_name, alpha):
        """
        Class constructor

        Args:
            filename (str): data file .csv to open
        """
        self.vars_name = vars_name[:-1]
        self.alpha = alpha
        self.df_traj = pd.DataFrame(columns = ['x', 'y'])
        self.df = pd.read_csv(utils.get_csv_path(filename), usecols=self.vars_name)
        self.inference_dict = dict()
        self.human_id = pd.read_csv(utils.get_csv_path(filename), usecols=["human_id"], nrows = 1).iloc[0]['human_id']
    

    @ignore_warnings(category=ConvergenceWarning)
    def run_causal_discovery_algorithm(self):
        """
        Run causal discovery algorithm

        Build a dictionary describing the inference between variables

        Convert dictionary to JSON string
        """
        # build tigramite dataset
        vector = numpy.vectorize(float)
        data = vector(self.df)
        dataframe = pp.DataFrame(data = data,
                                 var_names = self.vars_name)

        # init and run pcmci
        cond_ind_test = GPDC(significance = 'analytic', gp_params = None)
        pcmci = PCMCI(dataframe = dataframe,
                      cond_ind_test = cond_ind_test,
                      verbosity = 0)

        result = pcmci.run_pcmci(tau_max = 1,
                                 tau_min = 1,
                                 pc_alpha = 0.05)

        self.__log_result(result['p_matrix'],
                          result['val_matrix'])

        self.inference_dict = self.__build_causal_model(result['p_matrix'], result['val_matrix'])

    
    def __build_causal_model(self, pval_matrix, val_matrix):
        """
        Build a dictionary containing inference information 
        between variables for any time lag previously defined

        Args:
            pval_matrix (ndarray): pvalue matrix returning from causal discovery algorithm
            val_matrix (ndarray): value matrix returning from causal discovery algorithm

        Returns:
            (dictionary{str : ndarray}): causal model dictionary
        """

        # number of lags and vars definition
        nlags = val_matrix.shape[-1]
        nvars = val_matrix.shape[0]

        # filter causal model by alpha
        for lag in range(0, nlags):
            for var in range(0, nvars):
                for parent in range(0, nvars):
                    if pval_matrix[var, parent, lag] > self.alpha:
                        val_matrix[var, parent, lag] = 0
        
        # building dictionary
        dict_cm = dict()
        for lag in range(0, nlags):
            inference = numpy.zeros((3,3))
            for var in range(0, nvars):
                inference[var] = val_matrix[var][:,lag].transpose()
            dict_cm[lag] = inference.tolist()
        return dict_cm


    def __log_result(self,
                     p_matrix,
                     val_matrix,
                     conf_matrix=None,
                     graph=None,
                     ambiguous_triples=None,
                     alpha_level=0.05):
        """
        _summary_

        Args:
            p_matrix (array-like): Must be of shape (N, N, tau_max + 1)
            val_matrix (array-like): Must be of shape (N, N, tau_max + 1)
            conf_matrix (array-like, optional): Matrix of confidence intervals of shape (N, N, tau_max+1, 2). Defaults to None.
            graph (array-like, optional): Must be of shape (N, N, tau_max + 1). Defaults to None.
            ambiguous_triples (list, optional): List of ambiguous triples. Defaults to None.
            alpha_level (float, optional): Significance level. Defaults to 0.05.
        """
        if graph is not None:
            sig_links = (graph != "")*(graph != "<--")
        else:
            sig_links = (p_matrix <= alpha_level)
        string = "\n## Significant links at alpha = " + str(alpha_level) + ":"
        for j in range(len(self.vars_name)):
            links = {(p[0], -p[1]): numpy.abs(val_matrix[p[0], j, abs(p[1])])
                     for p in zip(*numpy.where(sig_links[:, j, :]))}
            # Sort by value
            sorted_links = sorted(links, key=links.get, reverse=True)
            n_links = len(links)
            string += ("\n    Variable %s has %d "
                      "link(s):" % (self.vars_name[j], n_links))
            for p in sorted_links:
                string += ("\n        (%s % d): pval = %.5f" %
                           (self.vars_name[p[0]], p[1],
                            p_matrix[p[0], j, abs(p[1])]))
                string += " | val = % .3f" % (
                    val_matrix[p[0], j, abs(p[1])])
                if conf_matrix is not None:
                    string += " | conf = (%.3f, %.3f)" % (
                        conf_matrix[p[0], j, abs(p[1])][0],
                        conf_matrix[p[0], j, abs(p[1])][1])
                if graph is not None:
                    if p[1] == 0 and graph[j, p[0], 0] == "o-o":
                        string += " | unoriented link"
                    if graph[p[0], j, abs(p[1])] == "x-x":
                        string += " | unclear orientation due to conflict"
        if ambiguous_triples is not None and len(ambiguous_triples) > 0:
            string += "\n## Ambiguous triples (not used for orientation):\n"
            for triple in ambiguous_triples:
                (i, tau), k, j = triple
                string += ("    [(%s % d), %s, %s]" % (self.vars_name[i], tau, self.vars_name[k], self.vars_name[j]))
        log.info(string)