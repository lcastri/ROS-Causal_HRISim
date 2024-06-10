from copy import deepcopy
import json
import os
from pathlib import Path
import pandas as pd
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType, ImageExt
from time import time
from datetime import timedelta
from fpcmci.preprocessing.subsampling_methods.Static import Static

def save_result(fpcmci_g_time = None, fpcmci_g_cm = None, 
                fpcmci_k_time = None, fpcmci_k_cm = None, 
                pcmci_time = None, pcmci_cm = None):
    res_tmp['gt'] = str(GT)
    
    if fpcmci_g_cm is not None:
        res_tmp['fpcmci_g']['time'] = fpcmci_g_time
        res_tmp['fpcmci_g']['shd'] = shd(GT, get_correct_SCM(GT, fpcmci_g_cm.get_SCM()))
        res_tmp['fpcmci_g']['scm'] = str(get_correct_SCM(GT, fpcmci_g_cm.get_SCM()))
    
    if fpcmci_k_cm is not None:
        res_tmp['fpcmci_k']['time'] = fpcmci_k_time
        res_tmp['fpcmci_k']['shd'] = shd(GT,get_correct_SCM(GT, fpcmci_k_cm.get_SCM()))
        res_tmp['fpcmci_k']['scm'] = str(get_correct_SCM(GT, fpcmci_k_cm.get_SCM()))
    
    if pcmci_cm is not None:
        res_tmp['pcmci']['time'] = pcmci_time
        res_tmp['pcmci']['shd'] = shd(GT, get_correct_SCM(GT, pcmci_cm.get_SCM()))
        res_tmp['pcmci']['scm'] = str(get_correct_SCM(GT, pcmci_cm.get_SCM()))
    
    
def get_correct_SCM(gt, scm):
    new_scm = {v: list() for v in gt.keys()}
    if list(scm.keys()):
        for k in scm:
            new_scm[k] = scm[k]
    return new_scm


def get_FP(gt, cm):
        """
        False positive number:
        edge present in the causal model 
        but absent in the groundtruth

        Args:
            cm (dict): estimated SCM

        Returns:
            int: false positive
        """
        counter = 0
        for node in cm.keys():
            for edge in cm[node]:
                if edge not in gt[node]: counter += 1
        return counter


def get_FN(gt, cm):
    """
    False negative number:
    edge present in the groundtruth 
    but absent in the causal model
    
    Args:
        cm (dict): estimated SCM
        
    Returns:
        int: false negative
    """
    counter = 0
    for node in gt.keys():
        for edge in gt[node]:
            if edge not in cm[node]: counter += 1
    return counter
    
    
def shd(gt, cm):
    """
    Computes Structural Hamming Distance between ground-truth causal graph and the estimated one
    
    Args:
        cm (dict): estimated SCM
        
    Returns:
        int: shd
    """
    fn = get_FN(gt, cm)
    fp = get_FP(gt, cm)
    return fn + fp


GT = {
      "v": [('v', -1), ('d_g', -1), ('r', -1)],
      "d_g": [('d_g', -1), ('v', -1)],
      "r": [('r', -1), ('v', -1)]}


ALGO_RES = {
            'time' : None,
            'shd' : None,
            'scm' : None,
           }


EMPTY_RES = {
             'gt' : None,
             'fpcmci_g' : deepcopy(ALGO_RES),   
             'fpcmci_k' : deepcopy(ALGO_RES),   
             'pcmci' : deepcopy(ALGO_RES),
            }

DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/Traj_pp'


if __name__ == '__main__':   
    resdir = "ROMAN2024/Evaluation_Hz"
    f_alpha = 0.5
    alpha = 0.05
    min_lag = 1
    max_lag = 1
    agents = ["A" + str(i) for i in range(1, 16)]
    perc_samples = [20, 10, 5, 3]
    
    for p in perc_samples:
        for a in agents:
            resfolder = 'results/' + resdir + '/' + str(p) + '/' + str(a)
            os.makedirs(resfolder, exist_ok = True)
            res_tmp = deepcopy(EMPTY_RES)
            
            csv = pd.read_csv(DATA_DIR + '/' + a + '.csv')
            d_obs = Data(csv, vars = ["v", "d_g", "r"], subsampling = Static(p))
                    
            #########################################################################################################################
            # # FPCMCI Gaussian
            # fpcmci_g = FPCMCI(deepcopy(d_obs),
            #                 f_alpha = f_alpha, 
            #                 pcmci_alpha = alpha, 
            #                 min_lag = min_lag, 
            #                 max_lag = max_lag, 
            #                 sel_method = TE(TEestimator.Gaussian), 
            #                 val_condtest = GPDC(significance = 'analytic'),
            #                 verbosity = CPLevel.INFO,
            #                 neglect_only_autodep = False,
            #                 resfolder = resfolder + "/fpcmci_g")

            # new_start = time()
            # _, fpcmci_g_cm = fpcmci_g.run()
            # elapsed_fpcmci_g = time() - new_start
            # fpcmci_g_time = str(timedelta(seconds = elapsed_fpcmci_g))
            # fpcmci_g.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PNG, node_proximity=3)
            # fpcmci_g.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PDF, node_proximity=3)
            
            # #########################################################################################################################
            # # FPCMCI Kraskov
            # fpcmci_k = FPCMCI(deepcopy(d_obs),
            #                 f_alpha = f_alpha, 
            #                 pcmci_alpha = alpha, 
            #                 min_lag = min_lag, 
            #                 max_lag = max_lag, 
            #                 sel_method = TE(TEestimator.OpenCLKraskovCMI), 
            #                 val_condtest = GPDC(significance = 'analytic'),
            #                 verbosity = CPLevel.INFO,
            #                 neglect_only_autodep = False,
            #                 resfolder = resfolder + "/fpcmci_k")

            # new_start = time()
            # _, fpcmci_k_cm = fpcmci_k.run()
            # elapsed_fpcmci_k = time() - new_start
            # fpcmci_k_time = str(timedelta(seconds = elapsed_fpcmci_k))
            # fpcmci_k.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PNG, node_proximity=3)
            # fpcmci_k.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PDF, node_proximity=3)
                    
            #########################################################################################################################
            # PCMCI
            pcmci = FPCMCI(deepcopy(d_obs),
                           pcmci_alpha = alpha, 
                           min_lag = min_lag, 
                           max_lag = max_lag, 
                           sel_method = TE(TEestimator.GaussianTE), 
                           val_condtest = GPDC(significance = 'analytic'),
                           verbosity = CPLevel.INFO,
                           neglect_only_autodep = False,
                           resfolder = resfolder + "/pcmci")
                    
            new_start = time()
            _, pcmci_cm = pcmci.run_pcmci()
            elapsed_pcmci = time() - new_start
            pcmci_time = str(timedelta(seconds = elapsed_pcmci))
            print(pcmci_time)
            pcmci.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PNG, node_proximity=3)
            pcmci.timeseries_dag(node_size=5, font_size=14, img_ext = ImageExt.PDF, node_proximity=3)
            
            
            #########################################################################################################################
            # SAVE
            save_result(fpcmci_k_time = fpcmci_k_time, 
                        fpcmci_k_cm = fpcmci_k_cm, 
                        pcmci_time = pcmci_time, pcmci_cm = pcmci_cm)
            
            Path(os.getcwd() + "/results/" + resdir).mkdir(parents=True, exist_ok=True)
            filename = os.getcwd() + "/results/" + resdir + "/" + str(p) + ".json"
            
            # Check if the file exists
            if os.path.exists(filename):
                # File exists, load its contents into a dictionary
                with open(filename, 'r') as file:
                    data = json.load(file)
            else:
                # File does not exist, create a new dictionary
                data = {}

            # Modify the dictionary
            # data[a] = res_tmp
            data[a]['fpcmci_g'] = res_tmp['fpcmci_g']
            data[a]['pcmci'] = res_tmp['pcmci']

            # Save the dictionary back to a JSON file
            with open(filename, 'w') as file:
                json.dump(data, file)
            res_tmp.clear()