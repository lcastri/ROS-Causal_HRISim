#!/usr/bin/env python

from enum import Enum
import glob
import json
import os
from tigramite.independence_tests.gpdc import GPDC
from fpcmci.CPrinter import CPLevel
from fpcmci.FPCMCI import FPCMCI
from fpcmci.preprocessing.data import Data
from fpcmci.selection_methods.TE import TE, TEestimator
from fpcmci.basics.constants import LabelType
import rospy
import pandas as pd
from mytiago_causal_discovery.msg import CausalModel
from std_msgs.msg import Header


class CausalDiscoveryMethod(Enum):
    PCMCI = "pcmci"
    FPCMCI = "fpcmci"


NODE_NAME = "mytiago_causal_discovery"
NODE_RATE = 1 # [Hz]
CDM = str(rospy.get_param("/mytiago_causal_discovery/cd_method", default = "fpcmci"))
FALPHA = float(rospy.get_param("/mytiago_causal_discovery/filter_alpha", default = 0.05))
ALPHA = float(rospy.get_param("/mytiago_causal_discovery/sig_alpha", default = 0.05))
MINLAG = int(rospy.get_param("/mytiago_causal_discovery/min_lag", default = 1))
MAXLAG = int(rospy.get_param("/mytiago_causal_discovery/max_lag", default = 1))
DATA_DIR = str(rospy.get_param("/mytiago_causal_discovery/data_dir", default = '/root/shared/')) + '/data_pool'
RES_DIR = str(rospy.get_param("/mytiago_causal_discovery/res_dir", default = '/root/shared/')) + '/cm_pool'

class CausalDiscovery():
    def __init__(self, df: pd.DataFrame, dfname) -> None:
        self.df = df
        self.dfname = dfname
        
    def run(self):
        df = Data(self.df)
        cdm = FPCMCI(df, 
                     f_alpha = FALPHA,
                     alpha = ALPHA,
                     min_lag = MINLAG, 
                     max_lag = MAXLAG, 
                     sel_method = TE(TEestimator.Gaussian), 
                     val_condtest = GPDC(significance = 'analytic', gp_params = None),
                     verbosity = CPLevel.NONE,
                     neglect_only_autodep = True,
                     resfolder = RES_DIR + '/' + self.dfname)
        
        if CDM == CausalDiscoveryMethod.FPCMCI.value:
            cdm.run()
        elif CDM == CausalDiscoveryMethod.PCMCI.value:
            cdm.run_pcmci()
            
        cdm.dag(label_type = LabelType.Lag)
        cdm.timeseries_dag()
        
        return cdm.CM.g
        
        
def find_csv(file_pattern='data_*.csv'):
    file_list = glob.glob(os.path.join(DATA_DIR, file_pattern))

    min_id = float('inf')
    min_id_file = None
    min_file_name = None

    for file_path in file_list:
        # Extract ID from the file name
        file_name = os.path.basename(file_path)
        data_id = int(file_name.split('_')[1].split('.')[0])
            
        # Update min ID and file if the current ID is smaller
        if data_id < min_id:
            min_id = data_id
            min_id_file = file_path
            min_file_name = file_name.split('.')[0]

    return min_id_file, min_file_name
                
        
if __name__ == '__main__':
    # Create res pool directory
    os.makedirs(RES_DIR, exist_ok=True)
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)

    # Publisher
    pub_causal_model = rospy.Publisher('/hri/causal_model', CausalModel, queue_size=10)
    
    while not rospy.is_shutdown():
        
        csv, name = find_csv()
        if csv is not None:
            dc = CausalDiscovery(pd.read_csv(csv), name)
            cm = dc.run()
            
            msg = CausalModel()
            msg.header = Header()
            msg.causal_model = json.dumps(cm)
            pub_causal_model.publish(cm)
                
        rate.sleep()