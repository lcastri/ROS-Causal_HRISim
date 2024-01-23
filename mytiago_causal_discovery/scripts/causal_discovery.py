#!/usr/bin/env python

from copy import deepcopy
from datetime import datetime
from enum import Enum
import glob
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
from sklearn.utils._testing import ignore_warnings
from sklearn.exceptions import ConvergenceWarning
import stat


class CausalDiscoveryMethod(Enum):
    PCMCI = "pcmci"
    FPCMCI = "fpcmci"


NODE_NAME = "mytiago_causal_discovery"
NODE_RATE = 10 # [Hz]
CDM = str(rospy.get_param("/mytiago_causal_discovery/cd_method", default = "fpcmci"))
FALPHA = float(rospy.get_param("/mytiago_causal_discovery/filter_alpha", default = 0.05))
ALPHA = float(rospy.get_param("/mytiago_causal_discovery/sig_alpha", default = 0.05))
MINLAG = int(rospy.get_param("/mytiago_causal_discovery/min_lag", default = 1))
MAXLAG = int(rospy.get_param("/mytiago_causal_discovery/max_lag", default = 1))
DATA_DIR = str(rospy.get_param("/mytiago_causal_discovery/data_dir", default = '/root/shared/')) + 'data_pool'
RES_DIR = str(rospy.get_param("/mytiago_causal_discovery/res_dir", default = '/root/shared/')) + 'cm_pool'
ID_FORMAT = str(rospy.get_param("/mytiago_causal_discovery/id_format", default = '%Y%m%d_%H%M%S'))
CSV_PREFIX = str(rospy.get_param("/mytiago_causal_discovery/cas_prefix", default = 'data_'))

class CausalDiscovery():
    def __init__(self, df: pd.DataFrame, dfname) -> None:
        self.df = df
        self.dfname = dfname
        
    @ignore_warnings(category=ConvergenceWarning)
    def run(self):
        df = Data(self.df)
        f_list = deepcopy(df.features)
        if "time" in f_list: f_list.remove("time")
        df.shrink(f_list)
        cdm = FPCMCI(df, 
                     f_alpha = FALPHA,
                     pcmci_alpha = ALPHA,
                     min_lag = MINLAG, 
                     max_lag = MAXLAG, 
                     sel_method = TE(TEestimator.Gaussian), 
                     val_condtest = GPDC(significance = 'analytic', gp_params = None),
                     verbosity = CPLevel.NONE,
                     neglect_only_autodep = True,
                     resfolder = RES_DIR + '/' + self.dfname)
        
        if CDM == CausalDiscoveryMethod.FPCMCI.value:
            feature, causalmodel = cdm.run()
        elif CDM == CausalDiscoveryMethod.PCMCI.value:
            feature, causalmodel = cdm.run_pcmci()
        
        if len(feature) > 0:   
            cdm.dag(label_type = LabelType.Lag, node_layout='circular')
            cdm.timeseries_dag()
        
        return feature, causalmodel
        

def extract_timestamp_from_filename(file_name, file_prefix=CSV_PREFIX, file_extension='.csv'):
    # Extract the timestamp from the file name
    file = os.path.basename(file_name)
    start_index = len(file_prefix)
    end_index = file.find(file_extension)
    timestamp_str = file[start_index:end_index]

    # Convert the timestamp string to a datetime object
    return datetime.strptime(timestamp_str, ID_FORMAT)


def get_file(file_prefix=CSV_PREFIX, file_extension='.csv'):
    # Construct the file pattern based on the prefix and extension
    file_pattern = os.path.join(DATA_DIR, f'{file_prefix}*{file_extension}')

    # List files in the directory matching the pattern
    files = glob.glob(file_pattern)

    if not files:
        return None, None  # No files found

    # Get the oldest file based on the extracted timestamp
    oldest_file = min(files, key=lambda file: extract_timestamp_from_filename(file))
    filename = os.path.basename(oldest_file)
    return oldest_file, filename[:filename.find(file_extension)]
                
                
if __name__ == '__main__':
    # Create res pool directory
    os.makedirs(RES_DIR, exist_ok=True)
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)

    # Publisher
    pub_causal_model = rospy.Publisher('/hri/causal_model', CausalModel, queue_size=10)
    
    rospy.logwarn("Waiting for a csv file...")
    while not rospy.is_shutdown():
        
        csv, name = get_file()
        if csv is not None:
            rospy.logwarn("Processing file: " + name)
            dc = CausalDiscovery(pd.read_csv(csv), name)
            f, cm = dc.run()
            
            if len(f) > 0:
                msg = CausalModel()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                
                cs = cm.get_skeleton()
                val = cm.get_val_matrix()
                pval = cm.get_pval_matrix()
                rospy.logwarn("CAUSAL STRUCTURE: " + str(cs))
                rospy.logwarn("VAL MATRIX: " + str(val))
                rospy.logwarn("PVAL MATRIX: " + str(pval))
                msg.features = f
                msg.causal_structure.data = cs.flatten().tolist()
                msg.val_matrix.data = val.flatten().tolist()
                msg.pval_matrix.data = pval.flatten().tolist()
                msg.original_shape = list(cs.shape)
                pub_causal_model.publish(msg)
                
            rospy.logwarn("Removing file: " + name)
            os.chmod(csv, stat.S_IWUSR | stat.S_IRUSR | stat.S_IXUSR)
            os.remove(csv)

                
        rate.sleep()