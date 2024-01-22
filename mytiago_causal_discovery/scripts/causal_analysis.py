#!/usr/bin/env python3
import json
from darko_causal_discovery_msgs.msg import CausalModel
from causal_model import causal_model
import rospy
import utils
from constants import *
import threading
import time
import pandas as pd
import message_filters


df_data = pd.DataFrame()
df_reset = True


def publish_model(model_json, h_id):
    """
    Publish model_json

    Args:
        model_json (JSON): JSON string to publish
    """
    
    # Building message
    causal_disc = CausalModel()
    causal_disc.model = model_json
    causal_disc.human_id = h_id
    
    # Publish message
    pub_causal_model.publish(causal_disc)
    log.info(utils.print_causal_model_msg(causal_disc))

    
def t_multicausal(csv_id):
    """
    Causal analysis on file named csv_id

    Args:
        csv_id (str): csv ID to analyse
    """
    log.info("Causal analysis on file " + utils.get_csv_path(csv_id) + " started")
    
    # Run causal analysis
    cm = causal_model(csv_id, vars_name, ALPHA)
    cm.run_causal_discovery_algorithm()
    log.info("Causal analysis on file " + utils.get_csv_path(csv_id) + " completed")

    # Publish causal model in JSON format
    publish_model(json.dumps(cm.inference_dict), cm.human_id)
    
    # Delete file .csv just analysed
    utils.delete_csv(csv_id)
    del cm
    
    
def t_fifocausal():
    """
    Causal analysis on files contained in folder data_pool with FIFO strategy
    """
    while True:
        list_datacsv = utils.file_in_folder(DATA_DIR)
        if len(list_datacsv) > 0:
            list_datacsv.sort()
            t_multicausal(list_datacsv[0])
        time.sleep(1)


def cb_handle_data(humans_pose, objs_pose):
    """
    Callback to handle new data on human trajectory

    Args:
        humans_pose (Humans): custom msg from T2.5
        objs_pose (SceneObjects): custom msg from MapServer
    """
    global df_data
    global df_reset

    
    # If TS_LENGTH reached then reset dataframes else append new data
    df_reset_diffdown = False
    if df_reset:
        df_data = pd.DataFrame(columns = vars_name)
        df_reset = False
        df_reset_diffdown = True
        
    if utils.selected_human_exist(humans_pose, selected_id=HUM_ID) and utils.selected_obj_exist(objs_pose, selected_id=OBJ_ID):
        h_state = utils.handle_human_pose(humans_pose, selected_id=HUM_ID)
        goal = utils.handle_obj_pose(objs_pose, selected_id=OBJ_ID)

        # Compute and append new data to the timeseries
        new_data = utils.compute_causal_var(h_state, goal)
        if df_reset_diffdown:
            new_data['human_id'] = HUM_ID
            df_reset_diffdown = False
        df_data = df_data.append(pd.Series(new_data), ignore_index=True)
        
    else:
        # append None data to the timeseries
        new_data = {'theta_g' : None, 'd_g' : None, 'v' : None}
        if df_reset_diffdown:
            new_data['human_id'] = HUM_ID
            df_reset_diffdown = False
        df_data = df_data.append(pd.Series(new_data), ignore_index=True)
        
    # Starting causal analysis if TS_LENGTH has been reached
    if (len(df_data) * 1/NODE_RATE) >= TS_LENGTH:
        # Saving dataframe into .csv file
        csv_id = utils.save_csv(df_data)
        
        if CAUSAL_STRATEGY == causal_stategy.MULTI:
            # Starting causal analysis on .csv just created
            t_causality = threading.Thread(target = t_multicausal, args = (csv_id,))
            t_causality.start()
        
        # New dataframe
        df_reset = True
        

if __name__ == '__main__':

    # Create data pool directory
    utils.create_data_dir()
    if CAUSAL_STRATEGY == causal_stategy.FIFO:
        # Starting thread checking for files .csv in data_pool folder
        t_causality = threading.Thread(target = t_fifocausal)
        t_causality.start()
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    
    # Reading variables name
    vars_name, vars_name_printable = utils.read_vars_name(VARS_FILENAME)

    rate = rospy.Rate(NODE_RATE)

    # Init subscriber & publisher
    sub_humans_pose = message_filters.Subscriber('/perception/humans', Humans)
    sub_objs_pose = message_filters.Subscriber('/mapping/scene_objects', SceneObjects)
    pub_causal_model = rospy.Publisher('/hri/causal_discovery', CausalModel, queue_size=10)

    # Init synchronizer and assigning a callback 
    ats = message_filters.ApproximateTimeSynchronizer([sub_humans_pose, sub_objs_pose], 
                                                      queue_size=NODE_RATE*TS_LENGTH, 
                                                      slop=1, 
                                                      allow_headerless=True)
    ats.registerCallback(cb_handle_data)

    while not rospy.is_shutdown():
        rate.sleep()
