
roslaunch roscausal_data data_collection.launch ts_length:=90 dt:=0.1 subsampling:=true data_dir:=/root/shared/data_pool pp_data_dir:=/root/shared/pp_data_pool pp_script:="postprocess.py"

roslaunch roscausal_discovery causal_discovery.launch data_dir:=/root/shared/pp_data_pool res_dir:=/root/shared/cm_pool


