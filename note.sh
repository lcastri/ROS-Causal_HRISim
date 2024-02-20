
roslaunch roscausal_data data_collection.launch ts_length:=120 dt:=0.3 subsampling:=true data_dir:=/root/shared/data_pool pp_data_dir:=/root/shared/pp_data_pool pp_script:=""
roslaunch roscausal_data data_collection.launch dt:=0.3 subsampling:=true data_dir:=/root/shared/data_pool
roslaunch roscausal_data data_collection.launch ts_length:=10 dt:=0.3 subsampling:=true data_dir:=/root/shared/data_pool

roslaunch roscausal_discovery causal_discovery.launch data_dir:=/root/shared/pp_data_pool res_dir:=/root/shared/cm_pool