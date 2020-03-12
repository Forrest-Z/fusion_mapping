# Fusion Mapping

## Prerequesties
* PCL 1.7
* ROS M
* GLOG
* OPENCV3
* Eigen3.3.1

## Run
### Map save
> rosservice call /save_map


## Evaluate
### Trajectory Evaluate
> pip install evo --upgrade --no-binary evo  
> 评价段段离离内的误差：  
> evo_rpe ground_truth.txt laser_odom.txt -r trans_part --delta 100 --plot_mode xyz  
> 价价总累计误差：  
> evo_rpe ground_truth.txt laser_odom.txt -r full --delta 100 --plot_mode xyz 
