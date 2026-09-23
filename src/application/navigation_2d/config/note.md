# Plugin params
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
ceres_preconditioner: SCHUR_JACOBI
ceres_trust_strategy: LEVENBERG_MARQUARDT
ceres_dogleg_type: TRADITIONAL_DOGLEG
ceres_loss_function: None

# ROS Parameters
odom_frame: odom
map_frame: map
base_frame: base_link
scan_topic: /scan
mode: mapping #localization

# if you'd like to immediately start continuing a map at a given pose
# or at the dock, but they are mutually exclusive, if pose is given
# will use pose
#map_file_name: test_steve
# map_start_pose: [0.0, 0.0, 0.0]
#map_start_at_dock: true

debug_logging: false
throttle_scans: 1
transform_publish_period: 0.02 #if 0 never publishes odometry
map_update_interval: 2.0  # 更新越快，影响算力。
resolution: 0.025   # 大了可能建图会更好，不是精度。
max_laser_range: 20.0 #for rastering images
minimum_time_interval: 0.5
transform_timeout: 0.2
tf_buffer_duration: 30.
stack_size_to_use: 40000000 #// program needs a larger stack size to serialize large maps
enable_interactive_mode: true

# General Parameters
use_scan_matching: true
use_scan_barycenter: true
minimum_travel_distance: 0.5
minimum_travel_heading: 0.2   # 改小提高laser的旋转敏感
scan_buffer_size: 10          # 不能有效匹配时改大，如：20    误匹配时改小
scan_buffer_maximum_scan_distance: 10
link_match_minimum_response_fine: 0.3 # 改大，要求必须搞好的匹配才行
link_scan_maximum_distance: 1.5
loop_search_maximum_distance: 3.0
do_loop_closing: true 
loop_match_minimum_chain_size: 5           # 改小，降低闭环的要求，更容易闭环，明显。改小的同时建议增大loop_match_..._fine=0.6,正确才闭环
loop_match_maximum_variance_coarse: 3.0  
loop_match_minimum_response_coarse: 0.15    # 降低闭环效果更好，占CPU
loop_match_minimum_response_fine: 0.3       # 降低闭环效果更好，占CPU

# Correlation Parameters - Correlation Parameters
correlation_search_space_dimension: 0.5
correlation_search_space_resolution: 0.01
correlation_search_space_smear_deviation: 0.1 

# Correlation Parameters - Loop Closure Parameters
loop_search_space_dimension: 8.0
loop_search_space_resolution: 0.05
loop_search_space_smear_deviation: 0.03

# Scan Matcher Parameters
distance_variance_penalty: 0.2  # 改小降低odom的权重   
angle_variance_penalty: 0.1     # 改小降低odom的权重

fine_search_angle_offset: 0.00349     
coarse_search_angle_offset: 0.349   
coarse_angle_resolution: 0.0349        
minimum_angle_penalty: 0.9
minimum_distance_penalty: 0.5
use_response_expansion: true




# base_frame: base_link
# resolution: 0.025

# loop_match_minimum_chain_size: 5 
    # loop_match_minimum_response_fine: 0.8
# angle_variance_penalty: 0.2 
# minimum_angle_penalty: 0.5
    # distance_variance_penalty: 0.3
    # minimum_distance_penalty: 0.3

# 特征多，速度慢，空间小的情况
# minimum_travel_distance: 0.25
# minimum_travel_heading: 0.25
# scan_buffer_size: 20

#  移动速度快，大空间的情况
# minimum_travel_distance: 0.5
# minimum_travel_heading: 0.5
# scan_buffer_size: 10

