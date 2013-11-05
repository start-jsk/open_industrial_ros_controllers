初期設定
========
必要なパッケージの導入
roswww (https://github.com/jihoonl/roswww)
tf2_web_republisher, interactive_marker_proxy, rosbridge_server (apt-get等で)

makeの実行


動かし方
========
Sim環境
"""
roslaunch vs060_moveit_config demo_simulation.launch
roslaunch open_industrial_web_interface sim_demo.launch
"""
実機
"""
roslaunch vs060_moveit_config demo.launch
roslaunch open_industrial_web_interface demo.launch
"""

http://(HOSTNAME):8000/open_industrial_web_interface/でアクセスできる。
