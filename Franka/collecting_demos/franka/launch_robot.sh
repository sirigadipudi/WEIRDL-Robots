source ~/anaconda3/etc/profile.d/conda.sh
conda activate polymetis-local
echo robot | sudo -S pkill -9 run_server
launch_robot.py robot_client=franka_hardware robot_client.executable_cfg.robot_ip=172.16.0.8

