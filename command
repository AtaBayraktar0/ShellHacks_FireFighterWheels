scp -r .\firefighter-wheels firebot@firebot.local:~/
cd ~/firefighter-wheels && bash install_pi.sh && source .venv/bin/activate && firebot doctor && firebot mission --sim
cd ~/firefighter-wheels
bash install_pi.sh --camera
ERROR: Could not find a version that satisfies the requirement pyrealsense2<3,>=2.54 (from versions: none)
ERROR: No matching distribution found for pyrealsense2<3,>=2.54
cd ~/firefighter-wheels
BUILD_JOBS=1 bash build_realsense.sh

bash install_pi.sh --camera
source .venv/bin/activate
firebot doctor --hardware --port /dev/ttyACM0
