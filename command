scp -r .\firefighter-wheels firebot@firebot.local:~/
cd ~/firefighter-wheels && bash install_pi.sh && source .venv/bin/activate && firebot doctor && firebot mission --sim
