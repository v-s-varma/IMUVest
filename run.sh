pkill notochord

(cd ../notochord/bin ; ./notochord --scan --raw --no_bundles --odr=100 -y localhost)&

sleep 3

python3 IMUVest.py
