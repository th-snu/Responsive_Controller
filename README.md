# dmm
* Training :
python3 ppo.py --ref=bvh_name.bvh --test_name=test_name (--pretrain=output/test_name/network-0) (--nslaves=16)
* Render :
./motion_ctr --bvh ***.bvh --ppo ***/network-0

Don't use run_cmake.sh install.sh yet
