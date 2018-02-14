#!/bin/bash

# land at Rocket-pad
#python ./softland.py --target_lat -0.0972066410402 --target_lng -74.557698656 --target_alt 74 --logfile vessel.dat --softlandlogfile traj.dat

# land on H-Pad
#python ./softland.py --target_lat -0.0968071692165 --target_lng -74.6172808614 --target_alt 180 --logfile vessel.dat --softlandlogfile traj.dat

# land on tower
lat=`echo -0.0951066410402+0.0030|bc -l`
lng=`echo -74.557698656+0.0053|bc -l`
python ./softland.py --target_lat $lat --target_lng $lng --target_alt 100 --logfile vessel.dat --softlandlogfile traj.dat
