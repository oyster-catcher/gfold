
Solve the soft-landing problem (a simplified version of G-FOLD) and auto pilot craft to soft-land with kRPC interfaced to Kerbal Space Program.
Requires cvxopt and kRPC to be installed.
http://cvxopt.org and http://krpc.github.io
Some description of this code and videos is available at http://seriousplaywithksp.weebly.com

The main program to run is softland.sh, which can be edited to switch between several targets, and if no target latitude or longitude is given it will use the current target, for a drone ship landing for instance. The code works best with small craft within few km of the landing site maximum, otherwise high speed causes problem since aerodynamic forces are not accounted for. There are a whole range of parameters to tweak given as command line arguments to softland.py, which can control the daringness of the descent by specifying maximum angles to thrust at, land at, and how late to slow down. Once the trajectory is computed for soft-landing (if possible) it will be drawn graphically.

softland.py calls classes within runprograms.py based in the current runmode which is switched between in the main program. The run programs used in softland.py are simply SoftLanding, which computed a trajectory from the current position to make a soft-landing at the target (almost), and FinalLanding which controls the very final descent to the surface vertical as, this needs to tight feedback lock on the throttle/speed/radar altitude.

Hope you have fun.

Note that you must install kRPC and cvxopt to run this code.

p.s. I'm still working on boostback.py which will compute the whole descent steering aerodynamically from orbit, and using the soft-landing code towards the landing. This did work, but needs a bit more tweaking.

Adrian S
