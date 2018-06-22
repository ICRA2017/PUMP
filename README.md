# Parallel Uncertainty-aware Multiobjective Planning

[![Docker Build Status](https://img.shields.io/docker/build/icra2017/pump.svg)](https://hub.docker.com/r/icra2017/pump/)

This repo contains the code for "Real-Time Stochastic Kinodynamic Motion Planning via Multiobjective Search on GPUs," which presents the parallel uncertainty-aware multiobjective planning algorithm submitted to ICRA '17 for solving the stochastic kinodynamic motion planning problem.

It is written in CUDA C and setup to run a 3D double integrator (in a 6D configuration space). PUMP can be compiled with 
`$ make pump NUM=<sample count>` (e.g. `$ make pump NUM=4000`) 
and run with 
`$ ./pump <input file name> <cp target> <eta>` (e.g., `$ ./pump input.txt 0.01 2`). This runs out of the mainPUMP.cu main file. 

The input file contains all the necessary run settings for a planning problem. It is structured as a csv where each line respectively represents initial state, goal state, state space lower bound, state space upper bound, noise multiplier, number of obstacles, and the obstacles.
The obstacles are enumerated as bounding boxes in 6D with the lowest corner listed first and the highest corner listed second.

# Disclaimer

Though it can be run it is intended primarily for reference purposes and is currently a mess. Feel free to reach out to me if you're interested in the code and I may be able to provide a more up to date code, or at least some advice on what should be expanded on and what was written quickly just to get the job done. It may be best to reference this [disclaimer](https://github.com/schmrlng/MotionPlanning.jl), then add a bit more alpha. 

# License
Copyright 2017 Brian Ichter

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Run in Docker

```
$ nvidia-docker run -it --rm icra2017/pump
root@4904e4bdbdb6:/PUMP# ./pump input.txt 0.01 2
***** Reading in init, goal, obs took: 0.104 ms *****

--- Motion planning problem, input.txt ---
Sample count = 4000, C-space dim = 6, Workspace dim = 3
hi = [1 1 1 1 1 1 ], lo = [0 0 0 -1 -1 -1 ]
edge discretizations 8, dt = 0.05
the 3 obstacles are = 
0.05 0.2 -3.1 -3.1 -3.1 -3.1 0.45 0.35 3.1 3.1 3.1 3.1 
0.7 0.3 -3.1 -3.1 -3.1 -3.1 0.9 0.5 3.1 3.1 3.1 3.1 
0.3 0.6 -3.1 -3.1 -3.1 -3.1 0.8 0.75 3.1 3.1 3.1 3.1 
--- PUMP specific
numHSMCParticles = 128
Allowing 3 half-spaces (not yet implemented)
rnIdx is 799800
0.05th percentile pre calc took: 606.109 ms for 15996000 solves and cutoff is 48.7217 at 799800
Discretizing motions took: 1851.29 ms for 799800 solves
max number of nn is 1211
Sample free took: 0.000139 s
Edge validity check took: 0.0053 s (192, 4166)
setupAsBs (1024, 21089)
HS calculation took: 0.013651s for 21594600 computations (1024, 7030)
number of particles tracked 64763392
*********************** Beginning PUMP ***********************
CP target = 0.01, CP min soln = 0.005 CP cutoff = 0.02
************** starting iteration 1 with 1 paths
launching propagateValidParticles (512, 11) new paths = 43, prop time: 5.1e-05 s
launching calculateCP_HSMC (128, 1), calc time: 3.5e-05 s
...
goalCondition met
Explore took: 0.052902 s
number of goal paths found = 3 with a total number of paths = 2254 of 97237
*********************** output paths and check cp heursitic ********************
Path 1949: cost = 225.235, time = 1.73208, cp = 0.00146484 vs 0 est, nodes = [3999, 818, 2936, 2176, 1920, 0]
Path 2008: cost = 220.118, time = 1.8328, cp = 0.000976562 vs 0 est, nodes = [3999, 1853, 3506, 2176, 1920, 0]
Path 2085: cost = 211.045, time = 1.73437, cp = 0.000976562 vs 0 est, nodes = [3999, 2897, 3506, 2176, 1920, 0]
****************** Beginning search ******************
tried path 2085, cost = 211.045, cp = 0.000976562
SUCCESS
****************** Beginning smoothing ******************
 from the original nodes: 
0.1 0.1 0.1 0 0 0 
0.00732422 0.146319 0.16896 -0.360267 0.245817 0.441966 
0.00415039 0.629172 0.22016 0.812578 0.810805 -0.0896677 
0.303467 0.85932 0.244864 0.845184 0.629124 0.497567 
0.542236 0.924707 0.56608 0.732731 -0.105662 0.707853 
0.9 0.9 0.9 0 0 0 
smoothing gave a cost savings of 4.4591%
final smoothed path is: 
0.1 0.1 0.1 0 0 0 
0.019116 0.145166 0.165699 -0.274223 0.275416 0.453299 
0.0369561 0.60377 0.232849 0.850378 0.84877 0.0321577 
0.326816 0.830903 0.273671 0.886151 0.690213 0.570908 
0.562565 0.909416 0.584188 0.758678 -0.00163574 0.736117 
0.9 0.9 0.9 0.000589733 0.000589733 0.000589701 
****************** Final output ******************
Search took: 0.009076 s
Solution path = 2085 with cp = 0.00976562 and cost = 201.635
****************** To Excel results: ******************
cp: 			0.00976562
cost: 			201.635
Explore time: 		52.902 ms (34.368 ms in propagate cp)
SolnSearch time: 	9.076 ms
Free time: 		5.439 ms
HS time: 		13.651 ms
```
