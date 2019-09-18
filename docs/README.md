## Physically Consistent GMM and DS Learning

### Abstract
<div style="text-align: justify">
We propose a physically-consistent Bayesian non-parametric approach for fitting Gaussian Mixture Models (GMM) to trajectory data. Physical-consistency of the GMM is ensured by imposing a prior on the component assignments biased by a novel similarity metric that leverages locality and directionality. The resulting GMM is then used to learn globally asymptotically stable Dynamical Systems (DS) via a Linear Parameter Varying (LPV) re-formulation. The proposed DS learning scheme accurately encodes challenging nonlinear motions automatically. Finally, a data-efficient incremental learning approach is introduced that encodes a DS from batches of trajectories, while preserving global stability. Our contributions are validated on 2D datasets and a variety of tasks that involve single-target complex motions with a KUKA LWR 4+ robot arm.
</div>

<div style="text-align: justify">
This new DS formulation and learning approach has been further used to learn navigation tasks for a semi-autonomous wheelchair and for a humanoid robot to navigate and co-manipulate and object as shown in the videos below.
</div>

### Video of Approach and Robot Experiments
- Robot Manipulation Experiments
<p align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/HfV4jbJBWTQ" frameborder="0" allow="autoplay; encrypted-media" allowfullscreen></iframe>
</p>

- Wheelchair Navigation Simulation
<p align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/r5EjMoMuOrs" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>

- iCub Navigation and Co-Manipulation Simulation
<p align="center">
<iframe width="560" height="315" src="https://www.youtube.com/embed/3z52S-u1qaI" frameborder="0" allow="accelerometer; autoplay; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</p>

### Code
Following we list all of the code repositories made available for this project, including:
- Code for Learning GMM and DS in MATLAB:  
[https://github.com/nbfigueroa/phys-gmm](https://github.com/nbfigueroa/phys-gmm)   
[https://github.com/nbfigueroa/ds-opt](https://github.com/nbfigueroa/ds-opt)

- Code for Executing Learned DS models in C++:  
[https://github.com/nbfigueroa/lpvDS-lib](https://github.com/nbfigueroa/lpvDS-lib)

- Code for Executing Learned DS models in ROS:  
[https://github.com/epfl-lasa/ds_motion_generator](https://github.com/epfl-lasa/ds_motion_generator)

- Code for Executing Manipulation Tasks on a KUKA-LWR 4+/Gazebo (in ROS):  
[https://github.com/epfl-lasa/kuka-lpvds-tasks](https://github.com/epfl-lasa/kuka-lpvds-tasks)

- Code for Executing Navigation Tasks on Gazebo (ROS) simulation of Quickie Salsa-M Wheelchair:  
[https://github.com/epfl-lasa/wheelchair-ds-motion](https://github.com/epfl-lasa/wheelchair-ds-motion)

- Code for Recording Kinesthetic Demonstrations in ROS (arm states, sensors and gripper states):  
[https://github.com/nbfigueroa/easy-kinesthetic-recording](https://github.com/nbfigueroa/easy-kinesthetic-recording)

### References
> [1] Figueroa, N. and Billard, A. (2018) "A Physically-Consistent Bayesian Non-Parametric Mixture Model for Dynamical System Learning". In Proceedings of the 2nd Conference on Robot Learning (CoRL). [[pdf]](http://proceedings.mlr.press/v87/figueroa18a/figueroa18a.pdf) 

### Contact 
[Nadia Figueroa](http://lasa.epfl.ch/people/member.php?SCIPER=238387) (nadia.figueroafernandez AT epfl dot ch)

### Acknowledgments
This work was supported by the EU project [Cogimon](https://cogimon.eu/cognitive-interaction-motion-cogimon) H2020-ICT-23-2014.

