# QuadrupedKinematics
C/C++ library for accurate control of quadruped kinematics, including foot positioning and stable gaits. Currently being developed, but simple inverse kinematics 
is already supported! 

For math explanations/development progress, see my blog: https://seanboe.github.io/blog/tag/quadruped

Current notes

Some things to be aware of:

Motor numbering system:
  Each leg is numbered like the unit circle when the robot is facing forwards. 
  For motors, motor 1 (M1) = shoulder x-axis, motor 2 (M2) = shoulder z-axis, motor 3 (M3) = elbow y-axis
  Limb numbering corresponds to which motor is moving it. Ex: limb 2 = controlled by M2 = bicep limb  
