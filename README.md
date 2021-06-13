# QuadrupedKinematics
C/C++ library for accurate control of quadruped kinematics, including foot positioning and stable gaits. Currently being developed, but simple inverse kinematics 
is already supported! 

For math explanations/development progress, see my blog: https://seanboe.github.io/blog/tag/quadruped

Current notes
(This is for me to keep track of stuff while I develop... a true wiki/guide will come when this library is fully operational.)

`DYNAMIC` vs `STATIC` angle positions
- Static positions are the final angles of the robot. If the foot needs to move 200mm forwards, then STATIC holds the angles needed to achieve that.
- Dynamic positions are created from interpolated coordinate values up to and including the final coordinate position. It goes through many coordinate
  positions __along the path__ to getting to the final destination

Benefits/consequences:
- Static positioning is fast. It involves one calculation and no need for more. Dynamic uses static positioning but commands it across different endpoints.
- Dynamic is slow. Not only does it involve many calculations, but it also is based on an interpolation object which must be slow (slower than static) since
  there is a set time to interpolate across. 

Thus, 
1. Use static angle positioning for leg gaits because it is fast, and the rapid update rate needed for successful leg gaits undermines the need for dynamic positioning anyway.
2. Use dynamic angle positioning for low update/command applications i.e. static robot movement (yaw, roll, etc.) or immediate endpoint setting (move the foot 100mm forwards to do something)