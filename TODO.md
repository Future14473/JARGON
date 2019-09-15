Simulations
-
- Model error, measurement/actuation error
- The actual flippin simulations


Control systems
-
- All the state space stuff
    - system components
    - LQR
    - Kalman filter
    - Automated Augmentation, maybe
- Better control chain links
    - Global to relative wrapper, with GlobalPoseTracker
    - StateSpace 
    - Wrap pass
- Other localization support
    - Gryos
    - Odometry wheels
    - Camera or manual injection

Pathing
-
- Model based constraints
    - Max motor velocity, acceleration <--> torque, voltage, current?
- PathGraph
    - kotlin DSL, java builder
- Dynamic Path/Trajectory generation
- State insertion (for example with CV)

Systems
-
- Concurrent stuff
    - Simple managed threads system thing
    - Master/slave integrating into ^ with commands
    - Better executors and/or kotlin coroutines (sorry java peeps)
    - Integrate control systems within above
- Tests
