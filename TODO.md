Simulations
-
- Model error, measurement/actuation error
- The actual flippin simulations


Control
-
- Automated model augmentation, maybe
- Better control
    - StateSpace 
- Other localization support
    - Gryos
    - Odometry wheels
    - Camera or manual injection

Pathing
-
- Model based constraints
    - Max motor velocity, acceleration <--> torque, voltage, current?
- PathGraph
    - kotlin DSL, java config
- Dynamic Path/Trajectory generation

Systems
-
- Concurrent stuff
    - Simple managed threads system thing
    - Master/slave integrating into ^ with commands
    - Better executors and/or kotlin coroutines (sorry java peeps)
    - Integrate control systems within above
- Tests
