General
-
- Proofread, document, and defensive everything
- @JvmOverloads, @JvmStatic
- equals, hashCode, and toString
- Double check SOLID, DRY, YAGNI (about the last one...)
- typealiases or subclassing to consolidate types? (sorry java peeps)
- Motion and MotionState interfaces/implementations

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
- Weird reference tracker thing: Generics or not?

Pathing
-
- Model based constraints
    - Max motor velocity, acceleration <--> torque, voltage, current?
- PathGraph
    - kotlin DSL
- Dynamic Path/Trajectory generation
- State insertion (for example with open CV)

Systems
-
- Concurrent stuff
    - Simple managed threads system thing
    - Master/slave integrating into ^ with commands
    - Better executors and/or kotlin coroutines (sorry java peeps)
    - Integrate control systems within above
