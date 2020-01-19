# WELCOME TO JARGON

JARGON stands For **J**ARGON: **A** **R**obot **G**uidance **O**peratio**N**,
a robotics library for creating more advanced control systems with a easy to use and extendable framework.

We hope that this will help inspire people to delve into more advanced mechanisms for control; to squeeze as much
functionality they can out of a few sensors and actuators. We encourage people to experiment with new ideas, and
dare to take steps into the more complex.

Version 0.1.0 has been released!!!
To play around,
```groovy
dependencies {
    // Core libraries: Robot modeling, motion profiles
    implementation 'org.futurerobotics.jargon:jargon-core:0.1.0'
    // pathing system: Paths, curves, graphs
    implementation 'org.futurerobotics.jargon:jargon-pathing:0.1.0'
    // experimental state-space system framework
    implementation 'org.futurerobotics.jargon:jargon-state-space:0.1. 0'
    // Extensions to integrate kotlin coroutines
    implementation 'org.futurerobotics.jargon:jargon-coroutine-integration:0.1.0'
}
```

TODO, soon: more instructions on how to use. For now you can read documentation.


#### Existing features includes:
- Advanced pathing system
   - Dynamic trajectory generation
   - Path graphing system: once setup, all you need to say is "go to point B"
   - 2d geometry framework
- Flexible control systems framework
   - Customizable and extendable motion profile generation system
   - FrequencyRegulator to control the speed of control loops intelligently
   - Utilities for math, linear algebra, geometry, physics
   - Basic controller elements like PID, feed-forward mechanisms, 2d localizers, etc.
- Model based control systems
   - Model your drive systems instead of guessing for more accurate control
- (Experimental) state-space based control systems
   - Inspired by some people at FRC. Basically, lots of math to model and create control systems, but it's worth it.
   - Basic framework providing basic functions and calculations, as well as an (experimental) runner class
 #### Planned features include:
 - Module to integrate FTC SDK
 - Swerve drive support
 - Declarative-programming based control system design
    - If you're interested in where this came from, go to the git history and see the "blocks" module that
     has now been removed  
 
 
#### Interested in contributing or giving feedback? See the [Contributing guidelines](CONTRIBUTING.md).

## JARGON is now open source
We now wish to share our knowledge with the world, free of charge. JARGON will now be accepting feedback, discussion,
 and contributions. Feel free to experiment and test with anything and see the 
 [Contributing guidelines](CONTRIBUTING.md) for more details on how to contribute.
 
## Other info
### Principles of design:
- Intuitive, as well as advanced. Usable both by beginners and advanced users alike.
- Keep everything well documented.
- Create with ease of use in mind -- "get to the point"
- Extensible and customizable. Easy to create your own implementations if you wish.
- Adherence to good programming principles, like SOLID
- Abstraction over performance without neglecting performance (another 10 ns is worth it, but not 3s)

### JARGON is inspired by:
- Pathing system inspired by [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner).
- State-space is implemented nearly directly from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Pathing and trajectory system adapted from this paper: <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
