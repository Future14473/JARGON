# WELCOME TO JARGON

JARGON stands For **J**ARGON: **A** **R**obot **G**uidance **O**peratio**N**,
a robotics library for creating control systems with an easy to use and extendable framework.

This is specifically targeted towards middle to high school level robotics users, such as in FTC, with a focus on ease of use and deepness of comprehension.

We hope that this will help inspire people to delve into more advanced mechanisms for control to squeeze as much functionality they can out of a few sensors and actuators. We encourage people to experiment with new ideas.

Version 0.1.0 has been released. Being the very first version, it may be buggy. We suggest waiting for
version 0.2.0.
To play around,
```groovy
dependencies {
    // Core libraries: Robot modeling, motion profiles
    implementation 'org.futurerobotics.jargon:jargon-core:0.1.1'
    // pathing system: Paths, curves, graphs
    implementation 'org.futurerobotics.jargon:jargon-pathing:0.1.1'
    // experimental state-space system framework
    implementation 'org.futurerobotics.jargon:jargon-state-space:0.1.1'
    // Extensions to integrate kotlin coroutines
    implementation 'org.futurerobotics.jargon:jargon-coroutine-integration:0.1.1'
}
```


#### Existing features includes:
- Path planning/profiling system
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
   - Inspired by some people at FRC. Basically, lots of math for lots of profit.
 #### Planned features include:
 - Module to integrate FTC SDK directly
 - Swerve drive support
 - Tutorial-like documentation
 - Declarative-programming based control system design using kotlin dsl
 
#### Interested in contributing or giving feedback? See the [Contributing guidelines](CONTRIBUTING.md).

## JARGON is now open source
JARGON will now be accepting feedback, discussion, and contributions. Feel free to experiment and test with anything and see the [Contributing guidelines](CONTRIBUTING.md).
 
## Other info
### Principles of design:
- Gentle learning curve -- provide both basic and advanced versions of components.
- Keep everything well documented.
- Create with ease of use in mind
- Extensible and customizable. Easy to create your own implementations if you wish.
- Adherence to good programming principles
- Abstraction over performance without neglecting performance

### JARGON is inspired by:
- Pathing system inspired by [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner).
- State-space is implemented nearly directly from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Pathing and trajectories adapted from concepts from this paper: <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
