# JARGON
JARGON stands For **J**ARGON: **A** **R**obot **G**uidance **O**peratio**N**,
a robotics library for creating control systems, primarily targeted towards FTC.


## THIS PROJECT IS CURRENTLY DISCONTINUED

The main developer of this project is leaving, the project is not complete to be easily usable. Also, there are already multiple other open-source similar libraries and repositories easily accessible with more people involved, and are better tested by users. Acknowledging the limitations of having one overly ambitious developer, this  library simply cannot at the current state keep up to be competitive. However, for the curious, there are a few strategies and implementations that may still be worth looking at, and can provide some inspiration; this is released under an MIT licence, so you are free to copy whatever you wish. 

Instead, here are some other libraries you can check out:
    
- [FTCLib](https://github.com/FTCLib/FTCLib): a general purpose FTC library containing lots of helpful utilizes, control system parts, a path and trajectory system, etc.
- [Roadrunner](https://github.com/acmerobotics/road-runner): A library for more advanced paths and trajectories.
- [RevExtensions2](https://github.com/OpenFTC/RevExtensions2): Provides additional features interacting with your REV hub that may be useful for making better control systems.
- [FutureResource](https://github.com/Future14473/FutureResource): Some introductions/tutorials on various beyond-basics concepts in robotics. (This is where is focused instead).


To play around,
```groovy
dependencies {
    // Core: Robot modeling, motion profiles, geometry and math
    implementation 'org.futurerobotics.jargon:jargon-core:0.2.0'
    // path system: Paths, curves, trajectories, constraints
    implementation 'org.futurerobotics.jargon:jargon-pathing:0.2.0'
    // experimental basic state-space control system utilities
    implementation 'org.futurerobotics.jargon:jargon-state-space:0.2.0'
}
```


#### Existing features include:
- Path planning/profiling
   - 2d geometry framework
   - Flexible path and trajectory generation system
- Control systems framework
   - Customizable and extendable motion profile generation system
   - Utilities for math, linear algebra, geometry, physics
   - Basic controller elements like PID, feed-forward, etc.
- Model based control systems
- (Experimental) state-space control system
 
 
## Other info
### Principles of design:
- Gentle learning curve -- provide both basic and advanced versions of components.
- Keep everything well documented.
- Extensible and customizable
- Adherence to good programming principles
- Abstraction over performance without neglecting performance

### JARGON is inspired by:
- Pathing system inspired by [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner).
- State-space is inspired by [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Pathing and trajectories adapted from concepts from this paper: <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
