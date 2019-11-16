# WELCOME TO JARGON

JARGON stands For **J**ust **A**nother **R**obot **G**uidance **O**peratio**N**,
a robotics library for creating advanced control systems in FTC.

We hope that this will help inspire people to delve into more advanced mechanisms for control to squeeze as much
functionality they can out of a few sensors and actuators. We encourage people to experiment with new ideas and
promote discussion and learning with the community.

Interested in contributing or giving feedback? See the [Contributing guidelines](CONTRIBUTING.md).

#### Planned and existing features includes:
- Advanced pathing system; dynamic trajectory generation
   - Once setup, "go to the opposite side of the field".
   - Customizable and extendable motion profile generation system
- Literal block-diagram based systems
- Model based control systems
   - Minimal PID fiddling
   - Drive trains and more
- (Experimental) state-space based control systems
   - Inspired by some people at FRC

## JARGON is now open source
We now wish to share our knowledge with the world, free of charge. Although a officially supported version is not
released yet, we are now accepting feedback, discussion, and contributions. Feel free to experiment and test with 
anything involved. See CONTRIBUTING.md for more details.

## Other info
### Principles of design:
- Intuitive, as well as advanced. Usable both by beginners and experienced alike
- Extensible and customizable. Create your own implementations.
- Adherence to good programming principles
- Abstraction over performance without neglecting performance (another 10 ns is worth it, but not 3s)

### JARGON is inspired by:
- Pathing system inspired by [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner). We extend
  on those features.
- State space guide comes from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Pathing and trajectory system adapted from this paper <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>
