## Hi!

This is a very work in progress extra-pre-alpha robotics library that is originally for FTC but could 
theoretically be used beyond that.

This is currently public so that other people know what I am working on and can provide earlier feedback.
Nothing in here is in any way final (yet).

_This may possibly be de-branded from Future robotics in the future._ No pun intended.
 
Planned features include:
- Robotics motion planning, with paths, trajectories, graphs, with lots of room for extensibility and customization
    -   Planned experiment: trajectories using only constraints on WHEEL velocity/acceleration/torque/voltage
- Throw the full power of linear algebra and control theory at yourself to create control systems, or do something
  simpler
- Coordinating hardware into Systems so you can say "extend arm" instead of "set target position 3000 degrees", and other
  relations
- Convenience things for better control for motors and servos and buttons etc
- Easy (enough) to use concurrency frameworks for robotics
- OpenCV pipelines and frameworks

Principles of design:
-   User friendly
-   Extensible and customizable, no requirement to adhere to one implementation; create your own
-   Adherence to SOLID

This project is inspired by the following:
- Class structure inspired by [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner)
- The decision to include control-theory stuff comes from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Motion profile/trajectory generation adapted from <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>

I plan to have working prototypes (or better) before the "real" start of next FTC season.

See TODO.md (maybe to be migrated into github issues) for a list of current todo items

PFAQ (predicted frequently asked questions):
-
##### 1. If there already exists several robotics libraries and things on the internet publicly available now. Why do you want to go through the trouble of making your own version that might not be as great?

A few reasons:
   - This is a personal project as much as it is a team/public project. I like designing things my own way and seeing 
        how they turn out and learning from the experience, and its not the same experience as stealing
        someone else's code off the internet.
   - I try to invent new things and learn things from scratch rather than imitate whats already there. 
    Therefore, my version is subjectively better.
   - Many of the libraries do not have features that I want and/or don't quite work the way I want
   - One of the goals is to also try to clarify some of the complexity in these systems to make it more understandable,
    so to inspire more interest in some of these subjects to others
   - Friendly competition and bragging rights
   
##### 2. There are a few similarities to [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner)...
I borrowed several concepts/names from roadrunner, but I did not copy code. Copying code is no fun. 
I implemented everything from scratch, trying to use my own judgment on how things should be structured

Where I did do some light "Copying", I make myself understand first and then implement it from scratch, 
and I try to improve on works
 
It's under a MIT licence anyways...

##### 3. Can I use this in my own projects and endeavours?
Yes! Please do. 
But keep in mind before the first "release" anything is subject change.
Feedback is appreciated (open an issue).

##### 4. Will this be open-source/available for contribution?
Possibly, after things have settled down and the design/structure is mostly fixed so collaboration does not become a 
    nightmare.