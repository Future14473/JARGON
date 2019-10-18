# WELCOME TO JARGON

JARGON stands For **J**ust **A**nother **R**obot **G**uidance **O**peratio**N**
###### (A.K.A cha cha real smooth)

This is a (currently) work in progress robotics library that is intended for creating semi-advanced
control systems in FTC.

VERSION 0.1.0 COMING SOON

This library will be extensively used by Future Robotics, Team 14473, in the upcoming 2019-2020 SkyStone competitions.

_This may possibly be de-branded from Future robotics in the future._ No pun intended.
 
#### Current beta features include:
- Robotics motion planning (paths, motion profiles, trajectories) with model-based and extensible constraints (make your own)
- Advanced and customizable control systems, based off of MODELs using the blocks system: inspired by taking block diagrams literally.
- Use of State Space Stuff
- Simple simulations

#### Planned features include:
- High level easy to understand abstractions of said above
- More advanced simulations
- Concurrency framework/program management situation
- Easy integration with CV stuff
- Dynamic trajectory generation
- High level TrajectoryManager class to streamline the pathing process
- Support for:
  - Other types of odometry/control schemes
  - Swerve drive
- Parameter estimation
- Tutorial/documentation
- Short papers explaining all the math used
- Visual interface (but that's like GUI stuff which takes time)


#### Principles of design:
- Intuitive at the same time as advanced
- Extensible and customizable (no requirement to adhere to one implementation, create your own)
- Adherence to SOLID; Decoupled code
- Abstraction over performance (Sometimes another 10 ns is worth it), without neglecting performance (Another 3s is not worth it)

This project is inspired by the following:
- Originally designed to be a better version of [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner).
  It has now evolved into a different creature entirely.
- The decision to include control theory/state space stuff comes from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- Motion profile for trajectories generation algorithm adapted from <http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf>

If you are interested in the current development, see TODO.md (maybe to be migrated into github issues) for a list of
 current todo items


PFAQ (predicted frequently asked questions):
-
##### 1. There already exists several robotics libraries and things on the internet publicly available now. Why do you want to go through the trouble of making your own version that might not be as great?

A few reasons:
   - I try to invent new things and learn things from scratch rather than imitate whats already there. 
    Therefore, my version is subjectively better.
   - This library contains a lot more subjectively useful features than all the other applicable ones I see on the
     internet
   - One of the goals is to also try to clarify some of the complexity in these systems to make it more understandable,
    so to inspire more interest in some of these subjects to others
   - Friendly competition and bragging rights
   
##### 2. There are a few similarities to [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner)...
This project was originally inspired by road-runner, and that can be seen in some parts of this library. However, I
believe it has now evolved into a entirely different creature.

##### 3. Can I use this in my own projects, endeavours, and plans to take over the world?
Yes! Please do. All we ask is that you give attribution and credit where credit is due.

But keep in mind that until version 0.1.0 anything is subject to change at any time.
Feedback is appreciated (open an issue).

##### 4. Will this be open-source/available for contribution?
Possibly, after I have finally decided to stop doing bad git practices, and we publish this with a
licence and contribution info.

#### UNTIL THEN

This is currently developed by only one person (Benjamin Ye) who wishes for companions. If you feel you have the guts
and the will to contribute to the development of this library before we open it up on purpose, contact Future Robotics.

'Guts' meaning _some_ of the following:
- Know Kotlin (a language very much worth learning)
- Understand what SOLID is and what it could look like in practice (look it up)
- Know some calculus and/or physics
- Know linear algebra and/or state-space theory (go read the state-space guide linked above somewhere)
- Willing to help proofread or document
- Willing to at least attempt good git practices
