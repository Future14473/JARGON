##Hi!

This is a robotics library (originally for FTC, theoretically beyond that)
 I've been working on for the past month or so (and will continue to work on).
 This is still a work in progress, but I have now made it public.
 
Planned features include:

-   Robotics motion planning, with paths, trajectories, graphs, with lots of room for extensibility and customization to 
    be able to apply to everyone and complex-ish systems.
    -   Planned experiment: trajectories using only constraints on WHEEL velocity and acceleration, for example
-   Better control for motors and servos and buttons etc
-   Coordinating hardware into Systems so you can say "extend arm" instead of "set target position 3000",
    and arms never collide into each other
-   Easy (enough) to use simple specialized concurrency frameworks for robotics
-   OpenCV things

Principles of design:
-   User friendly
-   Extensible and customizable, no requirement to adhere to one implementation
-   Strict adherence to SOLID

This library is inspired by the following:
- [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner)
- A little bit from [calcmogul's state-space-guide](https://github.com/calcmogul/state-space-guide)
- I enjoy programming things that aren't textbook questions

This is still a work in progress, and I plan to eventually make this publicly available and supported.
I plan to have working prototypes (or better) before the "real" start of next FTC season.
You can feel free to experiment with whats currently there (not much usable right now except test as of 7/30/2019).

PFAQ (predicted frequently asked questions):
-
#####1. If there already exists several robotics libraries and things on the internet publicly available now. Why do you want to go through the trouble of making your own version that might not be as great?

A few reasons:
   - This is a personal project as much as it is a team/public project. I like designing things my own way and seeing 
        how they turn out and learning from the experience, and there's not quite the same satisfaction from stealing
        someone else's code off the internet.
   - I try to do something new and learn things from scratch rather than imitate whats already there. 
    Therefore, my version is subjectively better.
   - Friendly competition: I obviously want this library to be better than everyone else's...
   - Bragging rights
   
#####2. There's quite a few similarities to [ACME robotics's roadrunner](https://github.com/acmerobotics/road-runner)...
I borrowed several concepts/names from roadrunner, but I did not directly copy code. Copying code is no fun. 
I implemented everything from scratch, using my own guidline on how it should world.

Where I did do some light "Copying", I make myself understand first and then implement it from scratch, with no copying,
and I try to improve what I see.
 
It's under a MIT licence anyways...

#####3. Can I use this in my own project/robot?
Yes, but keep in mind before the first "release" anything is subject change. Feedback is appreciated (open an issue)!
Does my documentation suck? Tell me at least