
# FORMAL TODO
### Add
- Preconditions DSL

### Change
- PathSegment is now Path, and the old Path is now ComponentPath, similarly for Curve
- Introduce super-interfaces of Curve and Path: MinimalCurve and MinimalPath, respectively, used in profile generation
    for interface segregation
- ReIntroduce "steppers": guided iterators, used for example to step along a curve at specific points without binary
 searching every time
- MotionProfiles generified using MotionProfiled interface
- Cleanup testing

# OTHER TODO NOTES
Whats already there
-

- Add Curve impl made from multiple curves, similar to path
- finish `QuickCurve`
- Evaluate if `DoubleArray`/`DoubleList` (specialized) is really worth doing over
        `List\<Double\>`
- Make the 'D' in SOLID more apparent in Trajectory/MotionProfile
- Cleanup Util package
- Generalize profiles
- Consider 

Whats not yet there
- `PathGraph`/`PathSkeleton`/`GraphBuilder`/`TrajectoryMaker`/`DerivativeResolver` or whatever its gonna be called
- Putting trajectories to use
- "*High school level control theory*"
- Simulations
- Hardware bridge
- Concurrency framework based on slave/master/actor-ish things
- Proofread, test, document, consistency, polish
- End-to-end tests in the physical world
- Maven Publish??
- Drive system
- Control for other things too...
- Wrap it all in a user-friendly-ish interface
- (??) Convert this to Github issues
- Write a short doc/paper with derivations/explanations of the yet unexplained math
- Finalize software structure
