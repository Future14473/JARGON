# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project (will eventually) adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Until a version that makes sense, everything will be version [Early#].
At that point, a new changelog will be made and this will be renamed to old_changelog.

## [Unreleased]
### Changed
- MotionState is now State; and an interface; ValueMotion and ValueState are implementations.
- Replace `Stopwatch` with `LoopRegulator`
- Ditch Path is-a curve and create `asCurve` extension function instead.
- Improved docs everywhere
- Renamed a few things
### Fixed
- A lot of random things

## [Early3] 9/14/2019
### Changed
- Path planning related are refactored into their own module. Control will be the emphasis of the core module. (not final)
- Updated readme
- Begin proper git habits
- Improve TODO.md
### Added
- Control system frameworks. This is an entire subject in it of itself; read everything in the 'control' package
- Mechanics package for models and common physics calculations

## [Early2] 8/21/2019
### Added
- `Stepper`s for "stepping" over paths and trajectories, similar to iterators
- `ReverseCurve`/`ReversePath`
- `MultipleCurve/Path`
- `PointTurn` Path
- `PointConstraint` interface: return of `MotionProfileConstraint`
- `MotionProfiled` interface, in preparation for anything motion profiled control
- [Koma](http://koma.kyonifer.com/) kotlin linear algebra library dependency

### Changed
- Change versioning since it doesn't make sense, and every change will likely be breaking
until some point in the future when things settle down. Since this is mostly practice in
trying to keep good version control.
- Change package structure
- Curve is now a typealias of GenericCurve, Path is a extension of GenericCurve. Only solution
for implement same interface different generics I can find.
- PathSegment is now Path, Path is now MultiplePath
- Realized that the JIT compiler is a rather good optimizer and removed some confusing local optimizations
- AccelConstraint no longer takes `reversed` as a parameter...
- Refactor ProfileConstraint interface to use the introduced PointConstraint, make ProfileConstraint a stepper.
- Path and Curve have had most of their methods removed to be minimal.
- CurvePoint now includes length info
- HeadingProviders now only take CurvePoint (only use for original Curve is length)
- Random fixes/improvements/small changes
- Some renaming of functions

### Removed
- Currently useless `QuickCurve`


## [Early1] - 8/14/2019
### Added
IN A REALLY TIGHT NUTSHELL:
- Make public
- Skeletal structure for control/hardware interfaces/classes
- Paths, reparameterization, motion profiles, trajectories, and generation
- Unit tests, inspection tests, and graph-generation tests
- Motion profile graphs
