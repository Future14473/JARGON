###Whats already there

- Add Curve impl made from multiple curves, similar to path
- finish `QuickCurve`
- Evaluate if `DoubleArray`/`DoubleList` (specialized) is really worth doing over
        `List\<Double\>`
- Make the 'D' in SOLID more apparent in Trajectory/MotionProfile
- Cleanup Util package


###Whats not yet there
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
- Goal is something kind of like (none of this is final):
```kotlin
fun configureTheThing(){
    val pathGraph = buildGraph(pathResolver, heading) {
        addPoint("A", 3, 4)
        addPoint("B", 32*inches,1.4*yards)
        addPoint("Finish")
        startPoint("A")
        val c = getPoint("B").withHeading(90*Degrees) thenTravel Pose2d(3*meters,234273843*nanometers,-TAU/4))
        addpoint("C",c)
        pathFrom("A").splineTo("B").apply{
            splineTo("C")
            lineTo("E") //lock derivatives
        }       
        fixAllPaths() //resolves derivatives
    }
    pathGraph.save("some/file.xml or json")
    ...
    val pathGraph = PathGraph.load("some/file.xml")
    val trajectoryGraph = pathGraph.calculateTrajectories(constraints)
    val drive: Drive = SomeMecanumOrWhateverDriveSystem(
        a=couple,
        of=params,
        that=are,
        intuitive=enough,
        but=not,
        too=many,
        startPoint = "A",
        graph=pathGraph
    )
}

fun doTheThing(){
    val driveTask = drive.goTo("B") //appends task to queue
    driveTask.addCallback { //at end
        someServoDoor.open()
        drive.thenGoTo("C")
    }
    driveTask.addTimeCallback(timeProgress=0.5){
        someMotor.turnOn()
    }
    ...
    drive.goTo("C") //after previous
    
    drive.goTo("E") //point E does not exist
    drive.goTo("F") //no path from C or wherever to F found
    drive.goFrom("C")
        .through("Place")
        .endAt("Destination")    //must call EndAt or nothing happens...
    ...
    log(drive.currentTask)
    log(drive.currentTask.progress)
    ...
    drive.cancel()
    drive.currentTask.await()
    drive.generatePath(...) {
        from = currentPosition
        from = endOflastTask
        to = point("e")
    }
}

```