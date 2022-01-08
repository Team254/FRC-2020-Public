# FRC 2020

Team 254's 2020 FRC robot code for [Dreadnought](https://www.team254.com/first/2020/). Dreadnought's code is written in Java and is based off WPILib's Java control system.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in that class' Java file.
## Setup Instructions

### General
1. Clone this repo
1. Run `./gradlew` to download Gradle and needed FRC/Vendor libraries (make sure you're using Java 11 or greater)
1. Run `./gradlew downloadAll` to download FRC tools (ShuffleBoard, etc.)
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension from the release page on [this repository](https://github.com/wpilibsuite/allwpilib/releases/latest)
2. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2020.ipr` file with IntelliJ
1. When prompted, select import Gradle build

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2020` folder then click finish

### Basic Gradle Commands
* Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
* Run `./gradlew build` to build the code.  Use the `--info` flag for more details
* Run `./gradlew test` to run all of the JUnit tests

## Code Highlights
- Path following with a RAMSETE nonlinear feedback controller and splines

  To control autonomous driving, a [nonlinear feedback controller](src/main/java/com/team254/frc2020/planners/DriveMotionPlanner.java#L225) is utilized to follow paths constructed of [quintic Hermite splines](src/main/java/com/team254/lib/spline/QuinticHermiteSpline.java).

- Image undistortion with OpenCV

  To correct lens distortion, [OpenCV is used](src/test/java/com/team254/frc2020/limelight/UndistortMapFileWriterTest.java) to map distorted or "raw" pixel coordinates to undistorted or "real" coordinates. These [maps](src/main/java/com/team254/frc2020/limelight/undistort/precomputedmaps) are precomputed for each individual Limelight and deployed with the rest of the code to reduce in-game computation time.

- Camera characterization with OpenCV

  To determine each camera's distortion constants, [this script](calibration/main.py) is used. Essentially, pictures of a checkerboard are taken at various distances and angles with each Limelight camera, then their cameras are characterized by the OpenCV script ([this guide](https://docs.opencv.org/4.5.2/dc/dbb/tutorial_py_calibration.html) goes into more detail). Derived constants are then copied into [`LimelightConstantsFactory`](src/main/java/com/team254/frc2020/limelight/constants/LimelightConstantsFactory.java) and used to generate each [`UndistortMap`](src/main/java/com/team254/frc2020/limelight/undistort/UndistortMap.java).

- Climbing with PTO drive gearbox

  The PTO climbing mechanism is controlled by a [state machine](src/main/java/com/team254/frc2020/statemachines/ClimbingStateMachine.java) which handles transitioning between climbing stages.

- Automatic Power Cell serializer

  To prevent jams when intaking, a [serializer](src/main/java/com/team254/frc2020/subsystems/Serializer.java) controlled by a state machine automatically sweeps Power Cells away from the intake area. It also feeds balls one by one up a [ramp](src/main/java/com/team254/frc2020/subsystems/Serializer.java#L65) into the shooter.


## Package Functions
- [`com.team254.frc2020`](src/main/java/com/team254/frc2020)

  Contains the robot's central functions and holds a class with all numerical constants used throughout the code. 

- [`com.team254.frc2020.auto`](src/main/java/com/team254/frc2020/auto)

  Handles the execution of autonomous routines and contains the [`actions`](src/main/java/com/team254/frc2020/auto/actions) and [`modes`](src/main/java/com/team254/frc2020/auto/modes) packages

- [`com.team254.frc2020.auto.actions`](src/main/java/com/team254/frc2020/auto/actions)
  
  Contains all actions used during the autonomous period, which all share a common interface, [`Action`](src/main/java/com/team254/frc2020/auto/actions/Action.java) (also in this package). Examples include driving paths, auto aiming the turret and scoring. Actions interact with the subsystems, which in turn interact with the hardware.

- [`com.team254.frc2020.auto.modes`](src/main/java/com/team254/frc2020/auto/modes)
  
  Contains all autonomous modes. Autonomous modes consist of a list of autonomous actions executed in a specific order.

- [`com.team254.frc2020.controlboard`](src/main/java/com/team254/frc2020/controlboard)
  
  Contains code for the driver to use either joysticks or gamepad and the operator to use a gamepad. Also contains a wrapper class specifically for Xbox controllers (see [XboxController.java](src/main/java/com/team254/frc2020/controlboard/XboxController.java)).

- [`com.team254.frc2020.limelight`](src/main/java/com/team254/frc2020/limelight)

  Handles interactions with Limelight.

- [`com.team254.frc2020.limelight.constants`](src/main/java/com/team254/frc2020/limelight/constants)
  
  Contains various constants for Limelights, including physical constants and pipeline information.

- [`com.team254.frc2020.limelight.undistort`](src/main/java/com/team254/frc2020/limelight/undistort)

  Contains code for undistorting images, including undistortion maps which implement the [`UndistortMaps`](src/main/java/com/team254/frc2020/limelight/undistort/UndistortMap.java) interface and essentially map "distorted" (raw) pixels to their "undistorted" equivalents. 

- [`com.team254.frc2020.loops`](src/main/java/com/team254/frc2020/loops)
  
  Contains codes for loops, which are routines that run periodically on the robot, such as for calculating robot pose, processing vision feedback, or updating subsystems. All loops implement the [`Loop`](src/main/java/com/team254/frc2020/loops/Loop.java) interface and are handled (started, stopped, added) by the [`Looper`](src/main/java/com/team254/frc2020/loops/Looper.java) class, which runs at 100 Hz. The [`Robot`](src/main/java/com/team254/frc2020/Robot.java) class has one main looper, `mEnabledLooper`, that runs all loops when the robot is enabled.

- [`com.team254.frc2020.paths`](src/main/java/com/team254/frc2020/paths)

  Contains the generator for all the trajectories that the robot can drive during the autonomous period.

- [`com.team254.frc2020.planners`](src/main/java/com/team254/frc2020/planners)
  
  Contains planners for various types of motion.

- [`com.team254.frc2020.statemachines`](src/main/java/com/team254/frc2020/statemachines)
  
  Contains the state machines for various subsystems.

- [`com.team254.frc2020.states`](src/main/java/com/team254/frc2020/states)
  
  Contains states and other classes used in various subsystem and state machine classes.

- [`com.team254.frc2020.subsystems`](src/main/java/com/team254/frc2020/subsystems)
  
  Contains code for subsystems, which are consolidated into one central class per subsystem, all of which extend the [`Subsystem`](src/main/java/com/team254/frc2020/subsystems/Subsystem.java) abstract class. Each subsystem uses state machines for control and is a singleton, meaning that there is only one instance of each. Subsystems also contain an enabled loop, a read periodic inputs method, and a write periodic outputs method, which are controlled by the [`SubystemManager`](src/main/java/com/team254/frc2020/SubsystemManager.java) class.

- [`com.team254.lib.drivers`](src/main/java/com/team254/lib/drivers)
  
  Contains a set of custom classes for motor controllers (TalonSRX's and Talon FX's) that include classes for creating motor controllers with factory default settings, handling errors, reducing CAN Bus usage, and checking motors.

- [`com.team254.lib.geometry`](src/main/java/com/team254/lib/geometry)
  
  Contains a set of classes that represent various geometric entities.

- [`com.team254.lib.motion`](src/main/java/com/team254/lib/motion)
  
  Contains all motion profiling code used for autonomous driving. Trapezoidal motion profiles are used for smooth acceleration and minimal slip.

- [`com.team254.lib.physics`](src/main/java/com/team254/lib/physics)
  
  Contains classes that model physical systems and their low level behavior (as opposed to high level subsystems).

- [`com.team254.lib.spline`](src/main/java/com/team254/lib/spline)
  
  Contains code for generating and optimizing splines.

- [`com.team254.lib.trajectory`](src/main/java/com/team254/lib/trajectory)

  Contains code for following and storing trajectories.

- [`com.team254.lib.util`](src/main/java/com/team254/lib/util)
  
  Contains a collection of assorted utilities classes used in the robot code. Check each file for more information.

- [`com.team254.lib.vision`](src/main/java/com/team254/lib/vision)
  
  Contains various classes that help with tracking and storing information about vision targets.

- [`com.team254.lib.wpilib`](src/main/java/com/team254/lib/wpilib)
  
  Contains parent classes of the main [`Robot`](src/main/java/com/team254/frc2020/Robot.java) class that get rid of loop overrun and watchdog print messages that clutter the console.

## Style Conventions
- k*** (i.e. `kDriveWheelTrackWidthInches`): Final constants, especially those found in the [`Constants.java`](src/main/java/com/team254/frc2020/Constants.java) file
- m*** (i.e. `mPathFollower`): Private instance variables
