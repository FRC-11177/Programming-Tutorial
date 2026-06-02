# Repository Guidelines

## Project Structure & Module Organization
This is a WPILib GradleRIO Java robot project. Primary code lives in `src/main/java/frc/robot/`, with the entry point in `Main.java`, robot lifecycle logic in `Robot.java`, and subsystem wiring in `RobotContainer.java`. Subsystem code is grouped by feature, such as `src/main/java/frc/robot/Drivetrain/`. Static deploy files belong in `src/main/deploy/`. Vendor dependencies are tracked in `vendordeps/`, and WPILib/Gradle wrapper files should remain checked in.

## Build, Test, and Development Commands
Use the Gradle wrapper from the repository root:

- `./gradlew build` - compiles the robot code and runs tests.
- `./gradlew test` - runs the JUnit 5 test suite only.
- `./gradlew deploy` - builds and deploys to the configured RoboRIO.
- `./gradlew simulateJava` - runs the robot code in desktop simulation when supported.

On Windows, use `gradlew.bat` instead of `./gradlew`.

## Coding Style & Naming Conventions
Use Java 17, 4-space indentation, and the existing WPILib command-based style. Package names are lowercase (`frc.robot`); class names use `PascalCase`; fields and methods use `camelCase`. Keep subsystem-specific constants in a dedicated `Constants.java` or similar class under the subsystem package. Favor small, direct methods and avoid introducing extra abstraction unless it simplifies robot behavior.

## Testing Guidelines
Tests use JUnit 5 through Gradle. Place unit tests under `src/test/java/` and name them `*Test.java` so Gradle picks them up automatically. Prioritize tests around control logic, state transitions, and math-heavy code; hardware-facing code should be isolated behind interfaces or wrappers where practical.

## Commit & Pull Request Guidelines
The repository currently has only an initial setup commit, so there is no established history pattern yet. Use short, imperative commit messages such as `Add drivetrain constants` or `Fix autonomous scheduling`. Pull requests should include a clear summary, notes on verification (`./gradlew build`, simulation, or robot test), and screenshots or logs when behavior changes are visible.

## Configuration Notes
Team-specific and vendor settings are stored in `.wpilib/` and `vendordeps/`. Update vendor JSON files through the usual WPILib workflow rather than editing them manually unless you know the schema. Keep deployable assets in `src/main/deploy/` so they are packaged correctly.
