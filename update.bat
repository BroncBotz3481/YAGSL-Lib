./gradlew.bat :spotlessApply
./gradlew.bat publish
copy /y build/repos/releases/swervelib yagsl/repos/
