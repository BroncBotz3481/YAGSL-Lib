./gradlew.bat :spotlessApply
./gradlew.bat publish
xcopy /y build/repos/releases/swervelib yagsl/repos/
