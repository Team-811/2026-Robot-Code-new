# Deep Analysis: WPILib 2025→2026 & Java 17→21 Upgrade Issues

**Date**: February 10, 2026  
**Analysis Scope**: Team 811 FRC 2026 Robot Code  
**Current Status**: Build successful; all tests passing  
**Target**: Identify compatibility issues and portability concerns

---

## Executive Summary

The upgrade from WPILib 2025 to 2026 and Java 17 to Java 21 was successful at the build level. However, this deep analysis identified **8 significant issues** ranging from behavioral changes to deprecated API usage and runtime warnings. Most require attention before competition deployment.

---

## 🔴 CRITICAL ISSUES

### 1. **URL Creation Behavior Change in LimelightHelpers** (HIGH SEVERITY)
**File**: `src/main/java/frc/robot/LimelightHelpers.java` (Line 918)  
**Issue**: OpenRewrite transformation changed exception handling behavior

```java
// BEFORE (Java 17):
url = new URL(urlString);

// AFTER (Java 21 - current):
url = URI.create(urlString).toURL();
```

**Problem**:
- `new URL(urlString)` performs **strict validation** and throws `MalformedURLException` immediately for invalid URLs
- `URI.create(urlString).toURL()` uses **lenient parsing** and may accept malformed URLs that `URL` constructor would reject
- `URI.create()` doesn't throw exceptions for invalid characters/structures—validation only happens at `.toURL()` conversion
- **This could allow invalid Limelight hostnames to silently pass validation**

**Impact**:
- Limelight snapshot capture requests may fail silently with null URLs
- Runtime errors instead of initialization errors
- Vision subsystem may not properly handle malformed Limelight hostnames

**Recommendation**: 
```java
// Revert to explicit URL validation
public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    try {
        // Explicit strict validation
        return new URL(urlString);
    } catch (MalformedURLException e) {
        System.err.println("Invalid Limelight URL: " + urlString);
        throw new RuntimeException("Limelight URL error: " + urlString, e);
    }
}
```

---

### 2. **Java 21 Annotation Processor Warning** (MEDIUM SEVERITY)
**Output**: Gradle build shows: `Supported source version 'RELEASE_17' from annotation processor 'org.gradle.api.internal.tasks.compile.processing.TimeTrackingProcessor' less than -source '21'`

**Problem**:
- Gradle's internal annotation processor doesn't support Java 21
- This is a **Gradle toolchain mismatch** issue
- Could lead to incomplete annotation processing

**Status**: Non-blocking (warning only)

**Recommendation**:
Update `build.gradle` to explicitly configure annotation processors for Java 21 compatibility:
```gradle
tasks.withType(JavaCompile) {
    options.compilerArgs.add '-XDstringConcat=inline'
    options.release = 21
    options.encoding = 'UTF-8'
}
```

---

## 🟠 MAJOR ISSUES

### 3. **Deprecated Limelight APIs in Use** (MEDIUM SEVERITY)
**File**: `src/main/java/frc/robot/LimelightHelpers.java` (Lines 1103-1125)

Three deprecated methods are still marked with `@Deprecated`:
```java
@Deprecated
public static double[] getBotpose(String limelightName)

@Deprecated
public static double[] getBotpose_wpiRed(String limelightName)

@Deprecated  
public static double[] getBotpose_wpiBlue(String limelightName)
```

**Problem**:
- These methods are **deprecated in LimelightHelpers v1.11**
- Code appears to be using newer methods (`getBotPose_*`) correctly
- **Risk**: If any commands still reference deprecated methods, they will break in future updates

**Search Results**: Code audit shows these deprecated methods are **not directly called** in commands/subsystems

**Recommendation**: 
Perform a search in all command classes to ensure no calls to `getBotpose*()` exist:
```bash
grep -r "getBotpose[^_]" src/main/java --include="*.java"
grep -r "getBotpose_wpi" src/main/java --include="*.java"
```

---

### 4. **System.out/System.err Usage in Production Code** (MEDIUM SEVERITY)
**File**: `src/main/java/frc/robot/LimelightHelpers.java`  
**Instances**: 20+ `System.out.println()` and `System.out.printf()` calls (Lines 830-858)

**Problem**:
- **FRC best practice**: Use WPILib logger (`DataLogManager`/`SmartDashboard`) instead of System streams
- Console output is not captured in FRC log files
- Performance: Multiple `System.out.printf()` calls in `printPoseEstimate()` method
- Competition robots should use structured logging

**Examples**:
```java
System.out.println("No PoseEstimate available.");  // Line 830
System.out.printf("Pose Estimate Information:%n");  // Line 834
System.err.println("bad LL URL");  // Line 921
```

**Recommendation**:
Replace with DataLogManager:
```java
// Instead of System.out.println()
import edu.wpi.first.wpilibj.DataLogManager;

DataLogManager.log("No PoseEstimate available");
DataLogManager.log("Pose Estimate - Timestamp: " + pose.timestampSeconds);
```

---

### 5. **Unused Imports & Code Cleanup** (LOW-MEDIUM SEVERITY)
**File**: `src/main/java/frc/robot/RobotContainer.java` (Line 30)

Outdated file path comment:
```java
//This file is loacted C:\Users\Team 811\FRC\2025-Robot-Code-new\2025-Robot-Code-new\src\main\java\frc\robot
```

**Problem**:
- Stale path reference (incorrect spelling "loacted")
- Hardcoded Windows paths in source code
- Not portable across team member machines

**Recommendation**: Remove this line entirely.

---

### 6. **CommandXboxController Binding Pattern** (MEDIUM SEVERITY - Future Compatibility)
**File**: `src/main/java/frc/robot/RobotContainer.java` (Lines 96-105)

**Observation**: Current bindings structure is correct, but verify command cancellation behavior:

**Potential Issue**:
- WPILib 2026 may have changes to how trigger-based commands are cancelled
- Current code uses `.whileTrue()` and `.onTrue()` which should still work
- **Verify**: Test with actual Xbox controller that speed mode toggle works correctly

**Recommendation**:
Test the following scenarios:
1. Hold right bumper (slow mode) - should toggle
2. Release and press again - should toggle back
3. Drive commands should respond to speed mode changes in real-time

---

### 7. **PathPlanner Error Handling** (MEDIUM SEVERITY)
**File**: `src/main/java/frc/robot/RobotContainer.java` (Lines 124-134)

```java
try {
    chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    chooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
    chooser.addOption("midL4x1 (if present)", new PathPlannerAuto("midL4x1"));
} catch (Exception ex) {
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    SmartDashboard.putString("Mode/autoChooser/Error", "PathPlanner chooser failed: " + ex.getMessage());
}
```

**Problem**:
- **Overly broad exception handling** (`catch (Exception ex)`)
- Doesn't distinguish between missing files vs. actual errors
- Could mask real problems during development
- `PathPlannerAuto("midL4x1")` may fail silently if path doesn't exist

**Recommendation**:
```java
try {
    chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    chooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
    // Only add optional auto if path exists
    try {
        new PathPlannerAuto("midL4x1");  // Test creation first
        chooser.addOption("midL4x1", new PathPlannerAuto("midL4x1"));
    } catch (Exception ignored) {
        // Path doesn't exist, skip this option
    }
} catch (RuntimeException ex) {
    DataLogManager.log("ERROR: PathPlanner initialization failed: " + ex);
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
}
```

---

## 🟡 WARNINGS & OBSERVATIONS

### 8. **Notifier Usage in Simulation** (LOW SEVERITY - Informational)
**File**: `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` (Lines 40, 288, 296)

```java
private Notifier m_simNotifier = null;
...
m_simNotifier = new Notifier(() -> { ... });
m_simNotifier.startPeriodic(kSimLoopPeriod);
```

**Status**: ✅ Correct usage for WPILib 2026

The `Notifier` class remains unchanged in WPILib 2026 and is correctly used for simulation loop timing.

---

### 9. **NetworkTables Struct Publishers** (INFORMATIONAL)
**File**: `src/main/java/frc/robot/Telemetry.java` (Lines 52-58)

```java
private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
```

**Status**: ✅ Correct WPILib 2026 API

The Struct API for NetworkTables is properly implemented and fully compatible with Java 21.

---

## 🟢 RECOMMENDATIONS SUMMARY

### Immediate Actions (Before Competition):

1. **FIX**: Revert URL creation in `LimelightHelpers.getLimelightURLString()` to use strict validation
   - **Priority**: HIGH
   - **Time**: 10 minutes
   - **Risk**: Vision subsystem reliability

2. **REFACTOR**: Replace all `System.out/err` calls with structured logging
   - **Priority**: MEDIUM
   - **Time**: 30 minutes
   - **Benefit**: Better diagnostics and FRC log integration

3. **TEST**: Verify Xbox controller bindings and speed mode toggling work correctly
   - **Priority**: HIGH
   - **Time**: 15 minutes (on actual robot)

4. **IMPROVE**: Update PathPlanner exception handling to be more specific
   - **Priority**: MEDIUM
   - **Time**: 15 minutes

### Pre-Competition Checklist:

- [ ] Test Limelight vision with actual Limelight hardware
- [ ] Verify snapshot capture functionality (`getLimelightURLString`)
- [ ] Test speed mode toggles (bumpers) with driver Xbox controller
- [ ] Verify all autonomous modes load correctly from PathPlanner
- [ ] Check that telemetry streams to dashboard properly
- [ ] Confirm robot deploys to RoboRIO without errors
- [ ] Test CANdle LED color changes
- [ ] Verify shooter motor control responds correctly

### Code Quality Improvements (Post-Season):

1. Add type-safe logging with DataLogManager throughout codebase
2. Use more specific exception handling (avoid bare `Exception`)
3. Remove hardcoded file paths and outdated comments
4. Add unit tests for vision subsystem error cases
5. Consider migrating to newer Limelight API versions as they become available

---

## 📊 Build Status Summary

| Aspect | Status | Notes |
|--------|--------|-------|
| **Compilation** | ✅ PASS | Java 21 compilation successful |
| **Tests** | ✅ PASS | No test failures |
| **CVE Scan** | ✅ PASS | No known vulnerabilities |
| **Behavior** | ⚠️ CHANGED | URL validation behavior change (see Issue #1) |
| **Warnings** | ⚠️ 1 WARNING | Annotation processor Java 21 support (non-blocking) |

---

## 📝 Issues by Category

### API Changes
- Limelight URL creation behavior change
- URL validation strictness reduction

### Best Practices
- System.out usage (20+ instances)
- Broad exception handling in PathPlanner
- Deprecated method availability

### Code Quality
- Stale file path comments
- Console output instead of structured logging

### Compatibility
- Annotation processor version mismatch (warning only)

---

## Additional Resources

- **WPILib 2026 Changelog**: Check official WPILib documentation for API changes
- **Limelight Documentation**: https://docs.limelightvision.io/
- **PathPlanner Documentation**: https://pathplanner.dev/
- **FRC Logging Best Practices**: Use DataLogManager for all diagnostic output

---

**Analysis Completed**: February 10, 2026  
**Confidence Level**: HIGH (Based on static code analysis)  
**Recommendation**: Address critical issues before deploying to competition robot
