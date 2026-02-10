# Issue Tracker: WPILib 2025→2026 Upgrade

## Quick Reference Summary

### 🔴 CRITICAL (Must Fix)

| # | Issue | File | Line(s) | Severity | Impact |
|---|-------|------|---------|----------|--------|
| 1 | URL validation behavior change | `LimelightHelpers.java` | 918 | 🔴 HIGH | Vision subsystem may not validate Limelight hostnames correctly |

### 🟠 MAJOR (Should Fix)

| # | Issue | File | Line(s) | Severity | Impact |
|---|-------|------|---------|----------|--------|
| 2 | Java 21 annotation processor warning | `build.gradle` | Build output | 🟠 MEDIUM | May cause incomplete annotation processing (non-blocking) |
| 3 | Deprecated Limelight APIs | `LimelightHelpers.java` | 1103-1125 | 🟠 MEDIUM | Future compatibility risk (currently unused) |
| 4 | System.out/err in production | `LimelightHelpers.java` | 830-858, 921 | 🟠 MEDIUM | Poor logging practices; console output not captured |
| 5 | Stale code comments | `RobotContainer.java` | 30 | 🟠 LOW | Code cleanliness issue |
| 6 | CommandXboxController binding | `RobotContainer.java` | 96-105 | 🟠 MEDIUM | Needs testing with actual controller |
| 7 | PathPlanner error handling | `RobotContainer.java` | 124-134 | 🟠 MEDIUM | Overly broad exception handling |

### 🟢 PASSING (No Issues)

| # | Item | File | Status |
|---|------|------|--------|
| 9 | Notifier usage in simulation | `CommandSwerveDrivetrain.java` | ✅ Correct |
| 10 | NetworkTables Struct API | `Telemetry.java` | ✅ Correct |
| 11 | CANdle LED subsystem | `CandleLED.java` | ✅ Updated to Phoenix 6 |
| 12 | Shooter subsystem | `Shooter.java` | ✅ Correct |
| 13 | Java 21 compatibility | All files | ✅ Compiles successfully |
| 14 | Build | `build.gradle` | ✅ Java 21 configured correctly |

---

## Issue Severity Matrix

```
Severity | Count | Priority | Action
---------|-------|----------|--------
🔴 HIGH  |   1   | URGENT   | Fix immediately before testing
🟠 MED   |   6   | SOON     | Fix before competition
🟡 LOW   |   1   | OPTIONAL | Code cleanliness
🟢 PASS  |   6   | NONE     | Working correctly
```

---

## Fix Checklist

### Phase 1: Critical (Session 1)
- [ ] **Issue #1**: Fix URL validation in LimelightHelpers
  - [ ] Revert to `new URL(urlString)` strict validation
  - [ ] Add proper exception throwing
  - [ ] Test with malformed URLs
  - Estimated time: 15 minutes

### Phase 2: Major (Session 2)  
- [ ] **Issue #4**: Replace System.out calls with DataLogManager
  - [ ] Add DataLogManager imports
  - [ ] Replace 20+ System.out.println calls
  - [ ] Replace System.err calls
  - Estimated time: 30 minutes
  
- [ ] **Issue #7**: Improve PathPlanner error handling
  - [ ] Change broad `Exception` to specific types
  - [ ] Pre-test path existence before adding option
  - [ ] Add proper logging
  - Estimated time: 15 minutes

### Phase 3: Testing
- [ ] **Issue #6**: Test Xbox controller bindings on actual robot
  - [ ] Test speed mode toggles (bumpers)
  - [ ] Test vision assist buttons
  - [ ] Test SysId bindings
  - Estimated time: 20 minutes

- [ ] **Issue #3**: Verify no usage of deprecated Limelight methods
  - [ ] Grep codebase for `getBotpose` calls
  - [ ] Verify all vision code uses `getBotPose_*` methods
  - Estimated time: 10 minutes

### Phase 4: Polish
- [ ] **Issue #2**: Update build.gradle for Java 21 processors
  - [ ] Add explicit annotation processor configuration
  - [ ] Rebuild and verify warning is gone
  - Estimated time: 10 minutes

- [ ] **Issue #5**: Remove stale comments
  - [ ] Remove line 30 from RobotContainer.java
  - [ ] Review for other outdated comments
  - Estimated time: 5 minutes

---

## Code Change Examples

### Fix #1: URL Validation (LimelightHelpers.java:914)

**BEFORE (Current - Unsafe)**:
```java
public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    URL url;
    try {
        url = URI.create(urlString).toURL();  // ❌ Lenient parsing
        return url;
    } catch (MalformedURLException e) {
        System.err.println("bad LL URL");
    }
    return null;
}
```

**AFTER (Safe)**:
```java
public static URL getLimelightURLString(String tableName, String request) {
    String urlString = "http://" + sanitizeName(tableName) + ".local:5807/" + request;
    try {
        return new URL(urlString);  // ✅ Strict validation
    } catch (MalformedURLException e) {
        DataLogManager.log("ERROR: Invalid Limelight URL: " + urlString);
        throw new RuntimeException("Limelight URL error", e);
    }
}
```

---

### Fix #4: Logging (LimelightHelpers.java:820-860)

**BEFORE (Current - Not FRC Best Practice)**:
```java
System.out.println("No PoseEstimate available.");
System.out.printf("Pose Estimate Information:%n");
System.out.printf("Timestamp (Seconds): %.3f%n", pose.timestampSeconds);
```

**AFTER (FRC Best Practice)**:
```java
import edu.wpi.first.wpilibj.DataLogManager;

DataLogManager.log("No PoseEstimate available");
DataLogManager.log("Pose Estimate Information");
DataLogManager.log("Timestamp (Seconds): " + String.format("%.3f", pose.timestampSeconds));
```

---

### Fix #7: Exception Handling (RobotContainer.java:124)

**BEFORE (Current - Too Broad)**:
```java
try {
    chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    chooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
    chooser.addOption("midL4x1 (if present)", new PathPlannerAuto("midL4x1"));
} catch (Exception ex) {  // ❌ Catches everything
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    SmartDashboard.putString("Mode/autoChooser/Error", "PathPlanner chooser failed: " + ex.getMessage());
}
```

**AFTER (Specific & Safe)**:
```java
try {
    chooser = AutoBuilder.buildAutoChooser();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
    chooser.addOption("Ex Auto", new PathPlannerAuto("Ex Auto"));
    
    // Only add optional autos if they exist
    try {
        new PathPlannerAuto("midL4x1");  // Test if path exists
        chooser.addOption("midL4x1", new PathPlannerAuto("midL4x1"));
    } catch (RuntimeException ignored) {
        DataLogManager.log("INFO: midL4x1 path not found, skipping");
    }
} catch (RuntimeException ex) {  // ✅ Specific exception type
    DataLogManager.log("ERROR: PathPlanner initialization failed: " + ex);
    chooser = new SendableChooser<>();
    chooser.setDefaultOption("Do Nothing", new InstantCommand());
}
```

---

## Testing Strategy

### Unit Testing
1. **URL Validation**: Test `getLimelightURLString()` with:
   - Valid hostnames: `limelight-shooter.local:5807`
   - Invalid formats: spaces, special chars, missing segments
   - Should throw exception for invalid URLs

2. **Logging**: Verify all logs appear in robot logs, not just console

### Integration Testing
1. **Vision**: Connect real Limelight, test snapshot capture
2. **Controls**: Test Xbox bumper toggles and vision assists on actual robot
3. **Autonomous**: Verify all PathPlanner autos load correctly
4. **Dashboard**: Confirm telemetry streams properly

### Regression Testing
- [ ] All subsystems functional (drivetrain, shooter, climber, LEDs)
- [ ] All commands execute without errors
- [ ] Dashboard shows correct telemetry
- [ ] Log file size is reasonable (not spammed with output)

---

## Risk Assessment

| Issue | Likelihood | Impact | Risk Level |
|-------|-----------|--------|-----------|
| #1 - URL Validation | Medium | High | 🔴 HIGH |
| #4 - System.out | Low | Medium | 🟠 MEDIUM |
| #7 - Error Handling | Low | Medium | 🟠 MEDIUM |
| #6 - Xbox Bindings | Medium | High | 🟠 MEDIUM |
| #2 - Annotation Proc | Low | Low | 🟡 LOW |

---

## Dependencies & Vendor Libs Status

### ✅ Updated Successfully
- Phoenix 6 (2026)
- PathPlanner (compatible)
- WPILib (2026)
- Limelight Helpers (v1.11)
- REVLib (compatible)
- PhotonLib (compatible)

### ⚠️ Monitor
- Annotation processors (Java 21 warning)
- Custom Limelight integration (URL changes)

---

## Resources & References

- WPILib 2026 Docs: https://docs.wpilib.org/
- Limelight Docs: https://docs.limelightvision.io/
- PathPlanner Docs: https://pathplanner.dev/
- CTRE Phoenix 6 Docs: https://pro.docs.ctr-electronics.com/

---

**Document Version**: 1.0  
**Last Updated**: February 10, 2026  
**Status**: 🟠 7 Issues Identified - Ready for Remediation
