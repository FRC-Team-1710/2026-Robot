## Coding Standards

### Naming Conventions
- When working in files or packages that already use this pattern, prefix private instance variables (non-static fields) with `m_` (e.g., `m_speed`, `m_motor`). Otherwise, follow the existing naming style in that file.
- Likewise, where this convention is already in use, prefer prefixing `static final` constants with `k` (e.g., `kMaxSpeed`, `kDefaultTimeout`); otherwise, match the surrounding code (such as ALL_CAPS constants where appropriate).

### Documentation
- Add Javadoc comments before all public methods

### Example
```java
public class ExampleSubsystem {

    private static final double kMaxSpeed = 1.0;
    private double m_currentSpeed;

    /**
     * Sets the speed of the subsystem.
     * @param speed The desired speed, between 0 and kMaxSpeed.
     */
    public void setSpeed(double speed) {
        m_currentSpeed = speed;
    }
}
```
