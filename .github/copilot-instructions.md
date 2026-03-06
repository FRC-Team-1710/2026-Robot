## Coding Standards

### Naming Conventions
- Prefix all private instance variables (non-static fields) with `m_` (e.g., `m_speed`, `m_motor`)
- Prefix all `static final` constants with `k` (e.g., `kMaxSpeed`, `kDefaultTimeout`)

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
