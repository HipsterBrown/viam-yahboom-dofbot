# Yahboom DofBot Robotic Arm Module

This is a [Viam module](https://docs.viam.com/how-tos/create-module/) for the [Yahboom DofBot](https://category.yahboom.net/collections/r-robotics-arm/products/dofbot-pi) robotic arm.
It provides access to teleoperate and integrate motion planning into this affordable starter arm.

## Model hipsterbrown:dofbot:arm

Control the individual joints and end effector position of the Yahboom DofBot.

### Configuration
The following attribute template can be used to configure this model:

```json
{
"speed_degs_per_sec": <float>,
"acceleration_degs_per_sec_per_sec": <float>
}
```

#### Attributes

The following attributes are available for this model:

| Name          | Type   | Inclusion | Description                |
|---------------|--------|-----------|----------------------------|
| `speed_degs_per_sec` | float  | Optional  |The rotational speed of the joints (must be greater than 3 and less than 180). Defaults to 10 degrees/second. |
| `acceleration_degs_per_sec_per_sec` | float | Optional  | The acceleration of joints in radians per second increase per second. The default is 20 degrees/second^2 |

#### Example Configuration

```json
{
  "speed_degs_per_sec": 12,
  "acceleration_degs_per_sec_per_sec": 0
}
```

## Model hipsterbrown:dofbot:gripper

Control the gripper of the Yahboom DofBot.

### Configuration

No configuration required.
