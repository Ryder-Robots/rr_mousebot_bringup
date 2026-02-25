# Testing

## Node Take Up

```bash
ros2 node list --no-daemon
/driver/driver_container
/driver/motor_controller
/launch_ros_4817
/lifecycle_manager_navigation
root@mazebot:/home/aaron/ros2_ws# ros2 lifecycle nodes
/driver/motor_controller
```

### Check State

```bash
ws# ros2 lifecycle get /driver/motor_controller
unconfigured [1]
```


```bash
root@mazebot:/home/aaron/ros2_ws# ros2 lifecycle set /driver/motor_controller configure
Transitioning successful
```

#### Log Results

```text
[component_container_mt-1] [DEBUG] [1771999544.408227173] [pluginlib.ClassLoader]: Checking path /home/aaron/ros2_ws/install/rr_gpio_pi4b_pigpio_plugin/lib/librr_gpio_pi4b_pigpio_plugin.so 
[component_container_mt-1] [DEBUG] [1771999544.408403343] [pluginlib.ClassLoader]: Library rr_gpio_pi4b_pigpio_plugin found at explicit path /home/aaron/ros2_ws/install/rr_gpio_pi4b_pigpio_plugin/lib/librr_gpio_pi4b_pigpio_plugin.so.
[component_container_mt-1] [DEBUG] [1771999544.432989701] [pluginlib.ClassLoader]: rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin maps to real class type rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin
[component_container_mt-1] [DEBUG] [1771999544.433137056] [pluginlib.ClassLoader]: std::unique_ptr to object of real type rr_gpio_pi4b_pigpio_plugin::RrGpioPi4BPigpioPlugin created.
[component_container_mt-1] [INFO] [1771999544.434076168] [GPIO_PI4]: configuring GPIO
[component_container_mt-1] [INFO] [1771999544.434581345] [driver.motor_controller]: Configuring motor controller...
[component_container_mt-1] [INFO] [1771999544.434747441] [Motor]: Configuring motor...
[component_container_mt-1] [INFO] [1771999544.435009891] [MotorEncoder]: Configuring encoder...
[component_container_mt-1] [INFO] [1771999544.435073225] [driver.motor_controller]: Configuring motor controller...
[component_container_mt-1] [INFO] [1771999544.435163375] [Motor]: Configuring motor...
[component_container_mt-1] [INFO] [1771999544.435236766] [MotorEncoder]: Configuring encoder...
[component_container_mt-1] [DEBUG] [1771999544.435370213] [rcl]: Sending service response
```

Activate failed:

```text
[component_container_mt-1] [INFO] [1771999544.435236766] [MotorEncoder]: Configuring encoder...
[component_container_mt-1] [DEBUG] [1771999544.435370213] [rcl]: Sending service response
[component_container_mt-1] [DEBUG] [1771999631.932319952] [rcl]: Service server taking service request
[component_container_mt-1] [DEBUG] [1771999631.932450695] [rcl]: Service take request succeeded: true
[component_container_mt-1] [DEBUG] [1771999631.932488714] [rcl]: Sending service response
[component_container_mt-1] [DEBUG] [1771999631.939745019] [rcl]: Service server taking service request
[component_container_mt-1] [DEBUG] [1771999631.939873484] [rcl]: Service take request succeeded: true
[component_container_mt-1] [INFO] [1771999632.069518287] [Motor]: Activating motor...
[component_container_mt-1] [INFO] [1771999632.071337224] [Motor]: Activating motor...
[component_container_mt-1] [ERROR] [1771999632.071550042] [GPIO_PI4]: set_isr_func_ex - PI_BAD_ISR_INIT
[component_container_mt-1] [ERROR] [1771999632.071672656] [MotorEncoder]: could not attach ISR callback to pin 8...
[component_container_mt-1] [INFO] [1771999632.071732731] [Motor]: Deactivating motor...
[component_container_mt-1] [ERROR] [1771999632.071786454] [driver.motor_controller]: Encoder activation failed!!
[component_container_mt-1] [INFO] [1771999632.172069734] [Motor]: Deactivating motor...
[component_container_mt-1] [INFO] [1771999632.172288033] [Motor]: Deactivating motor...
[component_container_mt-1] [FATAL] [1771999632.172375331] [driver.motor_controller]: failed to activate motor(s)
[component_container_mt-1] [DEBUG] [1771999632.200997191] [rcl]: Sending service response
```

This could be caused by incorrect PIGPIO library getting used.
