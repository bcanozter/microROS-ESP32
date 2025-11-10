# microROS-ESP32

## Technologies

- MacOS 15.3 Intel Chip
- Docker Engine 27.5.1
- Docker Desktop 4.38.0
- ROS Jazzy
- ESP-IDF 5.2
- ESP32-S3 with 8MB PSRAM

## References

- https://github.com/micro-ROS/micro_ros_espidf_component
- https://github.com/espressif/esp32-camera
- https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/

## MacOS Docker Limitation

Unable to mount `/dev/tty.usb*` devices even when running docker with `--privileged` argument. Espressif has a tool to remotely connect to a serial port.

Issue

- https://github.com/docker/for-mac/issues/900

Solution

- [Remote Serial Port](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/tools/idf-docker-image.html#using-remote-serial-port)

## Packages

This will be populated with different packages as I go.

### camera_publisher

#### Build

#### Terminal 1

`docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/microROS-ESP32 -v  /dev:/dev  --privileged --ipc host --workdir /microROS-ESP32 microros-esp32:latest /bin/bash  -c "cd sources/camera_publisher; idf.py  --port 'rfc2217://host.docker.internal:4000?ign_set_control' menuconfig build flash monitor"`

_Ensure PSRAM is enabled. The ESP32-S3 chip I am using has a 8 MB PSRAM. PSRAM Mode is `Octal Mode PSRAM`, and the clock RAM speed is `80MHz`._

_Ensure to set MicroROS Agent WIFI Adress and Port._

#### Terminal 2

`docker run -it --rm --privileged --ipc host  -p "8888:8888/udp" microros/micro-ros-agent:jazzy udp4 --port 8888 -v6`

#### Terminal 3

`esp_rfc2217_server.py -v -p 4000 /dev/tty.usbmodem14401`

#### Output

I used `rqt` to monitor the image topic and visualize. Feel free to use any other ros tool to visualize the image topic.

<img src="/docs/screenshots/Image_Output.png" width="50%" height="50%">

### gpio_interrupt_publisher

Publish to a ros topic on GPIO interrupt trigger.

### reconnection_example

This example demonsrates the ability to reconnect to a ros agent. De-init node entities gracefully before trying to reconnect.
Then, reinitialize necessary components in order to establish a healthy connection to the microros agent before publishing dummy data.

1. Agent running as initial state.
2. Disconnect agent. Observe `Lost connection` output.
3. Start micro-ros agent.
4. Confirm connection.

<img src="/docs/screenshots/reconnection_example.png" width="30%">

[micro-ROS tutorials](https://micro.ros.org/docs/tutorials/programming_rcl_rclc/micro-ROS/)
I came across this helpful micro-ROS tutorial above where I discovered there is a way to ping agent.

Using the `rmw_uros_ping_agent` function to check connection and break the main loop.

```c
if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK)
{
    ...
    printf("Lost connection to micro-ros-agent\n");
    ...
}
```

### joystick_controller

[Turtlesim](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html#use-turtlesim)

<img src="/docs/2_axis_joystick.jpg" width="20%">

Publish `cmd_vel (geometry_msgs/msg/Twist)` using a 2-axis Joystick connected to esp32.

Use the official ROS turtlesim to test and observe the movement on screen.

Result

<img src="/docs/turtlesim.png" width="20%">

### espnow_example

> TODO

## Extras

### Display forwarding MacOS

Check out this guide.

[X11 forwarding on macOS and docker](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088)
