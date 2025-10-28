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

![Image Output](/docs/screenshots/Image_Output.png "Output")

### gpio_interrupt_publisher

Publish to a ros topic on GPIO interrupt trigger.

>TODO

### reconnection_example

Reconnect to micro-ros-agent upon agent disconnect. 

>TODO

### joystick_controller

Publish `cmd_vel (geometry_msgs/msg/Twist)` using a Joystick connected to esp32.

>TODO

## Extras

### Display forwarding MacOS

Check out this guide.

[X11 forwarding on macOS and docker](https://gist.github.com/sorny/969fe55d85c9b0035b0109a31cbcb088)