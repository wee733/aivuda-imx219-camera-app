# Isaac ROS 5 sensor runtime

The camera and AprilTag archives each include this directory. The runtime starts the installed Isaac ROS image with the application user, a read-only app mount, local GPU device access, and shared host networking, IPC, PID namespace and /tmp. CUDA IPC exchanges buffer descriptors between local ROS processes. Startup clears the cached image’s DISPLAY to allow headless EGL on this Jetson. The in-container supervisor forwards shutdown and watches the foreground Docker client with a Linux pidfd.

The ROS graph uses the configured ROS_DOMAIN_ID. The bundled release of cuda_buffer_backend separately uses the upstream local endpoint registry at index 0 and looks up DDS endpoint GIDs to negotiate CUDA IPC. The migrated deployment uses DDS domain 0. Discovery and GPU transfer on alternate DDS domains require a matching ROS middleware registry.

The packaged plugin was built from ros2/rosidl_buffer_backends 0.1.2 with the onboard NODELETE linker change documented in cuda_buffer_backend/UPSTREAM.txt. Its source, manifest, pluginlib index and licence are included. To rebuild in the matching Lyrical image, source /opt/ros/lyrical/setup.bash and build cuda_buffer_backend with colcon and -DBUILD_TESTING=OFF. The pre-install check verifies the shipped plugin ELF checksum.
