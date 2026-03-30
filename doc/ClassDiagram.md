# Class Diagrams

The library is structured around two main classes: `SoarRunner` and `SoarAgent`.
`SoarRunner` owns the `sml::Kernel`, the run thread and the ROS2 service
interfaces. `SoarAgent` wraps a single `sml::Agent` and owns all ROS2 I/O
objects (publishers, subscribers, services, clients) for that agent.

```{eval-rst}
.. doxygenclass:: soar_ros::SoarRunner
   :allow-dot-graphs:
```

```{eval-rst}
.. doxygenclass:: soar_ros::SoarAgent
   :allow-dot-graphs:
```

The following class diagram of the soar_ros::Service class (inheritance diagram)
provides a basic overview of the ROS2 interface wrappers.

All ROS wrapper classes adjusted for usage with Soar are based on the input or
output base classes as well as the shared interface.

```{eval-rst}
.. doxygenclass:: soar_ros::Service
   :allow-dot-graphs:
```

The difference between service and client from the libraries perspective is
only the reversal of output and input calls.

```{eval-rst}
.. doxygenclass:: soar_ros::Client
   :allow-dot-graphs:
```

Publisher and subscriber only have either the input or the output functionality:

```{eval-rst}
.. doxygenclass:: soar_ros::Publisher
   :allow-dot-graphs:
```

```{eval-rst}
.. doxygenclass:: soar_ros::Subscriber
   :allow-dot-graphs:
```
