# Class Diagrams

The following class diagram of the soar_ros::Service class (inheritance diagram)
provides a basic overview of the library architecture.

All ROS wrapper classes adjusted for usage with Soar are based on the input or
output base classes as well as the shared interface.

```{eval-rst}
.. doxygenclass:: soar_ros::Service
   :allow-dot-graphs:
```

The difference bettwen service and client from the libraries perspective is
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
