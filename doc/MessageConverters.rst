Message Converters
==================

``soar_ros`` ships ready-made converters for the most common ROS 2 message
types. Including ``soar_ros/msg/converters.hpp`` (or the umbrella header
``soar_ros/soar_ros.hpp``) exposes two free functions in the
``soar_ros::msg`` namespace:

+--------------------------------+---------------------------------------------------+
| Function                       | Direction                                         |
+================================+===================================================+
| ``toSoar(parent, attr, msg)``  | ROS 2 message → child ID WME under *parent*       |
+--------------------------------+---------------------------------------------------+
| ``fromSoar<T>(id)``            | Soar WME subtree → ROS 2 message of type *T*      |
+--------------------------------+---------------------------------------------------+

Supported types
---------------

The converters are split into two groups depending on how they are included.

**Always available** — included automatically by ``soar_ros/soar_ros.hpp``
(these packages are hard dependencies of soar_ros):

- ``std_msgs``: ``Bool``, ``Int8``, ``Int16``, ``Int32``, ``Int64``,
  ``UInt8``, ``UInt16``, ``UInt32``, ``UInt64``, ``Float32``, ``Float64``,
  ``String``, ``Header``, ``ColorRGBA``
- ``geometry_msgs``: ``Vector3``, ``Point``, ``Point32``, ``Quaternion``,
  ``Pose``, ``PoseStamped``, ``PoseWithCovariance``, ``Twist``,
  ``TwistStamped``, ``TwistWithCovariance``, ``TwistWithCovarianceStamped``,
  ``Accel``, ``AccelWithCovariance``, ``AccelWithCovarianceStamped``,
  ``Transform``, ``TransformStamped``

**Optional** — each requires a separate include and the corresponding ROS 2
package as a dependency in the consuming package's ``package.xml`` and
``CMakeLists.txt``:

+---------------------+-----------------------------------------------------+----------------------------------------------+
| Include             | Requires                                            | Covered messages                             |
+=====================+=====================================================+==============================================+
| ``builtin_interfaces | ``builtin_interfaces``                              | ``Time``, ``Duration``                       |
| _converters.hpp``   |                                                     |                                              |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``sensor_msgs_      | ``sensor_msgs``                                     | ``Temperature``, ``FluidPressure``,          |
| converters.hpp``    |                                                     | ``Illuminance``, ``Range``,                  |
|                     |                                                     | ``NavSatStatus``, ``NavSatFix``,             |
|                     |                                                     | ``MagneticField``, ``Imu``,                  |
|                     |                                                     | ``JointState``, ``LaserScan``                |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``nav_msgs_         | ``nav_msgs``                                        | ``MapMetaData``, ``Odometry``, ``Path``      |
| converters.hpp``    |                                                     |                                              |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``visualization_    | ``visualization_msgs``                              | ``Marker``, ``MarkerArray``                  |
| msgs_converters     |                                                     |                                              |
| .hpp``              |                                                     |                                              |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``tf2_msgs_         | ``tf2_msgs``                                        | ``TFMessage``                                |
| converters.hpp``    |                                                     |                                              |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``action_msgs_      | ``action_msgs``,                                    | ``GoalInfo``, ``GoalStatus``,                |
| converters.hpp``    | ``unique_identifier_msgs``                          | ``GoalStatusArray``                          |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``diagnostic_msgs_  | ``diagnostic_msgs``                                 | ``KeyValue``, ``DiagnosticStatus``,          |
| converters.hpp``    |                                                     | ``DiagnosticArray``                          |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``control_msgs_     | ``control_msgs``                                    | ``GripperCommand``, ``JointTolerance``,      |
| converters.hpp``    |                                                     | ``JointJog``                                 |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``trajectory_msgs_  | ``trajectory_msgs``                                 | ``JointTrajectoryPoint``,                    |
| converters.hpp``    |                                                     | ``JointTrajectory``                          |
+---------------------+-----------------------------------------------------+----------------------------------------------+
| ``moveit_msgs_      | ``moveit_msgs``, ``sensor_msgs``,                   | ``MoveItErrorCodes``,                        |
| converters.hpp``    | ``trajectory_msgs``                                 | ``JointConstraint``, ``RobotState``          |
+---------------------+-----------------------------------------------------+----------------------------------------------+

WME naming conventions
-----------------------

- Attribute names match the ROS 2 field names exactly (e.g. ``frame_id``,
  ``child_frame_id``, ``linear``, ``angular``).
- Nested message fields become child ID WMEs; scalar fields become int or
  float leaf WMEs directly on the parent ID.
- ``PoseWithCovariance`` / ``TwistWithCovariance`` / ``AccelWithCovariance``:
  36-element covariance arrays are stored as flat float attributes
  ``covariance_0`` … ``covariance_35``.
- ``Bool``: ``data`` is stored as an int WME (``1`` = true, ``0`` = false).
- **Variable-length arrays** (``JointState::position``, ``LaserScan::ranges``,
  ``Path::poses``, etc.) use indexed WMEs: a ``<field>_count`` int WME plus
  ``<field>_0``, ``<field>_1``, … For scalar arrays (float/int/string) the
  helpers in ``soar_ros/msg/detail.hpp`` are used directly; for arrays of
  nested messages the same ``_count`` / ``_N`` pattern applies with child
  ID WMEs.

  .. note::

     Large arrays (e.g. ``LaserScan`` with 1080 rays) produce many WMEs and
     can slow down the Soar decision cycle. Consider downsampling or filtering
     the data before calling ``toSoar`` for performance-sensitive agents.

Usage examples
--------------

**Subscriber** — writing a ``PoseStamped`` onto the input link:

.. code:: cpp

   #include "soar_ros/msg/converters.hpp"

   class PoseSubscriber : public soar_ros::Subscriber<geometry_msgs::msg::PoseStamped>
   {
   public:
     using Subscriber<geometry_msgs::msg::PoseStamped>::Subscriber;

     void parse(geometry_msgs::msg::PoseStamped msg) override
     {
       soar_ros::msg::toSoar(m_pAgent->GetInputLink(), m_topic.c_str(), msg);
       // Resulting WME tree on <input-link>:
       //   <topic> (ID)
       //     +-- header (ID)
       //     |     +-- frame_id  (string)
       //     |     +-- stamp (ID)  →  sec (int), nanosec (int)
       //     +-- pose (ID)
       //           +-- position    (ID)  →  x, y, z (float)
       //           +-- orientation (ID)  →  x, y, z, w (float)
     }
   };

**Publisher** — reading a ``Twist`` from the output link:

.. code:: cpp

   class TwistPublisher : public soar_ros::Publisher<geometry_msgs::msg::Twist>
   {
   public:
     using Publisher<geometry_msgs::msg::Twist>::Publisher;

     geometry_msgs::msg::Twist parse(sml::Identifier * id) override
     {
       // Soar rule must write:
       //   (<output-link> ^cmd <c>)
       //   (<c> ^linear <l> ^angular <a>)
       //   (<l> ^x 1.0 ^y 0.0 ^z 0.0)
       //   (<a> ^x 0.0 ^y 0.0 ^z 0.5)
       return soar_ros::msg::fromSoar<geometry_msgs::msg::Twist>(id);
     }
   };

Extending converters to other message packages
----------------------------------------------

Adding support for a new message package requires three steps.

Step 1 – Create a converter header
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Create a new header under ``include/soar_ros/msg/``, e.g.
``sensor_msgs_converters.hpp``.  Include ``detail.hpp`` for the shared
helper utilities and ``detail.hpp`` provides:

- ``detail::getFloat(id, attr)`` — reads a float WME attribute as ``double``
- ``detail::getInt(id, attr)``   — reads an int WME attribute as ``int64_t``
- ``detail::getString(id, attr)``— reads a string WME attribute
- ``detail::getChild(id, attr)`` — returns the child ``sml::Identifier*``
  for a nested ID WME

For each message type provide one ``toSoar`` overload and one ``fromSoar``
explicit specialization:

.. code:: cpp

   // include/soar_ros/msg/sensor_msgs_converters.hpp
   #ifndef SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_
   #define SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_

   #include <sml_Client.h>
   #include <sensor_msgs/msg/temperature.hpp>
   #include "soar_ros/msg/detail.hpp"

   namespace soar_ros::msg
   {
       // ── Temperature ───────────────────────────────────────────────────────────
       // WME structure:
       //   <attr> (ID)
       //     +-- temperature (float)
       //     +-- variance    (float)

       inline sml::Identifier * toSoar(
           sml::Identifier * parent, const char * attr,
           const sensor_msgs::msg::Temperature & msg)
       {
           auto * id = parent->CreateIdWME(attr);
           id->CreateFloatWME("temperature", msg.temperature);
           id->CreateFloatWME("variance",    msg.variance);
           return id;
       }

       template <>
       inline sensor_msgs::msg::Temperature
       fromSoar<sensor_msgs::msg::Temperature>(sml::Identifier * id)
       {
           sensor_msgs::msg::Temperature msg;
           msg.temperature = static_cast<double>(detail::getFloat(id, "temperature"));
           msg.variance    = static_cast<double>(detail::getFloat(id, "variance"));
           return msg;
       }

   } // namespace soar_ros::msg

   #endif // SOAR_ROS__MSG__SENSOR_MSGS_CONVERTERS_HPP_

For messages with nested fields, call ``toSoar`` / ``fromSoar`` recursively
and use ``detail::getChild`` to navigate to the child identifier:

.. code:: cpp

   // Nested example: a hypothetical Stamped wrapper
   inline sml::Identifier * toSoar(
       sml::Identifier * parent, const char * attr,
       const sensor_msgs::msg::TemperatureStamped & msg)
   {
       auto * id = parent->CreateIdWME(attr);
       toSoar(id, "header",      msg.header);       // reuse std_msgs::Header
       toSoar(id, "temperature", msg.temperature);  // reuse Temperature
       return id;
   }

   template <>
   inline sensor_msgs::msg::TemperatureStamped
   fromSoar<sensor_msgs::msg::TemperatureStamped>(sml::Identifier * id)
   {
       sensor_msgs::msg::TemperatureStamped msg;
       msg.header      = fromSoar<std_msgs::msg::Header>(detail::getChild(id, "header"));
       msg.temperature = fromSoar<sensor_msgs::msg::Temperature>(
                             detail::getChild(id, "temperature"));
       return msg;
   }

Step 2 – Register the dependency
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add the ROS 2 package to ``package.xml``:

.. code:: xml

   <depend>sensor_msgs</depend>

And to ``CMakeLists.txt``:

.. code:: cmake

   find_package(sensor_msgs REQUIRED)

   # add to ament_target_dependencies / THIS_PACKAGE_INCLUDE_DEPENDS
   set(THIS_PACKAGE_INCLUDE_DEPENDS
     ...
     sensor_msgs
   )

   # link test targets against the new package
   target_link_libraries(my_test ... ${sensor_msgs_TARGETS})

Step 3 – Expose via the umbrella header
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Add an include to ``include/soar_ros/msg/converters.hpp`` so users pick
up the new converters automatically with ``soar_ros/soar_ros.hpp``:

.. code:: cpp

   #include "soar_ros/msg/sensor_msgs_converters.hpp"

Using custom converters from an external package
------------------------------------------------

If you are working in your own ROS 2 package that depends on ``soar_ros``
you do **not** need to modify ``soar_ros`` at all. Because all converters
are header-only and ``fromSoar`` is a function template, you can add
specializations for your own message types inside your own package by
reopening the ``soar_ros::msg`` namespace.

Step 1 – Create a converter header in your package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code:: cpp

   // my_pkg/include/my_pkg/msg/my_msgs_converters.hpp
   #pragma once

   #include "soar_ros/soar_ros.hpp"                  // pulls in detail.hpp + built-in converters
   #include <my_interfaces/msg/my_pose.hpp>

   namespace soar_ros::msg   // reopen – no modification to soar_ros required
   {
       // ── MyPose ────────────────────────────────────────────────────────────
       // WME structure:
       //   <attr> (ID)
       //     +-- x     (float)
       //     +-- y     (float)
       //     +-- label (string)

       inline sml::Identifier * toSoar(
           sml::Identifier * parent, const char * attr,
           const my_interfaces::msg::MyPose & msg)
       {
           auto * id = parent->CreateIdWME(attr);
           id->CreateFloatWME("x",      msg.x);
           id->CreateFloatWME("y",      msg.y);
           id->CreateStringWME("label", msg.label.c_str());
           return id;
       }

       template <>
       inline my_interfaces::msg::MyPose
       fromSoar<my_interfaces::msg::MyPose>(sml::Identifier * id)
       {
           my_interfaces::msg::MyPose msg;
           msg.x     = static_cast<float>(detail::getFloat(id, "x"));
           msg.y     = static_cast<float>(detail::getFloat(id, "y"));
           msg.label = detail::getString(id, "label");
           return msg;
       }

   } // namespace soar_ros::msg

Include it wherever you need conversions:

.. code:: cpp

   // my_node.cpp
   #include "my_pkg/msg/my_msgs_converters.hpp"

   class MySubscriber : public soar_ros::Subscriber<my_interfaces::msg::MyPose>
   {
   public:
     using Subscriber<my_interfaces::msg::MyPose>::Subscriber;

     void parse(my_interfaces::msg::MyPose msg) override
     {
       soar_ros::msg::toSoar(m_pAgent->GetInputLink(), m_topic.c_str(), msg);
     }
   };

.. note::

   The ``fromSoar<T>`` explicit specialization must be visible at every
   translation unit that calls it.  Include your converter header in every
   ``.cpp`` file that calls ``fromSoar``, not just where the subscriber
   class is defined.

Step 2 – Update your package's build files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``package.xml``:

.. code:: xml

   <depend>soar_ros</depend>
   <depend>my_interfaces</depend>

``CMakeLists.txt``:

.. code:: cmake

   find_package(soar_ros       REQUIRED)
   find_package(my_interfaces  REQUIRED)

   add_executable(my_node src/my_node.cpp)

   target_include_directories(my_node PRIVATE
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

   target_link_libraries(my_node
     soar_ros
     ${my_interfaces_TARGETS})

No other changes are needed — the ``soar_ros`` package itself stays
unmodified.
