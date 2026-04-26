# Soar Test Agent for Development

The `start` program is a minimum working example of a Soar agent
integrated in ROS2 for communicating. It is used for testing but also as an
example on how to set up the project structure.

The typical setup follows this pattern:

1. Create a `SoarRunner` node.
2. Call `node->addAgent(name, soar_file)` — this returns a `std::shared_ptr<SoarAgent>`.
3. Wire all ROS2 interfaces directly on the returned agent:
    - `agent->addPublisher(...)` / `agent->addSubscriber(...)`
    - `agent->addService(...)` / `agent->addClient(...)`
4. Call `node->startThread()` to begin the Soar decision cycle.

Multiple agents can be created by calling `addAgent()` repeatedly. Each agent
has its own I/O wiring; all agents share the same kernel and are advanced
together via `m_kernel->RunAllAgents(1)` on every step of the run thread.
The `updateWorld()` callback (triggered by `smlEVENT_AFTER_ALL_OUTPUT_PHASES`)
then iterates over all `SoarAgent` instances and calls their `updateWorld()`.

The tests can be executed either via `colcon test` or
`launch_test test/*.py`.

**Known Bug**: Soar does not exit cleanly after unit tests and the `start`
program must be terminated manually.
