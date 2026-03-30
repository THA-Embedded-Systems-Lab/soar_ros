# Software Architecture

The rationale behind this library is also simplifying the use of Soar via ROS2
while the API should remain as simple as possible. The challenges behind
connecting Soar and ROS2 lies in the software architecture of Soar: synchronous
callbacks and events. A detailed explanation of the Soar architecture is
provided on their website in the [Soar manual][soar_manual] or the [Soar
Threading Model][soar_threads].

The user is required to do three things: initialize the Soar kernel via
`SoarRunner`, add one or more agents with `SoarRunner::addAgent()` — which
returns a `SoarAgent` handle — and then register publishers, subscribers,
services and clients directly on each `SoarAgent`. For each message / topic
the conversion between Soar working memory elements (WMEs) and ROS2 message
types must be implemented manually.

- `SoarRunner` owns the `sml::Kernel` and the run thread, creates `sml::Agent`
  instances and wraps them in `SoarAgent` objects. It also exposes ROS2 service
  interfaces to start/stop the kernel and launch the Soar Java debugger.

- `SoarAgent` owns all ROS2 I/O wiring for a single agent. Publisher,
  Subscriber, Service and Client objects are registered directly on the agent
  via `agent->addPublisher()`, `agent->addSubscriber()`, `agent->addService()`
  and `agent->addClient()`.

```{mermaid}
:zoom: %% enable for rosdoc2 feature, requires sphinxcontrib.mermaid feature.
sequenceDiagram
box soar_ros
participant SoarRunner
participant SoarRunner-RunThread
participant SoarAgent
participant SoarKernel
end

box ROS2
participant Client
participant Client-RunThread
participant ClientInputQueue
participant ClientOutputQueue
end

SoarRunner ->> SoarKernel: CreateKernelInNewThread()
activate SoarKernel
activate SoarRunner

%% SETUP: addAgent + wire I/O on the returned SoarAgent
SoarRunner ->> SoarAgent: addAgent(name, soar_file)
activate SoarAgent
note right of SoarAgent: SoarAgent wraps one sml::Agent and
note right of SoarAgent: owns all its ROS2 I/O objects.
SoarAgent ->> Client: addClient(client)
activate Client
Client ->> Client-RunThread: InitializeRun
deactivate Client
activate Client-RunThread
deactivate SoarAgent

par ClientRun
    note right of Client: Start client worker thread.
    loop Client run thread
        Client-RunThread ->>+ ClientInputQueue: try read
        ClientInputQueue ->>- Client-RunThread: Element
        Client-RunThread ->> Client-RunThread: send ROS2 request
        Client-RunThread ->> Client-RunThread: await ROS2 response
        Client-RunThread ->> Client-RunThread: future.get()
        Client-RunThread ->> ClientOutputQueue: push(response)
        deactivate Client-RunThread
    end
and Agent run
    note right of SoarRunner: Start run thread — calls RunAllAgents(1) each step.
    SoarRunner ->> SoarRunner-RunThread: startThread()
    deactivate SoarRunner
    activate SoarRunner-RunThread
    loop m_running == true
        SoarRunner-RunThread ->> SoarKernel: RunAllAgents(1)
    end
    deactivate SoarRunner-RunThread
and Kernel callback
    note right of SoarKernel: Kernel fires callback after every output phase.
    SoarKernel ->> SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES → updateWorld()
    deactivate SoarKernel
    activate SoarRunner
    loop for each SoarAgent
        SoarRunner ->> SoarAgent: updateWorld()
        activate SoarAgent
        SoarAgent ->> SoarAgent: processOutputLinkChanges()
        SoarAgent ->> SoarAgent: processInput() + agent->Commit()
        deactivate SoarAgent
    end
    SoarRunner ->> SoarKernel: updateWorld() complete
    activate SoarKernel
    deactivate SoarRunner
end

note right of SoarRunner: Example: Client call triggered from Soar output-link
SoarKernel ->>+ SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES → updateWorld()
deactivate SoarKernel
activate SoarRunner
SoarRunner ->> SoarAgent: updateWorld()
activate SoarAgent
SoarAgent ->> SoarAgent: processOutputLinkChanges()
SoarAgent ->>+ Client: process_s2r()
    Client ->> ClientInputQueue: queue.push(parse(sml::Identifier *))
    note right of ClientInputQueue: Client run thread reads and processes queue.
    Client ->>- SoarAgent: void
SoarAgent ->> SoarAgent: output-link.command.AddStatusComplete()
SoarAgent ->> SoarAgent: processInput()
deactivate SoarAgent
SoarRunner ->>- SoarKernel: updateWorld() complete
activate SoarKernel

loop SoarKernel and SoarRunner not blocked
SoarKernel ->>+ SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES → updateWorld()
SoarRunner ->>- SoarKernel: updateWorld() complete
end

SoarKernel ->> SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES → updateWorld()
activate SoarRunner
deactivate SoarKernel
SoarRunner ->> SoarAgent: updateWorld()
activate SoarAgent
SoarAgent ->> SoarAgent: processOutputLinkChanges()
note right of Client: Check if response is available.
SoarAgent ->>+ Client: processInput()
    Client ->>+ ClientOutputQueue: Read Queue
    ClientOutputQueue ->>- Client: Element
    Client ->> Client: parse()
    Client ->>- SoarAgent: void
SoarAgent ->> SoarAgent: agent->Commit()
deactivate SoarAgent
SoarRunner ->> SoarKernel: updateWorld() complete
activate SoarKernel
deactivate SoarRunner
```

[soar_manual]: https://soar.eecs.umich.edu/soar_manual/
[soar_threads]: https://soar.eecs.umich.edu/development/soar/ThreadsInSML/
