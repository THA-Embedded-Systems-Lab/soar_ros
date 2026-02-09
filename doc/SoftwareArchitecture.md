# Software Architecture

The rationale behind this library is also simplifying the use of Soar via ROS2
while the API should remain as simple as possible. The challenges behind
connecting Soar and ROS2 lies in the software architecture of Soar: synchronous
callbacks and events. A detailed explanation of the Soar architecture is
provided on their website in the [Soar manual][soar_manual] or the [Soar
Threading Model][soar_threads].

The user should only be required to do two things: Initialize the Soar
kernel via `SoarRunner`, add one or more agents with `addAgent`, and then add
adapted publisher, subscriber, services and clients. For each message/ topic,
the conversion between Soar working memory elements (WMEs) and ROS2 message, or
vice versa, types must be implemented manually.

- The kernel is wrapped in a single class called `SoarRunner`. The main
responsibility is to create, start and maintain the Soar kernel and manage
multiple agents in one kernel.

- Publisher, Subscribers, Service and Clients are added via a separate functions
to the ROS2 node.

```{mermaid}
:zoom: %% enable for rosdoc2 feature, requires sphinxcontrib.mermaid feature.
sequenceDiagram
box Soar
participant SoarRunner
participant SoarRunner-RunThread
participant SoarKernel
end

box ROS2
participant Client
participant Client-RunThread
participant ClientInputQueue
participant ClientOutputQueue
end

SoarRunner ->> SoarKernel: CreateKernelInNewThread()
activate SoarKernel;
activate SoarRunner

%% INITIALIZE CLIENT
SoarRunner ->> Client : AddClient(client)
activate Client
Client ->> Client-RunThread: InitializeRun
deactivate Client
activate Client-RunThread
par ClientRun
    note right of Client: Start client worker thread.
    loop Client run thread
        Client-RunThread ->>+ ClientInputQueue: try read
        ClientInputQueue ->>- Client-RunThread: Element
        Client-RunThread ->> Client-RunThread: sent ROS2 request
        Client-RunThread ->> Client-RunThread: await ROS2 response
        Client-RunThread ->> Client-RunThread: future.get()
        Client-RunThread ->> ClientOutputQueue: push(response)
        deactivate Client-RunThread
    end
and Agent run
    note right of SoarRunner: Start agent worker thread.
    SoarRunner ->> SoarRunner-RunThread: Run()
    deactivate SoarRunner
    activate SoarRunner-RunThread
    loop continue == true
        SoarRunner-RunThread ->> SoarRunner-RunThread: RunAllAgentsForever()
    end
    deactivate SoarRunner-RunThread
and Kernel run
    note right of SoarRunner: Callback for Event is called.
    SoarKernel ->> SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES callback to UpdateWorld()
    deactivate SoarKernel
    activate SoarRunner
    SoarRunner ->> SoarRunner: processOutputLinkChanges()
    SoarRunner ->> SoarRunner: processInput()
    SoarRunner ->> SoarKernel: UpdateWorld() complete
    activate SoarKernel
    deactivate SoarRunner
end

note right of SoarRunner: Example for Client call from Soar: Process output
SoarKernel ->>+ SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES callback to UpdateWorld()
deactivate SoarKernel;
activate SoarRunner
SoarRunner ->> SoarRunner: outputs[output-link.command] = shared_ptr output
SoarRunner ->>+ Client: process_s2r()
    deactivate SoarRunner
    Client ->> ClientInputQueue: queue.push(parse(sml:Identifier *))
    note right of ClientInputQueue: Client run thread reads and processes queue.
    Client ->>- SoarRunner: void
    activate SoarRunner
SoarRunner ->> SoarRunner: output-link.command.AddStatusComplete()
SoarRunner ->> SoarRunner: processInput()
SoarRunner ->>- SoarKernel: UpdateWorld() complete
activate SoarKernel;

loop SoarKernel and SoarRunner not blocked
SoarKernel ->>+ SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES callback to UpdateWorld()
SoarRunner ->>- SoarKernel: UpdateWorld() complete
end

SoarKernel ->> SoarRunner: smlEVENT_AFTER_ALL_OUTPUT_PHASES callback to UpdateWorld()
activate SoarRunner;
deactivate SoarKernel;
SoarRunner ->> SoarRunner: processOutputLinkChanges()
note right of Client: Check if response is available.
SoarRunner ->>+ Client: Process input
deactivate SoarRunner;
    Client ->>+ ClientOutputQueue: Read Queue
    ClientOutputQueue ->>- Client: Element
    Client ->> Client: parse()
    Client ->>- SoarRunner: void
activate SoarRunner
SoarRunner ->> SoarRunner: agent.commit()
SoarRunner ->> SoarKernel: UpdateWorld() complete
activate SoarKernel;

deactivate SoarKernel;
deactivate SoarRunner;
```

[soar_manual]: https://soar.eecs.umich.edu/soar_manual/
[soar_threads]: https://soar.eecs.umich.edu/development/soar/ThreadsInSML/
