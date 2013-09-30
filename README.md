Conman
======

***WARNING: THIS IS SUPER-UNSTABLE.***

## Introduction

*Conman* is a controller and state estimator manager built on top of the
[Oroocos Toolchain](http://www.orocos.org) and designed to create a common
platform for sharing robot controllers and state estimators.

### Audience

This tool is meant for robotics researchers who wish to do real-time robot state
estimation and control within the Orocos real-time toolkit (RTT). While this is a
ROS-independent framework, tools for ROS integration are provided in the
**conman_ros** package. 

### Scope

*Conman* not trying to solve the world. *Conman* is just trying to get rich
quick. As such, it is designed with the following goals:

* Provide a common interface for running Orocos components for robot state
  estimation and control.
* Provide a more-constrained modeling paradigm for robot state estimation and
  control.
* Handle the computational scheduling of these blocks.
* Provide a special component, not a special framework.
* Enable external manipulation of the running set of estimators and controllers
  so that single blocks and groups of blocks can be enabled and disabled at
  runtime.

### Approach

*Conman* is primarily concerned with two features of estimation and control
system implementation:

* **Execution:** Correctly serialized computation of estimation and control
  blocks
* **Metadata:** Defining a set of standard Orocos/RTT interfaces for
  communicating between such blocks.

#### Schemes: Serialized Control, Hardware, and Estimation Model Execution 

A *Conman* "scheme" models the data flow of and execution constraints on a set
of RTT components connected by data flow ports. The scheme is an RTT component,
itself, and when executed at runtime, it serially executes a subset of its peers
according to their input and output data flow ports. 

The execution schedule guarantees that all inputs to a given component are
generated before said component, and each component is executed in a single
update of the scheme. This schedule corresponds to a topological sort of the
data flow graph. As such, cycles in data flow need to be explicitly broken
through "latching." Latching the data connections from component *A* to
component *B* means that the data that component *B* reads at cycle *(k)* will be
the data that component *A* wrote at cycle *(k-1)*.

#### Scheme Internals

Internally, the scheme is represented by three graphs:

1. **Data Flow Graph** (DFG): An exact directed graph model of the data flow
   between the scheme members. Each vertex represents an RTT task, and each
   arc represets a set of data flow connections between two tasks.
   This graph may be cyclic.
2. **Execution Scheduling Graph** (ESG): The DFG with "latched" edges removed.
   This graph is always acyclic, and is used to compute the execution order of
   the scheme members via standard topological sort.
3. **Runtime Conflict Graph** (RCG): An undirected graph of members where
   adjacent members cannot be run simultaneously due to some resource conflict.

#### Scheme Construction

Components are added to and removed from a scheme procedurally. Each time a
component is added, the scheme regnerates its model of the data flow and
conflict relationships between all the scheme members.

Currently, the scheme topology can only be changed when it is not in the *Running*
state. In the future, the scheme model will be double-buffered so that a copy of
it can be modified at runtime and then swapped in real-time with the active one.

#### Edge Latching

If the addition of a component to the scheme adds a cycle to the data flow
graph, one of the connections in the cycle must be latched before the scheme can
be executed in an unambiguous order. Latches are also set procedurally.

#### Component Grouping

Components in the scheme can be grouped together under alphanumeric labes (and
groups can contain other groups). Groups are useful for starting and stopping
multiple scheme components simultaneously at runtime. The manipulation of groups
does not change the graph topology.

#### Scheme Orchestration at Runtime

Components can be started and stopped synchonously with the execution of the
entire scheme at runtime. If numerous components are specified, they are started
in topological order, and stopped in reverse-topological order.

### Designing Components for Use in Conman

Conman imposes a few constraints on the design of RTT components. For components
to be used with Conman, they need the following properties:

* The component should be compatible with the `SlaveActivity` execution pattern.
  Each component added to a Scheme is assigned a `SlaveActivity` which enables
  it to be executed in the Scheme's thread.
* The component's `startHook()`, `updateHook()`, and `stopHook()` should be
  realtime-safe and not block for long durations. Components should either
  compute their results with bounded latency, or coordinate without locking with
  a separate activity. 
* Data ports should only be written to and read from in a component's
  `startHook()` and `updateHook()`. The assumption of the sched

#### Running Components at Different Rates

The *Conman* scheme will run at a fixed rate (the rate at which you want to
control the fastest hardware in the scheme), but you may want to have components
which run slower than that. If this is the case, then they can still run at
integral multiples of the loop rate.

Each member of the scheme has a minimum execution period associated with it. By
default, this minimum execution period is 0.0 seconds. A minimum execution
period of 0.0 seconds means that the block will be executed as fast as the
scheme, itself.

#### Running Multiple Schemes

**NOTE:** _Conman v1 is only concerned with schemes that do not interact_

**BRAINSTORM** Since *Conman* schemes do not _own_ the RTT components involved
in them, it's possible to run multiple schemes either in parallel or serialized.
Of course there's nothing to worry about if the two schemes do not interact, but
if they do, you can still run them in parallel...

#### Common RTT Port Interfaces

***IN PROGRESS***

We do not want to define _yet another_ datatype or message specification.
Instead, we want to take existing datatypes (Eigen, KDL, ROS, etc) and annotate
them with relevant metadata needed to make sense of them. Essentially, we want
any given port to be self-describing. 

For example, a port in joint-space should provide both the number of degrees of
freedom of the joint group, as well as the ordered list of joint names, and a
port in cartesian-space should also provide the name of its origin frame.

##### Joint-Space Quantity
##### Cartesian-Space Quantity

## Tutorials 

### Running a Conman Scheme

**TBD**

### Writing a State Estimation Block

**TBD**

### Managing a Scheme At Runtime

**TBD**

### Running Conman In Gazebo

**TBD**

### Loading a ros\_control-based Controller Into Conman

**TBD**

### Loading a ros\_control-based Hardware Interface Into Conman

**TBD**

## Motivating Use Cases

### Teaching: My First Controller™ ###

**TBD**

### Controller Switching at Runtime

**TBD**

### Automatic State Estimation Switching at Runtime

**TBD**

### Visual Feedback for Robotic Manipulator Control

**TBD**

## Background & Prior Work

There has long been a desire to have a common platform for building and sharing
robot control "blocks." Orocos RTT, itself, aims to be such a platform, however,
with great power comes great flexibility. This means that the many ways that
Orocos RTT provides to exchange information between components and different
ways to schedule components makes it hard for developers to come to a consensus
on how and _when_ data should move around the system.

This is the primary motivation of the [Rock](http://rock-robotics.org)
framework, but this framwork aims to solve far more than this problem. One
feature of Rock, however, is a notion of component dependency management which
can allow the system to reason about which components are needed for downstream
componentns. Rock relies heavily on code generation using the Orogen tool to
generate programs from more abstract definitions, but these definitions have the
exact same semantics of Orocos RTT components.

Another project, [Ptolemy](http://ptolemy.eecs.berkeley.edu/), takes a
completely model-driven approach to designing systems. While this approach makes
formal system validation easy, it also requires that all aspects of the system
ar modeled completely. As an aside, ConMan
performs some of the same roles as a "Director" in Ptolemy. 

#### April 2012 Orocos RTT "Best Practices" Discussion

In [April 2012](http://www.orocos.org/forum/rtt/rtt-dev/ideas-about-formalizing-and-tooling-composition-primitives),
Piotr Trojanek outlined a desire for "best practices" for RTT component
composition on the RTT Developers list. While not specific to constructing a
controller manager, there are several opinions on how to model more general
systems which are relevant to this project and prior work. The contributors in
this discussion made the following points:

* **Piotr Trojanek**
  * A lot of what's needed to formalize/standardize composition might already be
    in Rock, but Rock adds even more ways to specify component interactions.
  * Some of Rock's design paradigms can be considered analogous to
    object-oriented design (templates <-> specializations, inheritance <->
    refinement, etc). This is useful because these analogies can be used to
    communicate the semantics of some of the composition primitives.
  * SIMULINK-like composition is also desirable, with graphical interfaces.
  * It's unclear how sequencer/execuation frameworks (TDL/PLEXIL) fit into
    component-based design, but DSMLs can be useful.
  * There is a lot of nomenclature which needs to be better defined. Some of these have been defined by Szyperski, but also [OMG](http://www.omg.org/spec/RTC/1.0/) has formalized a lot of compositional design patterns.
  * DAGs and Polytrees yeild obvious serialization, so they are useful
    constraints for modelling.
  * At the time of writing, it is not possible to use a single cosntraint
    language across the entire toolchain.
  * OO-based diagrams can't capture everything needed for component-based
    design.

* **Herman Bruyninckx**
  * Essential difference between component-based and object-oriented design [link (PDF)](http://atlanmod.emn.fr/www/papers/OnTheUnificationPowerOfModels.pdf) and this is clarified further by [link (PDF)](https://oxygen.informatik.tu-cottbus.de/IT/Research/AssmannZW06.pdf).
  * Operations are needed on the modelling level.
  * Making Rock's meta-model explicit could be useful, even as a learning
    experience.
  * "Port-renaming" is an important compositional tool.
  * There is no apparent reason to limit the topology of the computational
    structures to DAGs or Polytrees; the "serialization" can be done on graphs
    via (possibly multiple alternative) scheduling data structures at the
    composition level. Each of these will superimpose a DAG/Polytree on top of
    the graph topology.
  * Schedulers are an important facet of the computational graph model.
  * One of the big lack of Piotr's originally proposed graphical modelling
    tools is a set of formal constraint languages.
  * Execution frameworks don't really fit into the "component model" but they
    still need to be modelled.
  * The MDE M3-M0 modeling paradigm was incorporated into the BRICS Component Model. [link (PDF)](http://www.clawar.org/downloads/Robot_Modularity/presentations/4.%20BRICS%20Component%20Model-Bruyninckx-20130509.pdf) [link (PDF)](http://lucagherardi.it/wp-content/papercite-data/pdf/brics2013component.pdf).

* **Geoffery Biggs**
   * OpenRTM supports forming and breaking compositions at runtime without
     affecting the lifetime of the components

* **Nico Hochgeschwnder**
   * It's not so bad to have more than one constraint language, it depends on
     what we want to do with the constraints.

* **Sylvain Joyeux**
  * Operatins are not used or needed on the modelling level in Rock, so they are
    not promoted.
  * As in OpenRTM, compositions can change at rutime without affecting component
    lifecycle.

#### January 2013 Joint Controller Manager Discussion

In [January 2013](http://www.orocos.org/forum/orocos/orocos-users/joint-controller-manager),
Adolfo Rodríguez Tsouroukdissian initiated a more recent discussion, inspired by
the PR2 controller manager on the Orocos Users list. The contributors in this
discussion made the following pouints:

* **Adolfo Rodríguez Tsouroukdissian**
  * This proposal has the same goals as the [ros\_control framework](http://www.github.com/ros-controls/ros_control)
  * Provided an example data flow diagram [link (PDF)](http://www.orocos.org/files/joint_controller_manager.pdf)
  * Proposed a joint-level controller manager, motivated by the design of the
    PR2 controller manager
  * It's important to not require a single controller provide the full pipeline
    of computation from ROS command interfaces to hardware joint-level
    interfaces
  * The time reference should be external to the computational blocks
  * Hardware transmission transformation from joint-space to actuator-space and
    back should be handled explicitly.
  * It would be great to be able to split up the JointTrajectoryController to
    output something other than an effort command, and instead feed its output
    into a lower-level controller.
  * I picture the controller manager as being mostly single-threaded, where
    clients are explicitly serialized in the manager's update cycle (using slave
    activities in Orocos speak).
  * An example of where I'd like to have a separate thread is a joint_states
    publisher, a sink-only client that publishes information like joint
    positions and velocities at a lower frequency.  So, having most clients
    serialized for performance, and a few exceptions spinning separate threads
    seems reasonable to me. I'm open to alternative solutions, though.  I
    dislike bringing down the update frequency of some module with code like:

      ```cpp
      void update() {
          if (count % 10 == 0) { // Eyes start bleeding
            // Do stuff
          }
          ++count;
      }
      ```
  * The solution proposed in [1] (see original post for link) uses a plugin
    mechanism for implementing controllers, which enforces single-threaded
    execution and passing data by pointers. It just does not allow the use case
    of clients running with different update policies (lower frequency,
    non-periodic triggering). How would you go about this?
  * We want to be able to load controllers as plugins, and serialize their
    execution in a single thread.
  * We want to create controllers with arbitrary hardware interfaces.
  * We should trigger each controller with a dedicated, configurable timer event
    to allow each controller to have its own update rate. This timer event
    should be compatible with deployment in hard-realtime and simulated clock
    contexts.
  * I'd stick with the current controller\_manager design and not deal with
    clock/timer issues, but rather push this responsibility downstream. It's not
    trivial to provide a solution that works well across different (realtime)
    OS's and simulation environments
  * I'll detail a bit more (my view of) the timer approach suggested by Herman.
    Consider a system with two Orocos components (two threads):

    * The controller\_manager, whose thread is non-periodic, and has n
      event-triggered input ports. When a port is triggered, it executes some
      work (update a controller, read hardware ,etc.). Without external
      triggers, the controller_manager component does nothing.
    * An OCL [Timer
      component](http://www.orocos.org/stable/documentation/ocl/v2.x/api/html/classOCL_1_1TimerComponent.html)
      configured with n timers which trigger the ports of the
      controller_manager. Note: You could also setup non-periodic update
      policies here if it made sense to your application.
    * An important thing to note with this setup is that although triggers come
      from outside the controller_manager, the actual work is executed _in_ the
      controller_manager thread, so no concurrency handling is required. I
      overlooked this benefit in my previous post (!!!). Relating to Jonathans's
      question on parallelism: there is none here, work is still done
      sequentially inside the controller_manager.

* **Marcus Liebhardt**
  * It would be good to be able to load transmission-computation plugins just
    like controller plugins.

* **Sachin Chitta**
  * The main goal of the **ros\_control** framework is to set upa robot-agnostic
    version of the PR2 controlle framework (see below)
  * It is possible for to implement a **ros\_control**-based controller which
    implements multiple control interfaces
  * 
* **Shaun Edwards**
  * The **ros\_control** framework and Adolfo's architectures both rely on [actionlib](http://www.ros.org/wiki/actionlib)-based interfaces, but it's very important to have streaming-type interfaces for visual servoing and teleoperation applications.

* **J.D. Yamokoski**
  * Mapping raises concerns due to the overhead of hash tables and string
    comparisons
  * An impressive characterization of space descriptions is given by [OMPL spaces](http://ompl.kavrakilab.org/spaces.html)

* **Herman Bruyninckx**
  * It is important to _first_ agree on the "meta model" of all these things
    before spending effort on an _implementation_. This is the only approach for
    long-term viability. Without a clear _computer-verifiable_ model you end up
    with undebuggable systems.
  * The mainstream software development in robotics is all about writing
    software libraries with C++ code, while quite some other successful domains
    "out there" don't write code, but generate it from models. Especially in the
    context of this message: industrial control practice uses Simulink, 20Sim,
    LabView or Modelica _models_, and _tools_ to generate the code. This helps a
    lot in avoiding the problem of hand-writing APIs that support _all possible_
    relevant combinations of robot control capabilities; the latter is just not
    maintainable. (I see the same problem occurring in our KDL library, in the
    context of kinematics and dynamics algorithms.)
  * In the "Model Driven Engineering" approach it is; in the "class library API"
    world it is a lot more difficult. Your "configuration file" is basically a
    "model in disguise" :-) So, it makes more sense to make that model explicit,
    and agree on that first.
  * In many orocos applications that support motion control, people have made
    the error to deploy the kind of architecture that you have in your drawing
    ("sinks" and "sources" connected via "topic" data flows) one on one on an
    Orocos "TaskContext" component design, which is _very_ inefficient.  Since
    ages already, industry deploys such architectures into one single thread or
    process, as different functions that access the "topics" as shared memory;
    this is alot more efficient, especially since the computations in the
    "components" are very simple, but a lot of data has to be streamed around
    all the time. In addition, the Simulink, Modelica or 20Sim tools do the code
    generation from your kind of "model" to such single-thread computations for
    you.
  * My summary: the ROS/Orocos worlds are not providing the right tools,
    concepts and primitives for doing efficient and advanced (realtime) motion
    control for robots.
  * I am suggestion a design in which the "manager" is doing the "Coordination &
    Configuration" of several new complementary/decoupled responsibilities:
    - timing port
    - time interrupt handling: input to schedule function and to execution function
    - scheduling function to compute the to-be-serially-executed computations
    - execution of these computations
    - shared data resource management (e.g., via an "immutable data" policy).  This
      includes adding computations to the schedule for logging etc.
  * Moving all timing out of code has been industry standard for years already.
  * The common thing behind all these use cases are the things I mentioned:
    separating the schedule from the model and the execution; generate code from
    a domain-independent 'template implementation' with domain-dependent plug-in
    functions; (immutable) data management; deployment onto various software
    "containers".
  * Rock is one of the most rock-solid tool chains for _component_ based
    programming but not for _computational_ models. In other words, I do not see
    it play in the league of the tools I mentioned. And that is not a criticism,
    because it is not designed to be a computational toolchain that optimizes
    efficiency and code generation to the "bare metal". :-)
  * The logical next step is to focus first on the semantics, hence a "model".
    Unfortunately, this is not a common reflex in the ROS/Orocos universe. But
    things are slowly changing; this concrete topic could be a good start for a
    more visible effort in this direction.

* **Wim Meeussen**
  * We pass in two times into the update method to deal with the difference
    between the system clock and the monotonic realtime clock.  The first
    argument is the (estimated) non-monotonic system time, which the controller
    can use to stamp data it sends out to other components, and to compare with
    timestamps of incoming data from other components.  But since the system
    time is non-monotonic (it can be modified by e.g. an ntp daemon), it can't
    be used to compute e.g. the time difference between different update cycles.
    So therefore the controller also need access to a monotonic clock. To make
    it very difficult to mix up the system clock and the monotonic realtime
    clock, we pass the system time in as a ros::Time, and the monotonic realtime
    time as a ros::Duration -- which maps on two common uses of the two clocks:
    stamping data and computing the time difference between two update cycles.

#### Other Discussions

http://www.orocos.org/forum/orocos/orocos-users/about-time-through-cascade-components
http://www.orocos.org/node/1088
http://www.orocos.org/node/1050

#### Early 2013 ros\_control Framework
 
In Spring and Summer 2013, the [ros\_control framework](http://www.github.com/ros-controls/ros_control) 
attracted a lot of attention. This framework was ported from the PR2 controller
framework, and as such, its design is best suited for a fully-integrated robotic
platform. **ros\_control** assumes that there is a single atomic robot
hardware interface class which owns several types of interfaces (state, effort,
velocity, etc). One strange effect of this is that to get the joint state out of this
framework, you write a "controller" which only reads the state, but doesn't
control anything. There is just one level of dynamically-loadable plugins, and
they cannot be pipelined or easily composed.


 *
 * ConMan is a framework for building real-time-safe state estimators and
 * controllers with Orocos RTT 
 *
 * Dataflow
 *  Conman dataflow interfaces are normal Orocos RTT ports, except the ports are
 *  connected with a publish/subscribe paradigm.
 *  We need to associate additional metadata with the conman ports, however, to
 *
 *  satisfy the following requirements:
 *    - Determine the exclusivity of a port (one or many connections)
 *    - Determine if a port is an input or an output (or we could just try to
 *      over-connect)
 *
 *  Conman places no hard requirements on the names of ports, but instead we
 *  standardize on a set of conventions:
 *    * Hard standardization on datatypes (decided at build time)
 *    * Soft standardization on port names (decided at build or runtime)
 *  
 *  Standard convention is to remap port names from block-relative naming to
 *  runtime-relaative naming. For example, there might be several blocks with
 *  <JointArrayAcc> ports performing joint-level state estimation. In this case,
 *  each block might have "joint_state_unfiltered" and "joint_state_filtered"
 *  ports. These blocks should be able to be remapped in a standard way to allow
 *  pipelining of state estimation filters.
 *
 *  .estimation.left_arm.visual_pose
 *  .control.left_arm.effort_command
 *   --> ekf -->
 *  .estimation.left_arm.joint_state_filtered  
 *
 *  TODO: check ports after each call to make sure they aren't being written to
 *  outside of the appropriate compute-control or compute-estimation hooks
 *  
 * Resources
 *  RTT Ports are the only resources in conman. Reading resources is
 *  unrestricted, but writing to a resource can be controlled. Access is
 *  controlled when the Scheme enables and disables various control
 *  components, and not when it sets up the RTT Port network. This means that
 *  RTT Ports may be connected in such a way that violates the maximum numvber
 *  of connections.  
