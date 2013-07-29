Conman
======

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

* Provide a common interface for running Orocos components for robot state estimation and control.
* Handle the computational scheduling of these blocks.
* Provide a special component, not a special framework.
* Enable external manipulation of the running set of estimators and controllers.

### Approach

*Conman* is primarily concerned with two features of estimation and control
system implementation:

* **Execution:** Correctly serialized computation of estimation and control
  blocks
* **Metadata:** Defining a set of standard Orocos/RTT interfaces for
  communicating between such blocks.

#### Schemes: Serialized Control, Hardware, and Estimation Model Execution 

A *Conman* "scheme" is a set of RTT components, and a pair of directed acyclic
computational graphs on these components and their RTT ports: the **estimation
graph** and the **control graph**.  These graphs are meant to be computed
topologically in realtime, so their respective parts should either compute in a
separate thread and exchange information, or should compute with bounded latency.

These two graphs may share vertices (RTT components), but are distinguished by
their arcs (relevant RTT ports). 

Each scheme update involves the following stages, executed in a single thread:

1. Read from all hardware (sensors)
2. Compute **estimation graph**
3. Compute **control graph**
4. Write to all hardware (actuators)

These stages are supported by the corresponding *Conman* RTT operations which
**should execute quickly**:

1. **readHardware()**
  * Reads lower-level hardware APIs or external interfaces (like ROS topics)
2. **computeEstimation()**
  * Computes state estimation based on RTT port inputs
  * Writes estimated state to RTT ports modeled by edges in the **estimation graph**
3. **computeControl()**
  * Computes control command based on RTT port inputs
  * Writes control command to RTT ports modeled by edges in the **control graph**
4. **writeHardware()**
  * Writes to lower-level hardware APIs or external interfaces

#### Common RTT Port Interfaces

We do not want to define _yet another_ datatype or message specification.
Instead, we want to take existing datatypes (Eigen, KDL, ROS, etc) and annotate
them with relevant metadata needed to make sense of them. Essentially, we want
any given port to be self-describing. 

For example, a port in joint-space should provide both the number of degrees of
freedom of the joint group, as well as the ordered list of joint names, and a
port in cartesian-space should also provide the name of its origin frame.

##### Joint-Space Quantity
##### Cartesian-Space Quantity

### Designing Components for Use in Conman

#### Running Components at Different Rates

The *Conman* scheme will run at a fixed rate (the rate at which you want to
control the fastest hardware in the scheme), but you may want to have components
which run slower than that. If this is the case, then they can still run at
integral multiples of the loop rate, simply by checking the time at which they
are invoked.

#### Running Multiple Schemes

**NOTE:** _Conman v1 is only concerned with schemes that do not interact_

**BRAINSTORM** Since *Conman* schemes do not _own_ the RTT components involved
in them, it's possible to run multiple schemes either in parallel or serialized.
Of course there's nothing to worry about if the two schemes do not interact, but
if they do, you can still run them in parallel...

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
on how data should move around the system.

This is the primary motivation of the [Rock](http://rock-robotics.org)
framework, but this framwork aims to solve far more than this problem. One
feature of Rock, however, is a notion of component dependency management which
can allow the system to reason about which components are needed for downstream
componentns.

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
    - The controller\_manager, whose thread is non-periodic, and has n
      event-triggered input ports. When a port is triggered, it executes some
      work (update a controller, read hardware ,etc.). Without external
      triggers, the controller_manager component does nothing.
    - An OCL Timer component [1] configured with n timers which trigger the
      ports of the controller_manager. Note: You could also setup non-periodic
      update policies here if it made sense to your application.
    An important thing to note with this setup is that although triggers come
    from outside the controller_manager, the actual work is executed _in_ the
    controller_manager thread, so no concurrency handling is required. I
    overlooked this benefit in my previous post (!!!). Relating to Jonathans's
    question on parallelism: there is none here, work is still done sequentially
    inside the controller_manager.




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

* **Piotr Trojanek**
* **Sylvain Joyeux**

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

