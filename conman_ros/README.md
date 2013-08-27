Conman ROS Interfaces
=====================

This package provides interfaces to interact with Conman Schemes over ROS. These
interfaces include an RTT service which provides the following:

* Connection of Scheme operations to ROS services
* A ROS topic with status information
* A ros\_control-compatible controller switching API

Note that individual blocks in the scheme may have their own ROS interfaces in
addition to the one for the scheme.

### Conman ROS API

***TBD***

### Conman ros\_control-Compatible ROS API

Conman also provides an API using the same messages and services used by the
ros\_control controller manager.

#### Topics

* `controllers_statisics` `controller_manager_msgs/ControllersStatistics`
  * *This contains the same information as it does in ros\_control, but for each
    block in the Schame.*
* `controller_state``controller_manager_msgs/ControllerState`
  * *This contains only the "name", "state", and "type" values for each block in
    the Scheme.*

#### Services

Only the services which are required for starting and stopping controllers, and
adding and removing them from the system. The services which are specific to the
plugin interface for ros\_control are not provided. For loading libraries, it is
preferred that Orocos Ops script be used. Note that while ros\_control allows
loading of plugins via ROS services, the mappings between controller names and
the controller types must still be put on the parameter server somehow.

* `list_controllers` `controller_manager_msgs/ListControllers`
  * *List all block names.*
* `list_controller_types` `controller_manager_msgs/ListControllerTypes`
  * ***UNIMPLEMENTED*** 
  * This is specific to the ros\_control plugin manager.
* `switch_controller` `controller_manager_msgs/SwitchController`
  * *Maps directly to Scheme::switchBlocks with controller names correspond to
    block names.*
* `load_controller` `controller_manager_msgs/LoadController`
  * *Adds a peer of the Scheme to the Scheme by name. This is equivalent to
    calling Scheme::addBlock with a block name.*
* `reload_controller_libraries` `controller_manager_msgs/ReloadControllerLibraries`
  * ***UNIMPLEMENTED*** 
  * This is specific to the ros\_control plugin manager.
* `unload_controller` `controller_manager_msgs/UnloadController`
  * *Removes a block from the Scheme by name. This is equivalent to calling
    Scheme::removeBlock with a block name.*

