Conman Orocos Scripting Reference
=================================

## Adding a Block to a Scheme

There are three things that are needed to add a component to a conman scheme. In order to be part of a scheme, a block needs to be a peer of the scheme, slaved to the scheme's activity, have the conman_hook service, and then finally it can be added to the scheme's graph:

```
addPeer("scheme","my_block");
setMasterSlaveActivity("scheme","my_block");
loadService("my_block","conman_hook");
scheme.addBlock("my_block");
```

However, the conman_hook can be loaded automatically and the block's activity can be slaved to the scheme's activity automatically in `addBlock()` so all you need to do is:

```
addPeer("scheme","my_block");
scheme.addBlock("my_block");
```

## Configuring the Block

Then, you want to set the minimum desired period for the component:

```
my_block.conman_hook.setDesiredMinPeriod(0.0015);
```
