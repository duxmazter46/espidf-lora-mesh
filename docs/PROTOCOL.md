
# Network Operation and TDMA Protocol

The proposed protocol combines asynchronous network formation with centrally coordinated TDMA communication. The overall operation of the network is divided into four stages: **network discovery, anchor time establishment, bootstrap cycle, and runtime TDMA cycle**. This design allows nodes to join the network flexibly while enabling deterministic and collision-controlled communication during normal operation.

---

## 1. Network Discovery (Asynchronous Phase)

Initially, all nodes operate asynchronously without requiring prior synchronization. When a node powers on, it attempts to join the network by transmitting a **JOIN request** to nearby nodes. Neighboring relay nodes receive this request and forward it toward the **root node**, which maintains the global view of the network topology.

After receiving the request, the root node assigns the joining node a **network identity and a position within the mesh topology**, typically based on hop distance from the root. This information is propagated back through relay nodes to the joining node. Through this process, a hierarchical multi-hop structure is gradually established.

Because this stage operates asynchronously, nodes can join the network dynamically without requiring any predefined timing or synchronization.

---

## 2. Anchor Time Establishment

Once the network structure has been formed, the **root node establishes a global time reference called the anchor time**. This time reference serves as the basis for all subsequent TDMA scheduling.

The root periodically broadcasts **SYNC packets** containing the anchor time and timing information. Relay nodes forward these synchronization packets downstream so that all nodes in the network receive the same reference time. Each node adjusts its internal clock based on the received synchronization information, allowing the entire network to share a consistent time base.

This global synchronization enables coordinated slot scheduling during the TDMA communication phase.

---

## 3. Bootstrap Cycle

Before the network begins normal data communication, a **bootstrap cycle** is executed to distribute the TDMA schedule across the network.

During this stage, the root node generates a **TDMA table** that specifies transmission slots for relay nodes. Slot allocation follows a **topology-aware ordering**, where nodes closer to the root (with smaller hop counts) receive their scheduling information earlier in the cycle. This ensures that scheduling information can be propagated efficiently to deeper nodes in the network.

Example topology:

```
Root
 |
Node 1
 |
Node 2
```

Bootstrap cycle slot ordering:

```
| Node 1 | Node 2 | Free |
```

This ordering allows **Node 1**, which is closer to the root, to receive the TDMA schedule first and immediately forward control information to **Node 2** during its slot. Through this cascading process, the TDMA table is distributed throughout the entire network.

The bootstrap cycle therefore ensures that every node understands the **slot timing and transmission order** before normal data communication begins.

---

## 4. Runtime TDMA Cycle

After the bootstrap phase, the network transitions into the **runtime TDMA cycle**, which is used for regular data communication.

During runtime operation, communication proceeds in repeating frames divided into fixed transmission slots. Each relay node is allowed to transmit only during its assigned slot while other nodes remain silent. This controlled channel access reduces packet collisions and provides predictable communication timing.

To improve data forwarding efficiency in multi-hop topologies, the **runtime slot ordering is reversed compared to the bootstrap cycle**.

Runtime TDMA order example:

```
| Node 2 | Node 1 | Free |
```

This reversed ordering allows nodes located farther from the root (e.g., Node 2) to transmit earlier in the cycle. As a result, their data packets can propagate upward through intermediate relay nodes within the same cycle, reducing the total latency required for the packet to reach the root node.

Sensor nodes transmit data to their parent relay nodes, which then aggregate and forward the packets toward the root during their assigned slots. Because slot timing is synchronized using the anchor time, packet collisions are minimized and deterministic multi-hop communication can be achieved.
