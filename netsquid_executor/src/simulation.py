import types

from netsquid.qubits.operators import H, CNOT
from netsquid.components import Message
from netsquid.protocols import NodeProtocol
from netsquid.components.instructions import *
from netsquid.qubits import create_qubits, operate
# from qiskit import QuantumCircuit

from distributed_circuit import DistQuantumCircuit
from qiskit.circuit import Clbit
from netsquid import sim_reset, sim_run

from .network import *


class ExecutionProtocol(NodeProtocol):
    """
    The class ExecutionProtocol implements the protocol for executing the distributed computation that will be assigned
    to each node of the network.
    It stores internally information such as: the network, the circuit is has to execute, and a reference to the node itself
    Each node is given the entire circuit and will execute in parallel with the others only the instructions
    that involve him.
    """

    def __init__(self, node: Node, gate_tuples, network: NetworkWrapper):
        super().__init__(node)
        self.gate_tuples = gate_tuples  # List of gates of the algorithm
        self.node = node
        self.network = network
        self.coupling_map = self.network.coupling_map  # List of coupled ebits between QPUs

        self.instructions = {
            "h": INSTR_H,
            "y": INSTR_Y,
            "z": INSTR_Z,
            "x": INSTR_X,
            "s": INSTR_S,
            "t": INSTR_T,
            # "toffoli": netsquid.components.instructions.INSTR_TOFFOLI,
            # "swap": netsquid.components.instructions.INSTR_SWAP,
            "prep_z": INSTR_INIT,
            "cz": INSTR_CZ,
        }

        self.instr_id = -1

    def is_local_qubit(self, qubit: Qubit):
        """
        Returns True if the given qubit belongs to the node executing the protocol.

        Args:
            qubit: qubit to check if local

        Returns: True if qubit is local, False otherwise

        """
        return self.get_local_qubit_index(qubit) is not None

    def is_local_cbit(self, bit: Clbit):
        """
        Returns True if the given classical bit belongs to the node executing the protocol.

        Args:
            bit: bit to check if local

        Returns: True if bit is local, False otherwise

        """
        return self.node.name in bit._register.name

    def get_local_qubit_index(self, qubit: Qubit):
        """
        Returns the index within the local register of the specified qubit or None if it does not belong to the node
        executing the protocol

        Args:
            qubit: qubit of which to find the local index

        Returns: qubit's position inside local register or None if it is not there

        """
        if self.node.name in qubit._register.name:
            return qubit._index
        return None

    def get_local_qubit_indices(self, qubit_list: list):
        """
        Returns the indices within the local register of the specified qubits

        Args:
            qubit_list: list of qubits of which to retrieve the indices

        Returns: list of indices of the input qubits

        """
        res = []
        for qubit in qubit_list:
            index = self.get_local_qubit_index(qubit)
            if index is None:
                raise Exception("Cannot retrieve qubit indices... a qubit does not belong to the current node")
            res.append(index)
        return res

    def get_classicla_port(self, dest_node: Node):
        conn = self.network.get_classical_connection_between_nodes(self.node, dest_node)
        if conn is None:
            raise Exception(
                f"There is no classical connection between the nodes {self.node.name} and {dest_node.name}")
        return get_port(conn, self.node)

    def remote_cx(self, q0: Qubit,
                  ent0: Qubit,
                  q1: Qubit,
                  ent1: Qubit):
        """
        Performs the "Remote CNOT" operation between two QPUs by specifying first the control qubit and communication qubit
        of the "Control QPU" and then the target qubit and communication qubit of the "Target QPU".
        The specified Ebits must be entangled or an error is raised.

        The remoteCX operation is executed as:
        cx q0 [0] , ent0 [0];
        cx ent1 [0] , q1 [0];
        h ent1 [0];
        measure ent0 [0] -> ce0t1 [0];
        if ( ce0t1 ==1) x q1 [0];
        measure ent1 [0] -> ce1t0 [0];
        if ( ce1t0 ==1) z q0 [0];

        Args:
            q0: control qubit
            ent0: control communication qubit
            q1: target qubit
            ent1: target communication qubit

        """
        # let's find the two nodes of the gate
        node0 = self.network.get_owner_of_qubit(q0)
        node1 = self.network.get_owner_of_qubit(q1)
        if node0 is node1:
            raise Exception("Cannot do remote CX on the same node")

        # Qubit and Ebit must belong to the same node
        if node0 is not self.network.get_owner_of_qubit(ent0) or node1 is not self.network.get_owner_of_qubit(ent1):
            raise Exception("Malformed remote CX: make sure each node has a qubit and ebit")

        # Let's find the classical connection between the two nodes
        connection = self.network.get_classical_connection_between_nodes(node0, node1)
        if connection is None:
            raise Exception(f"There is no classical connection between the nodes {node0.name} and {node1.name}")

        # We check if this node is supposed to be the control or target
        if self.node is node0:
            # We are the control (The first node specified in the instruction is the Control)

            # We identify its endpoint within the classical communication channel
            if list(connection.ports.values())[0].connected_port.component is node0:
                port0 = list(connection.ports.values())[0].connected_port
            else:
                port0 = list(connection.ports.values())[1].connected_port

            # The control asserts that the ebits are currently entangled
            ent_index = None
            for i in range(len(self.network.entangled_qubits)):
                if [ent0, ent1] == self.network.entangled_qubits[i] or [ent1, ent0] == self.network.entangled_qubits[i]:
                    ent_index = i
                    break

            if ent_index is None:
                raise Exception("Ebits must be entangled for remote CX to work")

            # Entanglement is consumed
            self.network.entangled_qubits.pop(ent_index)

            INSTR_CX(self.node.subcomponents["main_memory"],
                     positions=[q0._index, ent0._index])

            meas = INSTR_MEASURE(self.node.subcomponents["main_memory"],
                                 positions=[ent0._index])
            # The control outputs the result of the measurement through its port
            port0.tx_output(Message(meas[0], header=f'{self.instr_id}_{self.node.name}'))
            # Waiting for the measurement of the target
            yield self.await_port_input(port0)
            mex = port0.rx_input()
            val = mex.items[0]
            if val == 1:
                INSTR_Z(self.node.subcomponents["main_memory"],
                        positions=[q0._index])
            # To synchronize the execution of these complex gates an Acknowledgment Protocol is employed.
            # The header has the purpose of allowing the selective extraction of the ack even if multiple messages are
            # ready to be read from the input queue.
            port0.tx_output(Message("ACK_RCX", header="RCX"))

        elif self.node is node1:  # The current node is the target
            # We identify its endpoint within the classical communication channel
            if list(connection.ports.values())[0].connected_port.component is node1:
                port1 = list(connection.ports.values())[0].connected_port
            else:
                port1 = list(connection.ports.values())[1].connected_port

            INSTR_CX(self.node.subcomponents["main_memory"],
                     positions=[ent1._index, q1._index])
            INSTR_H(self.node.subcomponents["main_memory"],
                    positions=[ent1._index])
            # Waits for the control's measurement
            yield self.await_port_input(port1)

            mex = port1.rx_input(header=f'{self.instr_id}_{node0.name}')
            val = mex.items[0]
            if val == 1:
                INSTR_X(self.node.subcomponents["main_memory"],
                        positions=[q1._index])
            meas = INSTR_MEASURE(self.node.subcomponents["main_memory"],
                                 positions=[ent1._index])
            # Sends its measurement to the target
            port1.tx_output(meas[0])
            # Waits for the acknowledgement from the target before proceeding
            yield self.await_port_input(port1)
            mex = port1.rx_input(header="RCX")
            if mex.items[0] != "ACK_RCX":
                raise Exception("Could not synchronize nodes")
        else:
            raise Exception("Invalid parameters in Remote CNOT gate")

    def ent_swap(self, qubits):
        node0 = self.network.get_owner_of_qubit(qubits[0])
        node1 = self.network.get_owner_of_qubit(qubits[-1])
        n_brokers = (len(qubits) - 2) / 2

        if n_brokers - int(n_brokers) != 0.0:
            raise Exception(
                f"[{self.node.name}] Got {qubits} ebits, from which the number of intermediaries nodes is {n_brokers}.")
        else:
            n_brokers = int(n_brokers)
        brokers = {self.network.get_owner_of_qubit(qubits[1 + 2 * b]).name:
                       (self.network.get_owner_of_qubit(qubits[2 * b]).name,
                        [qubits[1 + 2 * b], qubits[2 + 2 * b]],
                        self.network.get_owner_of_qubit(qubits[3 + 2 * b]).name) for b in range(n_brokers)}

        if self.node == node0:

            port = self.get_classicla_port(self.network.get_owner_of_qubit(qubits[1]))
            yield self.await_port_input(port)
            mex = port.rx_input(header=self.network.get_owner_of_qubit(qubits[1]).name)

            if mex.items[0] != 'READY':
                raise Exception(f'[{self.node.name}] Error while synchronizing with broker node {self.network.get_owner_of_qubit(qubits[1]).name}')

            dest_port = self.get_classicla_port(node1)
            for broker in brokers:
                port = self.get_classicla_port(self.network.get_node(broker))

                dest_port.tx_output(Message('READY', header=self.node.name))

                yield self.await_port_input(port)
                mex = port.rx_input(header=broker)
                if mex.items[0] == 1:
                    INSTR_Z(self.node.subcomponents["main_memory"],
                                    positions=[qubits[0]._index])

                yield self.await_port_input(dest_port)
                mex = dest_port.rx_input(header=node1.name)
                if mex.items[0] != 'DONE':
                    raise Exception(
                        f'[{self.node.name}] Error while synchronizing with broker node {self.network.get_owner_of_qubit(qubits[1]).name}')

        elif self.node == node1:
            source_port = self.get_classicla_port(node0)
            for broker in brokers:
                port = self.get_classicla_port(self.network.get_node(broker))
                yield self.await_port_input(source_port)
                mex = source_port.rx_input(header=node0.name)
                if mex.items[0] != 'READY':
                    raise Exception(f'[{self.node.name}] Error while synchronizing with broker node {node0.name}')
                port.tx_output(Message('WAITING', header=self.node.name))

                yield self.await_port_input(port)
                mex = port.rx_input(header=broker)
                if mex.items[0] == 1:
                    INSTR_X(self.node.subcomponents["main_memory"],
                                    positions=[qubits[-1]._index])

                source_port.tx_output(Message('DONE', header=self.node.name))

            self.network.entangled_qubits.append([qubits[0],qubits[-1]])
        else:
            prev, ebits, succ = brokers[self.node.name]
            e0, e1 = ebits

            source_port = self.get_classicla_port(node0)
            dest_port = self.get_classicla_port(node1)

            port0 = self.get_classicla_port(self.network.get_node(prev))
            port1 = self.get_classicla_port(self.network.get_node(succ))

            if prev == node0.name:
                ent0 = qubits[0]
            else:
                ent0 = brokers[prev][1][1]
            ent_index0 = None
            for i in range(len(self.network.entangled_qubits)):
                if [ent0, e0] == self.network.entangled_qubits[i] or [e0, ent0] == self.network.entangled_qubits[i]:
                    ent_index0 = i
                    epr0 = self.network.entangled_qubits[i]
                    break
            if ent_index0 is None:
                raise Exception(f"[{self.node.name}] Ebits must be entangled for remote CX to work")

            if succ == node1.name:
                ent1 = qubits[-1]
            else:
                ent1 = brokers[succ][1][0]
            ent_index1 = None
            for i in range(len(self.network.entangled_qubits)):
                if [e1, ent1] == self.network.entangled_qubits[i] or [ent1, e1] == self.network.entangled_qubits[i]:
                    ent_index1 = i
                    epr1 = self.network.entangled_qubits[i]
                    break
            if ent_index1 is None:
                raise Exception(f"[{self.node.name}] Ebits must be entangled for remote CX to work")

            INSTR_CX(self.node.subcomponents["main_memory"],
                     positions=[e0._index, e1._index])
            INSTR_H(self.node.subcomponents["main_memory"],
                    positions=[e0._index])

            if prev != node0.name:
                port0.tx_output(Message('DONE', header=self.node.name))

            measure0 = INSTR_MEASURE(self.node.subcomponents["main_memory"],
                                    positions=[e0._index])

            if succ != node1.name:
                yield self.await_port_input(port1)
                mex = port1.rx_input(header=succ)
                if mex.items[0] != 'DONE':
                    raise Exception(f'[{self.node.name}] Error while synchronizing with broker node {succ}')

            measure1 = INSTR_MEASURE(self.node.subcomponents["main_memory"],
                                    positions=[e1._index])

            if n_brokers == 1:
                source_port.tx_output(Message('READY', header=self.node.name))
            elif self.node.name == self.network.get_owner_of_qubit(qubits[1]).name:
                last = self.network.get_owner_of_qubit(qubits[-2])
                port = self.get_classicla_port(last)
                yield self.await_port_input(port)
                mex = port.rx_input(header=last.name+'_READY')
                if mex.items[0] != 'READY':
                    raise Exception(f'[{self.node.name}] Error while synchronizing with broker node {last.name}')
                source_port.tx_output(Message('READY', header=self.node.name))
            elif self.node.name == self.network.get_owner_of_qubit(qubits[-2]).name:
                first = self.network.get_owner_of_qubit(qubits[1])
                port = self.get_classicla_port(first)
                port.tx_output(Message('READY', header=self.node.name+'_READY'))

            yield self.await_port_input(dest_port)
            mex = dest_port.rx_input(header=node1.name)
            if mex.items[0] != 'WAITING':
                raise Exception(f'[{self.node.name}] Error while synchronizing with broker node {node0.name}')

            source_port.tx_output(Message(measure0[0], header=self.node.name))
            dest_port.tx_output(Message(measure1[0], header=self.node.name))

            # Remove used epr pairs from entangled list
            if epr0 in self.network.entangled_qubits:
                self.network.entangled_qubits.remove(epr0)
            if epr1 in self.network.entangled_qubits:
                self.network.entangled_qubits.remove(epr1)


    # AT SOURCE github.com/Wojtek242/draft-irtf-qirg-principles/blob/master/draft-irtf-qirg-principles-07.txt (line 672)
    def entangle_qubits(self, q0: Qubit,
                        q1: Qubit):
        """
        Allows for local as well as "AT SOURCE" entanglement of two qubits:
        - If the two qubits are local to this Node, they are entangled.
        - If the two qubits are of different Nodes, they are entangled only if coupled.

        Args:
            q0: first qubit to be entangled
            q1: second qubit to be entangled
        """
        if q0 == q1:
            raise Exception("Cannot entangle a qubit with itself")

        node0 = self.network.get_owner_of_qubit(q0)
        node1 = self.network.get_owner_of_qubit(q1)
        # Local entanglement
        if node0 is node1:
            ent_q1, ent_q2 = create_qubits(2)
            operate(ent_q1, H)
            operate([ent_q1, ent_q2], CNOT)
            self.node.subcomponents["main_memory"].put(ent_q1, q0._index)
            self.node.subcomponents["main_memory"].put(ent_q2, q1._index)
        else:
            # We retrieve the quantum connection between the QPUs
            quantum_connection = self.network.get_quantum_connection_between_nodes(node0, node1)
            if quantum_connection is None:
                raise Exception(f"There is no quantum connection between the nodes {node0.name} and {node1.name}")

            classical_connection = self.network.get_classical_connection_between_nodes(node0, node1)
            if classical_connection is None:
                raise Exception(f"There is no classical connection between the nodes {node0.name} and {node1.name}")

            # Checks the ebits are coupled before allowing the entanglement
            found = False
            for entry in self.coupling_map:
                if (node0.name == entry[0].name and q0._index == entry[0].index) or \
                        (node0.name == entry[1].name and q0._index == entry[1].index):
                    if (node1.name == entry[0].name and q1._index == entry[0].index) or \
                            (node1.name == entry[1].name and q1._index == entry[1].index):
                        found = True

            if not found:
                raise Exception("Cannot execute entanglement between uncoupled ebits")

            if self.is_local_qubit(q0):
                # The entanglement generator asserts the ebits have not already been entangled with other ebits
                for entangled_pair in self.network.entangled_qubits:
                    if q0 in entangled_pair or q1 in entangled_pair:
                        raise Exception("One of the ebits is already entangled with another one")

                # We identify its endpoint within the quantum communication channel
                if list(quantum_connection.ports.values())[0].connected_port.component is node0:
                    port0 = list(quantum_connection.ports.values())[0].connected_port
                else:
                    port0 = list(quantum_connection.ports.values())[1].connected_port

                # We identify its endpoint within the classical communication channel
                if list(classical_connection.ports.values())[0].connected_port.component is node0:
                    port0_c = list(classical_connection.ports.values())[0].connected_port
                else:
                    port0_c = list(classical_connection.ports.values())[1].connected_port

                # The first nodes created the entangled pair and distributes it
                ent_q0, ent_q1 = create_qubits(2)
                operate(ent_q0, H)
                operate([ent_q0, ent_q1], CNOT)
                self.node.subcomponents["main_memory"].put(ent_q0, q0._index)
                # Saves in the global list of entangled pairs the current ones
                self.network.entangled_qubits.append([q0, q1])
                port0.tx_output(ent_q1)
                yield self.await_port_input(port0_c)
                # To synchronize the execution of these complex gates an Acknowledgment Protocol is employed.
                # The header has the purpose of allowing the selective extraction of the ack even if multiple messages
                # are ready to be read from the input queue.
                mex = port0_c.rx_input(header="ENT")
                if mex.items[0] != "ACK_ENT":
                    raise Exception("Could not synchronize entanglement")
            else:
                # We identify its endpoint within the quantum communication channel
                if list(quantum_connection.ports.values())[0].connected_port.component is node1:
                    port1 = list(quantum_connection.ports.values())[0].connected_port
                else:
                    port1 = list(quantum_connection.ports.values())[1].connected_port

                # We identify its endpoint within the classical communication channel
                if list(classical_connection.ports.values())[0].connected_port.component is node1:
                    port1_c = list(classical_connection.ports.values())[0].connected_port
                else:
                    port1_c = list(classical_connection.ports.values())[1].connected_port

                # Waits for the entangled qubit coming from the first node
                yield self.await_port_input(port1)
                mex = port1.rx_input()
                ent_q1 = mex.items[0]
                self.node.subcomponents["main_memory"].put(ent_q1, q1._index)
                # Sends acknowledgement
                port1_c.tx_output(Message("ACK_ENT", header="ENT"))

    def condition_passed(self, gate_tuple, condition):
        """
        Checks whether the gate is controlled and, in such case, if the condition is satisfied.

        Args:
            gate_tuple: instruction to be checked

        Returns: True if the node can execute this instruction, False if a condition prevents it.

        """
        gate, qubits, bits = gate_tuple

        if gate.condition is None:
            condition.passed = True
        else:
            # Checks if the name of the current node appears in the condition register and if such register is equal to
            # the value which satisfies the condition.
            if gate.name.lower() in ["remotecx", "entangle", "epr"]:
                raise Exception("Controlled remoteCNOT and controlled entanglement are not supported")
            if self.node.name in gate.condition[0].name:
                condition.passed = self.node.register_equal(gate.condition[1])
            else:
                yield from self.remote_condition_passed(gate, condition)

    def remote_condition_passed(self, gate, condition):
        val = gate.condition[1]
        if val >= 2 ** gate.condition[0].size:
            raise Exception("Register Overflow in conditional gate")
        control = self.network.get_node(gate.condition[0].name.split('_')[0])
        connection = self.network.get_classical_connection_between_nodes(self.node, control)
        # We identify its endpoint within the classical communication channel
        if list(connection.ports.values())[0].connected_port.component is self.node:
            port1 = list(connection.ports.values())[0].connected_port
        else:
            port1 = list(connection.ports.values())[1].connected_port

        yield self.await_port_input(port1)
        mex = port1.rx_input(header='{}'.format(self.instr_id))
        bin_val = mex.items[0]
        if len(bin_val) != gate.condition[0].size:
            raise Exception("Incomplete binary value for classical remote conditioned operation.")
        res = format(val, "b")
        while len(res) < gate.condition[0].size:
            res = '0{}'.format(res)
        if res == bin_val:
            condition.passed = True
        else:
            condition.passed = False

    def measure(self, qubit: Qubit, cbit: Clbit):
        """
        Performs a local measurement and saves the result in the specified bit of the register.
        The specified qubit and classical bit must be of the same QPU.
        Args:
            qubit: qubit to be measured
            cbit: classical bit in which to store the result

        """
        res = INSTR_MEASURE(self.node.subcomponents["main_memory"],
                            positions=[qubit._index])[0]
        self.node.classical_register[cbit._index] = res
        # print(f"{self.node.name} measured: {res}")

    def local_cx(self, control: Qubit, target: Qubit):
        """
        Performs a local cnot between the specified qubits only if the topology of the current node allows it.
        The specified qubits must belong to the same QPU.
        Args:
            control: local control qubit
            target: local target qubit

        """
        if control == target:
            raise Exception("CNOT is a two qubit gate... cannot execute on a single qubit")
        if self.node.topology is None or (control._index, target._index) in self.node.topology:
            INSTR_CNOT(self.node.subcomponents["main_memory"],
                       positions=self.get_local_qubit_indices([control, target]))
        else:
            raise Exception(f"Invalid CNOT with the given coupling map of node {self.node.name}")

    def execute_instruction(self, gate_tuple: tuple):
        """
        Executes the specified instruction on the current Node.

        Args:
            gate_tuple: instruction to be executed

        """
        gate, qubits, bits = gate_tuple
        condition = types.SimpleNamespace(passed=False)
        yield from self.condition_passed(gate_tuple, condition)
        if condition.passed:
            if gate.name.lower() == "entangle" or gate.name.lower() == "epr":
                yield from self.entangle_qubits(*qubits)
            elif gate.name.lower() == "entswap":
                yield from self.ent_swap(qubits)
                # exit(-1)
            elif gate.name.lower() == "remotecx":
                yield from self.remote_cx(*qubits)
            elif gate.name.lower() == "measure":
                self.measure(*qubits, *bits)
            elif gate.name.lower() == "cx":
                self.local_cx(*qubits)
            else:
                # Calls the specific instruction within the Table specified in the constructor.
                try:
                    self.instructions[gate.name.lower().strip()](self.node.subcomponents["main_memory"],
                                                                 positions=self.get_local_qubit_indices(qubits))
                except KeyError:
                    raise Exception(f"Instruction {gate.name} has not yet been implemented")
        del condition

    def can_execute(self, gate_tuple: tuple):
        """
        Checks if the current Node is allowed to execute the specified instruction. The instruction must make use of at
        least on qubit pertaining to the current Node. For remote instructions such as entanglement or remoteCX, both
        parties execute the instruction but will behave differently according to the inner implementation.
        (See remote_cx and entangle_qubits)

        Args:
            gate_tuple: instruction to be executed

        """
        gate, qubits, bits = gate_tuple
        one_local = False
        # We check if there is at least one local qubit.
        for qubit in qubits:
            if self.is_local_qubit(qubit):
                one_local = True
                break
        if not one_local:
            if gate.condition and self.node.name in gate.condition[0].name:
                self.cl_remote_control(gate_tuple)
            return False

        # RemoteCX and Entanglement are executed by both parties.
        if gate.name.lower() in ["remotecx", "entangle", "entswap", "epr"]:
            return True
        elif gate.name.lower() == "measure":
            if self.is_local_qubit(*qubits) and self.is_local_cbit(*bits):
                return True
            else:
                raise Exception("Remote measurements are not supported yet")
        else:
            # For normal instructions it checks that all qubits belong to the current Node.
            for qubit in qubits:
                if not self.is_local_qubit(qubit):
                    raise Exception(f"Invalid qubits were specified in the gate {gate.name}")
            return True

    def cl_remote_control(self, gate_tuple):
        gate, qubits, bits = gate_tuple
        node1 = self.network.get_node(qubits[0].register.name.split('_')[0])
        connection = self.network.get_classical_connection_between_nodes(self.node, node1)
        if connection is None:
            raise Exception(f"There is no classical connection between the nodes {self.name} and {node1.name}")
        # We identify its endpoint within the classical communication channel
        if list(connection.ports.values())[0].connected_port.component is self.node:
            port0 = list(connection.ports.values())[0].connected_port
        else:
            port0 = list(connection.ports.values())[1].connected_port

        res = ''
        for i in range(gate.condition[0].size):
            res = '{}{}'.format(self.node.classical_register[i], res)
        port0.tx_output(Message(res, header='{}'.format(self.instr_id)))

    def run(self):
        """
        Executed the protocol
        """
        for instr_id, gate_tuple in enumerate(self.gate_tuples):
            self.instr_id = instr_id
            if self.can_execute(gate_tuple):
                yield from self.execute_instruction(gate_tuple)


class Simulation:
    """
    The class Simulation contains all the objects that are required to execute the simulation and performs an initialization
    of the system as well as asserting the coherence of the structural topology of the network and the algorithm to be
    mapped upon it.
    """

    def __init__(self, network: NetworkWrapper, quantum_circuit: DistQuantumCircuit):
        """
        Performs coherence check of the topology and quantum circuit to be executed.
        Args:
            network: network on which to execute the computation
            quantum_circuit: circuit description of the computation
        """
        self.network = network
        self.quantum_circuit = quantum_circuit
        self.__check_classical_registers()
        self.__check_quantum_registers()
        self.__setup_classical_connections()
        self.__check_coupling_map()

    def __check_classical_registers(self):
        """
        Checks if the QASM definition of registers and the QPUs definition in the YAML are coherent with each other.
        Requirements:
        - Each node must contain 1 classical register in the QASM and the node's name must be contained in the reg's name
        - There cannot be registers which are not mapped to QPUs.
        """
        # Setting up classical registers
        for reg in self.quantum_circuit.cregs:
            found = False
            for node_name, node in self.network.get_node_items():
                if node_name in reg.name:
                    if len(node.classical_register) != 0:
                        raise Exception("Another register is already bound to this node")
                    node.initialize_classical_register(reg.size)
                    found = True
                    break
            if not found:
                raise Exception("No node maps to the specified classical register")

    def __check_quantum_registers(self):
        """
        Checks if the QASM definition of registers and the QPUs definition in the YAML are coherent with each other.
        Requirements:
        - Each node must contain 1 quantum register in the QASM and the node's name must be contained in the reg's name
        - There cannot be registers which are not mapped to QPUs.
        """
        for reg in self.quantum_circuit.qregs:
            found = False
            for node_name, node in self.network.get_node_items():
                if node_name in reg.name:
                    if reg.size != node.subcomponents["main_memory"].num_positions:
                        raise Exception("Node definition and circuit do not match")
                    else:
                        if node.subcomponents["main_memory"].num_used_positions != 0:
                            raise Exception("Another register is already bound to this node")
                        node.subcomponents["main_memory"].put(create_qubits(reg.size))
                        found = True
                        break
            if not found:
                raise Exception(f"Register: {reg.name} does not match with any node")

    def __setup_classical_connections(self, channel_length: int = 20):
        """
        A classical connection is automatically setup between each node of the network.

        Args:
            channel_length: length of the classical channel

        """
        node_list = list(self.network.network.nodes.values())
        for i in range(len(self.network.network.nodes)):
            for j in range(i + 1, len(self.network.network.nodes)):
                self.network.network.add_connection(node_list[i], node_list[j],
                                                    ClassicalDirectConnection(
                                                        f"c_conn_{node_list[i].name}_{node_list[j].name}",
                                                        channel_length), label=f"classical_{i}_{j}")

    def __check_coupling_map(self):
        """
        Checks that the specified coupling map is coherent with the network's structure and specifications.
        Requirements:
        - Specified nodes must exist
        - Specified qubits must be defined as an Ebit of the specified node
        """
        for entry in self.network.coupling_map:
            for operand in entry:
                try:
                    node0 = self.network.network.nodes[operand.name]
                except KeyError:
                    raise Exception(f"Invalid node {operand.name} defined in the coupling map")
                found = False
                for ebit in node0.ebits:
                    if operand.index == ebit.index:
                        found = True
                        break
                if not found:
                    raise Exception("Invalid coupling map defined")

    def start(self):
        """
        Starts the Simulation by executing a protocol for each node.
        """
        sim_reset()
        for node in self.network.network.nodes.values():
            protocol = ExecutionProtocol(node, self.quantum_circuit.data, self.network)
            protocol.start()
        stats = sim_run(duration=2000*(10**10))
        print(stats)
