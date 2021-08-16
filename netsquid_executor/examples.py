from src.simulation import *
import netsquid as ns
import sys

from netsquid_netconf.builder import ComponentBuilder
from netsquid_netconf.netconf import netconf_generator

from distributed_circuit import DistQuantumCircuit


def main():
    conf = input("Insert YAML file: ")
    code = input("Insert QASM file: ")

    # conf = 'examples/example6.yaml'
    # code = 'examples/example6.qasm'

    if conf == "" or code == "":
        print("Insert valid input files")
        sys.exit(1)

    # In order for the component builder to recognize our custom classes, we must add them to its dictionary.
    # ComponentBuilder.add_type("classical_connection", ClassicalDirectConnection)
    ComponentBuilder.add_type("quantum_connection", QuantumDirectConnection)
    ComponentBuilder.add_type("quantum_node", QuantumNode)
    # Invokes the network generator which takes nodes and connections and sets them up within a network object.
    generator = netconf_generator(conf, extra_builders=[EbitCouplingMapBuilder])
    components, config = next(generator)
    try:
        coupling_map = components["ebit_coupling_map"]
    except KeyError:
        coupling_map = []

    # The QASM file is parsed and interpreted by Qiskit to be later on interpreted by the tool to setup the simulation.
    # quantum_circuit = qs.QuantumCircuit.from_qasm_file(code)
    quantum_circuit = DistQuantumCircuit.from_qasm_file(code)
    print(quantum_circuit.draw(output="text"))
    network = NetworkWrapper(components["network"], coupling_map)
    simulation = Simulation(network, quantum_circuit)
    simulation.start()

    if "example1" in conf:
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0],
                                            network.network.nodes["bob"].subcomponents["main_memory"].peek(0)[0]]))

    if "example2" in conf:

        (ns.qubits.qubitapi.combine_qubits([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["alice"].subcomponents["main_memory"].peek(1)[0],
                                             network.network.nodes["bob"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["bob"].subcomponents["main_memory"].peek(1)[0],
                                             network.network.nodes["charlie"].subcomponents["main_memory"].peek(0)[0]]))

        print(network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0].qstate.qrepr.ket)

    if "example3" in conf:
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["alice"].subcomponents["main_memory"].peek(1)[0]]))
        print()

        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["bob"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["bob"].subcomponents["main_memory"].peek(1)[0]]))

    if "example5" in code:
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["alice"].subcomponents["main_memory"].peek(1)[0]]))
        print()

        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["bob"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["bob"].subcomponents["main_memory"].peek(1)[0]]))

    if "example6" in code:
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0]]))
        print()

        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["bob"].subcomponents["main_memory"].peek(1)[0]]))
        print()

        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["charlie"].subcomponents["main_memory"].peek(1)[0]]))
        print()

        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["eve"].subcomponents["main_memory"].peek(1)[0]]))

        print()
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["adam"].subcomponents["main_memory"].peek(1)[0]]))

        print()
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["alice"].subcomponents["main_memory"].peek(0)[0],
                                             network.network.nodes["adam"].subcomponents["main_memory"].peek(1)[0]]))

    if "example7" in conf:
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["qpu1"].subcomponents["main_memory"].peek(0)[0]]))
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["qpu2"].subcomponents["main_memory"].peek(0)[0]]))
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["qpu3"].subcomponents["main_memory"].peek(0)[0]]))
        print(ns.qubits.qubitapi.reduced_dm([network.network.nodes["qpu4"].subcomponents["main_memory"].peek(0)[0]]))


if __name__ == "__main__":
    main()



'''
@TODO(Sawii):
- Remotely controlled operations
- Sequential Execution of Chunks of Code
    --> Maybe a gate "wait" at which both involved nodes must wait for the other before continuing with the simulation
- implement all instructions also with custom parameters
- make channels more flexible to allow specification of loss and delay from yaml
- support multiple qubits instruction from qasm (X q[0,2,3])
'''
