OPENQASM 2.0;
include "qelib1.inc";

qreg alice_qubits[2]; //must contain the node name in the name of reg
qreg bob_qubits[3];
qreg charlie_qubits[3];
qreg eve_qubits[3];
qreg adam_qubits[2];

x alice_qubits[0];

h alice_qubits[0];

epr alice_qubits[1], bob_qubits[0];

epr bob_qubits[2], charlie_qubits[0];

epr charlie_qubits[2], eve_qubits[0];

epr eve_qubits[2], adam_qubits[0];

entswap alice_qubits[1], bob_qubits[0], bob_qubits[2], charlie_qubits[0], charlie_qubits[2], eve_qubits[0], eve_qubits[2], adam_qubits[0];

remoteCx alice_qubits[0], alice_qubits[1], adam_qubits[1], adam_qubits[0];

