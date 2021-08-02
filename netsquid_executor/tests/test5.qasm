OPENQASM 2.0;
include "qelib1.inc";
opaque entangle a, b;
opaque remoteCX a,b,c,d;

qreg alice_qubits[2]; //must contain the node name in the name of reg
qreg bob_qubits[2];

creg alice_bits[2];

x alice_qubits[0];
x alice_qubits[1];
measure alice_qubits[0] -> alice_bits[0];
measure alice_qubits[1] -> alice_bits[1];
h bob_qubits[0];
if (alice_bits == 2) z bob_qubits[0];
if (alice_bits == 1) h bob_qubits[0];
if (alice_bits == 3) x bob_qubits[1];
