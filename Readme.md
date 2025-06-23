# Single-Cycle MIPS Processor (Verilog)

This project implements a single-cycle MIPS processor in Verilog HDL. The processor includes:

- ALU
- Register File
- Control Unit
- Instruction Memory
- Data Memory
- Top-Level Module
- Testbench for simulation

## 📁 File Structure

├── alu.v # Performs arithmetic and logic operations
├── control_unit.v # Generates control signals based on opcode
├── data_memory.v # RAM for storing data
├── datapath.v # Connects processor components
├── instruction_memory.v # Stores instructions
├── mips_top.v # Integrates all components
├── mips_tb.v # Testbench
├── register_file.v # Contains 32 general-purpose registers
├── README.md # Project overview


## 🧪 Simulation

- Tool Used: Vivado 
- Testbench: `mips_tb.v`
- Features: Supports basic instructions like `add`, `sub`, `lw`, `sw`, `beq`, `j`

## 👨‍💻 Author

Srinivas Namathoti  


