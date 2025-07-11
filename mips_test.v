module Mips_SingleCycleProcessor_tb();

     
  reg clk;
  reg rst;
  wire [31:0] pc, instr, alu_result, read_data1, read_data2, mem_read_data, final_write_data;

  // Instantiate the processor
  Mips_SingleCycleProcessor uut (
    .clk(clk),
    .rst(rst),
    .pc(pc),
    .instr(instr),
    .alu_result(alu_result),
    .read_data1(read_data1),
    .read_data2(read_data2),
    .mem_read_data(mem_read_data),
    .final_write_data(final_write_data)
  );

  // Clock Generation
  always #5 clk = ~clk;

  // Initialize Instruction Memory with some basic operations
  initial begin
    clk = 0;
    rst = 1;

    // Wait for reset
    #10 rst = 0;

    // Wait for a few instructions to execute
    #200;

    $display("Finished simulation");
    $finish;
  end

  // Preload Instruction Memory with instructions
  initial begin
    // Load the instructions manually into IM memory array
    // Format: mem[addr] = 8-bit hex
    // Example instruction: add $t1, $t2, $t3  => opcode: 000000 rs=01010 rt=01011 rd=01001 shamt=00000 funct=100000
    uut.dp.im_inst.mem[0] = 8'h01;  // MSB
    uut.dp.im_inst.mem[1] = 8'h4B;
    uut.dp.im_inst.mem[2] = 8'h48;
    uut.dp.im_inst.mem[3] = 8'h20;  // LSB

    // Example: sub $t1, $t2, $t3
    uut.dp.im_inst.mem[4] = 8'h01;
    uut.dp.im_inst.mem[5] = 8'h4B;
    uut.dp.im_inst.mem[6] = 8'h48;
    uut.dp.im_inst.mem[7] = 8'h22;

    // lw $t0, 4($t1)
    uut.dp.im_inst.mem[8]  = 8'h8D;
    uut.dp.im_inst.mem[9]  = 8'h28;
    uut.dp.im_inst.mem[10] = 8'h00;
    uut.dp.im_inst.mem[11] = 8'h04;

    // sw $t0, 8($t1)
    uut.dp.im_inst.mem[12] = 8'hAD;
    uut.dp.im_inst.mem[13] = 8'h28;
    uut.dp.im_inst.mem[14] = 8'h00;
    uut.dp.im_inst.mem[15] = 8'h08;

    // beq $t0, $t0, label (branch to +4)
    uut.dp.im_inst.mem[16] = 8'h11;
    uut.dp.im_inst.mem[17] = 8'h00;
    uut.dp.im_inst.mem[18] = 8'h00;
    uut.dp.im_inst.mem[19] = 8'h01;
  end

  // Optionally preload Register File
  initial begin
    // Example: Set $t2 = 5, $t3 = 3
    uut.dp.rf.regfile[10] = 32'd5;  // $t2
    uut.dp.rf.regfile[11] = 32'd3;  // $t3
    uut.dp.rf.regfile[9]  = 32'd100; // $t1 for address offset
  end

  // Optionally preload Data Memory
  initial begin
    uut.dp.dm.mem[104] = 8'h00;
    uut.dp.dm.mem[105] = 8'h00;
    uut.dp.dm.mem[106] = 8'h00;
    uut.dp.dm.mem[107] = 8'h64; // 32'h00000064 = 100
  end

endmodule
