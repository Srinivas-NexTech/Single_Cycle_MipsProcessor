module Mips_SingleCycleProcessor( clk, rst,pc, instr, alu_result, read_data1, read_data2, mem_read_data, final_write_data

    );
    input clk;
    input rst;
    wire RegDst, AluSrc, MemtoReg, RegWrite, MemRead, MemWrite, Branch, Jump;
    wire [2:0] ALUControl;
    output [31:0] pc, instr, alu_result, read_data1, read_data2, mem_read_data, final_write_data;

  // Control Path
  mips_controlpath cp(
    .opcode(instr[31:26]),
    .funct(instr[5:0]),
    .Regwrite(RegWrite),
    .Regdst(RegDst),
    .Alusrc(AluSrc),
    .MemWrite(MemWrite),
    .MemRead(MemRead),
    .MemReg(MemtoReg),
    .branch(Branch),
    .jump(Jump),
    .Alucontrol(ALUControl)
  );

  // Data Path
  mips_datapath dp(
    .clk(clk),
    .rst(rst),
    .RegDst(RegDst),
    .AluSrc(AluSrc),
    .MemtoReg(MemtoReg),
    .RegWrite(RegWrite),
    .MemRead(MemRead),
    .MemWrite(MemWrite),
    .Branch(Branch),
    .Jump(Jump),
    .ALUControl(ALUControl),
    .pc(pc),
    .instr(instr),
    .alu_result(alu_result),
    .read_data1(read_data1),
    .read_data2(read_data2),
    .mem_read_data(mem_read_data),
    .final_write_data(final_write_data)
  );
endmodule


module mips_controlpath(opcode,funct,Regwrite,Regdst,Alusrc,MemWrite,MemRead,MemReg,branch,jump,Alucontrol

    );
    input[5:0]opcode,funct;
    output Regwrite,Regdst,Alusrc,MemWrite,MemRead,MemReg,branch,jump;
    output [2:0]Alucontrol;
    wire [1:0]Aluop;
    
   main_decoder uut(opcode,Regwrite,Regdst,Alusrc,MemWrite,MemRead,MemReg,branch,jump,Aluop);
   Alu_decoder uut1(Aluop,funct,Alucontrol);
endmodule


module main_decoder(input [5:0]opcode,
                    output reg Regwrite,Regdst,Alusrc,MemWrite,MemRead,MemReg,jump,branch,
                    output reg[1:0]Aluop);
                     always@(*)begin
                    Regdst=0;
                    Regwrite=0;
                    Alusrc=0;
                    MemWrite=0;
                    MemRead=0;
                    MemReg=0;
                    jump=0;
                   branch=0;
                   Aluop=2'b00;
                 //can asssign values like this also { Regdst,Regwrite,Alusrc,MemWrite, MemRead,MemReg,jump,pcsrc,Aluop}='b00000000_000;
                   
       case(opcode)
       6'b000000:begin
                    Regdst=1;
                    Regwrite=1;
                    Alusrc=0;
                    MemWrite=0;
                    MemRead=0;
                    MemReg=0;
                    jump=0;
                   branch=0;
                   Aluop=2'b10;
                 end
     6'b100011: begin //loadWord LW
                 Regdst=0;
                 Regwrite=1;
                 Alusrc=1;
                 MemWrite=0;
                 MemRead=1;
                 MemReg=1;
                 jump=0;
                 branch=0;
                 Aluop=2'b00;
               end
      6'b101011: begin //storeWord SW
                 Regdst=0;
                 Regwrite=0;
                 Alusrc=1;
                 MemWrite=1;
                 MemRead=0;
                 MemReg=0;
                 jump=0;
                 branch=0;
                 Aluop=2'b00;
               end
       6'b000100: begin //Branch if equals to BEQ
                 Regdst=0;
                 Regwrite=0;
                 Alusrc=1;
                 MemWrite=0;
                 MemRead=0;
                 MemReg=0;
                 jump=0;
                 branch=1;
                 Aluop=2'b01;
               end
         6'b000010: begin //jump j
                 Regdst=0;
                 Regwrite=0;
                 Alusrc=0;
                 MemWrite=0;
                 MemRead=0;      
                 MemReg=0;
                 jump=1;
                 branch=0;
               end
       default: begin
                   Regdst=0; Regwrite=0; Alusrc=0;
                   MemWrite=0; MemRead=0; MemReg=0;
                   jump=0; branch=0; Aluop=2'b00;
                 end
       endcase
    end
 endmodule
 
 module Alu_decoder(input [1:0]Aluop,
                    input[5:0]funct,
                    output reg [2:0]Alucontrol
                    );
                  always@(*)begin
                       if(Aluop==2'b00)
                            Alucontrol=3'b000;
                       else if(Aluop==2'b01)
                            Alucontrol=3'b010;
                       else if(Aluop==2'b10)begin  //look at function field 
                         case(funct)
                              6'b100000:Alucontrol=3'b000;//ADD
                              6'b100010:Alucontrol=3'b001;//SUB
                              6'b100100:Alucontrol=3'b010;//AND
                              6'b100101:Alucontrol=3'b011;//OR
                              6'b100110:Alucontrol=3'b100;//XOR
                              6'b100111:Alucontrol=3'b101;//NOR
                              6'b101010:Alucontrol=3'b110;//if $rt1<rt2 then set $rd=1 otherwise $rd=0 (assembly language command slt $t1,$t2,$t3
                                                    //SLT set less than 
                          default: Alucontrol = 3'b000; // fallback to ADD
                       endcase
                 end 
              end        
    endmodule
    
    
    module mips_datapath(input clk,
    input rst,
    input RegDst,
    input AluSrc,
    input MemtoReg,
    input RegWrite,
    input MemRead,
    input MemWrite,
    input Branch,
    input Jump,
    input [2:0] ALUControl,

    output [31:0] pc,
    output [31:0] instr,
    output [31:0] alu_result,
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] mem_read_data,
    output [31:0] final_write_data
     );
   
    wire [31:0] next_pc;
    wire [31:0] pc_plus4, branch_addr, jump_mux_out, branch_mux_out;
    wire [4:0] write_reg;
    wire [31:0] alu_src_b;
    wire [31:0] sign_ext_imm;
    wire [31:0] shifted_imm;
    wire [31:0] branch_target;
    wire pcsrc;
 PC pc_inst(clk, rst, next_pc, pc);

    // Instruction Memory
    IM im_inst(clk, pc, instr);

    // PC + 4
    pc_adder pcadd_inst(pc, pc_plus4);

    // Register Destination MUX
    mux regdst_mux(instr[20:16], instr[15:11], RegDst, write_reg);

    // Register File
    Register_file rf(clk, instr[25:21], instr[20:16], write_reg, final_write_data, RegWrite, read_data1, read_data2);

    // ALU Source MUX + Sign Extension
    Mux_1 alusrc_mux(instr[15:0], read_data2, AluSrc, alu_src_b);

    // ALU
    ALU alu_inst(read_data1, alu_src_b, ALUControl, alu_result);

    // Data Memory
    DM dm(clk, alu_result, MemRead, MemWrite, read_data2, mem_read_data);

    // MemtoReg MUX (corrected)
    Mux memtoreg_mux(mem_read_data, alu_result, MemtoReg, final_write_data);

    // Shifted Immediate (for Branch target)
    adder branch_adder(instr[15:0], pc_plus4, branch_target);

    // Branch Decision Logic (based on ALU result = zero)
    assign pcsrc = Branch & (alu_result == 0);

    // Branch MUX
    Mux_2 branch_mux(pc_plus4, branch_target, pcsrc, branch_mux_out);

    // Jump MUX
    Mux_3 jump_mux(instr[25:0], pc_plus4, branch_mux_out, Jump, next_pc);   
    
    
endmodule

//Program Counter
module PC(clk,rst,next_pc,pc);
   input clk, rst;
   input[31:0]next_pc;
   output reg[31:0]pc;
always@(posedge clk)
  begin
if(rst)
   pc<=32'b0;
else
   pc<=next_pc;
  end
endmodule

//Instruction memory
module IM(clk,pc,ins);
   input clk;
   input[31:0]pc;
   output reg[31:0]ins;
   reg[7:0]mem[0:255];
   wire[7:0] addr=pc[7:0]; 
always@(posedge clk)
begin
ins<={mem[addr],mem[addr+1],mem[addr+2],mem[addr+3]};
   end
endmodule

//MUX5bit :selecting registers
module mux(a,b,RegDst,mux_out);
   input[4:0]a,b;
   input RegDst;
   output reg[4:0]mux_out;
always@(*)begin
  if(RegDst)
      mux_out=b;
 else 
     mux_out=a;
 end
endmodule

//Register file
module Register_file(clk,Read_reg1,Read_reg2,Write_reg,Write_data, RegWrite,Read_data1,Read_data2);
       input clk;
       input[4:0]Read_reg1,Read_reg2,Write_reg;
       input[31:0] Write_data;
       input RegWrite;
       output reg[31:0]Read_data1,Read_data2;
       reg[31:0]regfile[0:31];
    always@(*)begin
         Read_data1=regfile[Read_reg1];
         Read_data2=regfile[Read_reg2];
      end
  always@(posedge clk) begin
     if(RegWrite&&Write_reg!=0)          //$zero register always stays zero
        regfile[Write_reg]<=Write_data;
    end
endmodule

//Sign Extensinon and MUX
module Mux_1(in,Read_data1,Alusrc,out);
  input[15:0]in;
  input[31:0]Read_data1;
  input Alusrc;
  output reg[31:0]out;
 wire[31:0]sign_extension;
assign sign_extension={{16{in[15]}},in[15:0]};
always@(*)begin
   if(Alusrc)
      out=sign_extension;
   else
      out=Read_data1;
  end
endmodule

//ALU CONTROL
module ALU(a,b,Alucontrol,Alu_out);
input[31:0]a,b;
input[2:0]Alucontrol;
output reg[31:0]Alu_out;
always@(*)begin
  if(Alucontrol==3'b000)        //ADD
    Alu_out=a+b;
  else if(Alucontrol==3'b001)  //SUB
      Alu_out=a-b;
  else if(Alucontrol==3'b010)  //AND
       Alu_out=a&b;
  else if(Alucontrol==3'b011)   //OR
       Alu_out=a|b;
  else if(Alucontrol==3'b100)  //XOR
       Alu_out=a^b;
  else if(Alucontrol==3'b101)  //NOR
       Alu_out=~(a|b);       
  else if(Alucontrol==3'b110) begin    //SLT SET LESS THAN
     if(a<b)
         Alu_out=32'b1;
   else
        Alu_out=32'b0;
      end
   end
endmodule

//Data Memory
module DM(clk,Alu_out,Mem_Read,Mem_Write,Write_data,Read_data);
       input clk;
       input[31:0]Alu_out;
       input Mem_Read,Mem_Write;
       input[31:0]Write_data;
       output reg [31:0]Read_data;
     reg[7:0]mem[0:255];
     wire[7:0]addr=Alu_out[7:0];
 always@(*)begin
  if(Mem_Read)
     Read_data={mem[addr],mem[addr+1],mem[addr+2],mem[addr+3]};
  else
     Read_data=  32'b0;
  end
 always@(posedge clk)begin
   if(Mem_Write)begin
     mem[addr]<=Write_data[31:24];
     mem[addr+1]<=Write_data[23:16];
     mem[addr+2]<=Write_data[15:8];
     mem[addr+3]<=Write_data[7:0];
    end 
  end   
endmodule

//MUX
module Mux(Read_data,Alu_out,Mem_Reg,Mux_out);
   input[31:0]Read_data;
   input[31:0]Alu_out;
   input Mem_Reg;
   output reg[31:0]Mux_out;
always@(*)begin
  if(Mem_Reg)
     Mux_out=Read_data;
  else
     Mux_out=Alu_out;
   end
endmodule

//shift left  Branch ADDER
module adder(x,y,out);
    input[31:0]x;
    input[31:0]y;
    output reg[31:0]out;
always@(*)begin
   out=y+(x<<2);//shift left by 2 units
  end
endmodule

//PC+4  ADDER
module pc_adder(pc,next_pc);
  input[31:0]pc;
  output reg [31:0]next_pc;
  always@(*)
   next_pc=pc+4;
endmodule

//MUX
module Mux_2(next_pc,out,pcsrc,y);
   input[31:0]next_pc,out;
   input pcsrc;
   output reg[31:0]y;
always@(*)
 begin
  if(pcsrc)
     y=out;
  else
    y=next_pc;
 end
endmodule

module Mux_3(ins,next_pc,y,jump,jump_out);
  input[25:0]ins;
  input[31:0]next_pc,y;
  input jump;
  output reg[31:0]jump_out;
   wire[31:0]jump_adrress;
   assign jump_adrress={{next_pc[31:28]},{ins[25:0]},2'b00};
  always@(*)
   begin
    if(jump)
     jump_out=jump_adrress;
   else
     jump_out=y;
   end
endmodule