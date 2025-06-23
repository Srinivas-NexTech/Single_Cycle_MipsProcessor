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