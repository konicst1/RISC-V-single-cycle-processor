`default_nettype none
module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );
                
	//define wires
	wire branchJalr_w, branchJal_w, branchBeq_w, regWrite_w, memToReg_w, memWrite_w, aluSrc_w, auiControl_w;
	wire[2:0] immControl_w;
	wire[3:0] aluControl_w;
	wire[31:0] immOp_w;
	wire[31:0] srcA_w, srcB_w;
	wire[31:0] rs2_w;
	wire zero_w;
	wire[31:0] aluOut_w;
	wire[31:0] wd3_w;
	wire[31:0] pc_plus_4_w, pc_plus_imm_w;
	wire[31:0] branchmux_to_datamux_w;
	wire[31:0] PCn_w, PC_w;
	wire[31:0] auimuxout_w;
	
	control_unit CONTROL_UNIT(instruction, branchJalr_w, branchJal_w, branchBeq_w, regWrite_w, memToReg_w, 					  memWrite_w, aluSrc_w, auiControl_w,aluControl_w, immControl_w);
	
	imm_decoder IMM_DECODER( instruction[31:7], immControl_w, immOp_w);
	
	mux2_1 ALU_MUX(rs2_w, immOp_w, aluSrc_w, srcB_w);
	
	ALU ALU_3000(srcA_w, srcB_w, aluControl_w, zero_w, aluOut_w);
	
	GPR_set GPR_SET(instruction[19:15], instruction[24:20], instruction[11:7], wd3_w, regWrite_w, srcA_w, rs2_w, clk);
	
	assign address_to_mem = auimuxout_w;
	assign data_to_mem = rs2_w;
	assign WE = memWrite_w;
	
	
	mux2_1 BRANCH_WRITE_MUX(auimuxout_w, pc_plus_4_w, branchJalr_w | branchJal_w, branchmux_to_datamux_w);
	mux2_1 DATA_MUX(branchmux_to_datamux_w, data_from_mem, memToReg_w, wd3_w);
	mux2_1 AUIPC_MUX(aluOut_w, pc_plus_imm_w, auiControl_w, auimuxout_w);
	
	mux3_1 PC_MUX(pc_plus_4_w, aluOut_w, pc_plus_imm_w, (branchBeq_w & zero_w) | branchJal_w, branchJalr_w, PCn_w);
	pc_passer PC_PASSER(PCn_w, clk, reset, PC_w);
	assign PC = PC_w;
	
	adder_4 PC_4_ADDER(PC_w, pc_plus_4_w);
	adder_32 PC_IMM_ADDER(immOp_w, PC_w, pc_plus_imm_w);
    	
    	
    
endmodule

`default_nettype wire

module mux2_1(input[31:0] d0, d1,input s, output[31:0] y);
  assign y = s ? d1 : d0;
endmodule


module mux3_1(input[31:0] d0, d1, d2, input s1, s0, output[31:0] y);
	wire[31:0] w0;
	mux2_1 A(d0, d2, s1, w0);
	mux2_1 B(w0, d1, s0, y);
endmodule 

module GPR_set(input[4:0] a1, a2, a3, input[31:0] wd3, input we3, output reg[31:0] rd1, rd2, input clk);
	reg [31:0] regs[31:0];
	
	always@(*) begin	
		regs[0] = 0; 	
		rd1 = regs[a1];
		rd2 = regs[a2];
	end
	
	always@(posedge clk) begin
		if(we3) regs[a3]=wd3;	
	end
endmodule


module adder_32(input[31:0] a1, a2, output [31:0] y);
	assign y = a1 + a2;
endmodule

module adder_4(input[31:0] a1, output [31:0] y);
	assign y = a1 + 4;
endmodule

// 0000 (+), 0001 (-), 0010 (<), 0011 (&), 0100 (BYTE+), 0101 (FILL_TO_32), 0110 (sll), 0111 (srl), 1000 (sra) 
module ALU(input signed [31:0] srcA, srcB, input[3:0] ALUControl, output reg zero, output reg[31:0] ALUOut);
	always@(*) begin
		case(ALUControl)
			4'b0000:  ALUOut = srcA + srcB;
			4'b0001:  ALUOut = srcA - srcB;
			4'b0010:  ALUOut = (srcA < srcB) ? 1 : 0 ;
			4'b0011:  ALUOut = srcA & srcB;
			4'b0100:  ALUOut = { srcA[31:24] + srcB[31:24], srcA[23:16] + srcB[23:16], srcA[15:8] + srcB[15:8], srcA[7:0] + srcB[7:0]};
			
			4'b0101:  ALUOut = { srcB[31:12], 12'b0};
			4'b0110:  ALUOut = srcA << srcB;
			4'b0111:  ALUOut = ( $unsigned(srcA) >> (srcB) );
			4'b1000:  ALUOut = ( $signed(srcA) >> (srcB) );
			
		default:
			ALUOut = 32'b11111111111111111111111111111111;
		endcase
		if(ALUOut == 0) zero = 1;
		else zero = 0;
	end
endmodule

// 000 (addi, lw, jalr), 001 (sw), 010 (beq), 011(lui, auipc), 100(jal)   
module imm_decoder(input[31:7] imm, input[2:0] immControl, output reg[31:0] immOp);
	always@(*) begin
		case(immControl)
			3'b000: immOp = { {20{imm[31]}}, imm[31:20] };
			3'b001: immOp = { {20{imm[31]}}, imm[31:25], imm[11:7] };
			3'b010: immOp = { {19{imm[31]}}, imm[31], imm[7], imm[30:25], imm[11:8], 1'b0 };
			3'b011: immOp = { imm[31:12], 12'b0 };
			3'b100: immOp = { {11{imm[31]}}, imm[31], imm[19:12], imm[20], imm[30:21], 1'b0 };
		default:
			immOp = 32'b0;
		endcase
	end
endmodule

module control_unit(input[31:0] instr, output reg branchJalr, branchJal, branchBeq, regWrite, memToReg, memWrite, aluSrc, auiControl_w 						,output reg[3:0] aluControl, output reg[2:0] immControl);
	always@(*)begin
		//check opcode
		case(instr[6:0])
			//R - type (add, and, sub, slt, sll, srl, sra)	
			7'b0110011: begin
				case(instr[14:12])
					//add, sub
					3'b000: begin
						case(instr[31:25])
							//add
							7'b0000000: begin
								immControl = 3'b000;
								aluSrc = 0;
								aluControl = 4'b0000;
								memWrite = 0;
								memToReg = 0;
								regWrite = 1;
								branchBeq = 0;
								branchJal = 0;
								branchJalr = 0;
								auiControl_w = 0;
							end
							//sub
							7'b0100000: begin
								immControl = 3'b000;
								aluSrc = 0;
								aluControl = 4'b0001;
								memWrite = 0;
								memToReg = 0;
								regWrite = 1;
								branchBeq = 0;
								branchJal = 0;
								branchJalr = 0;
								auiControl_w = 0;
							end
						default: immControl = 3'b111;
						endcase
					end
					//and
					3'b111: begin
						immControl = 3'b000;
						aluSrc = 0;
						aluControl = 4'b0011;
						memWrite = 0;
						memToReg = 0;
						regWrite = 1;
						branchBeq = 0;
						branchJal = 0;
						branchJalr = 0;
						auiControl_w = 0;
					end
					//slt
					3'b010: begin
						immControl = 3'b000;
						aluSrc = 0;
						aluControl = 4'b0010;
						memWrite = 0;
						memToReg = 0;
						regWrite = 1;
						branchBeq = 0;
						branchJal = 0;
						branchJalr = 0;
						auiControl_w = 0;
					end
					//sll
					3'b001: begin
						immControl = 3'b000;
						aluSrc = 0;
						aluControl = 4'b0110;
						memWrite = 0;
						memToReg = 0;
						regWrite = 1;
						branchBeq = 0;
						branchJal = 0;
						branchJalr = 0;
						auiControl_w = 0;
					end
					//srl, sra
					3'b101: begin
						case(instr[31:25])
							//srl
							7'b0000000: begin
								immControl = 3'b000;
								aluSrc = 0;
								aluControl = 4'b0111;
								memWrite = 0;
								memToReg = 0;
								regWrite = 1;
								branchBeq = 0;
								branchJal = 0;
								branchJalr = 0;
								auiControl_w = 0;
							end
							//sra
							7'b0100000: begin
								immControl = 3'b000;
								aluSrc = 0;
								aluControl = 4'b1000;
								memWrite = 0;
								memToReg = 0;
								regWrite = 1;
								branchBeq = 0;
								branchJal = 0;
								branchJalr = 0;
								auiControl_w = 0;
							end
						default: immControl = 3'b111;
						endcase
					end
				default: immControl = 3'b111;
				endcase
			end
			//I - type (addi)
			7'b0010011: begin
				immControl = 3'b000;
				aluSrc = 1;
				aluControl = 4'b0000;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//I - type (lw)
			7'b0000011: begin
				immControl = 3'b000;
				aluSrc = 1;
				aluControl = 4'b0000;
				memWrite = 0;
				memToReg = 1;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//I - type (jalr)
			7'b1100111: begin
				immControl = 3'b000;
				aluSrc = 1;
				aluControl = 4'b0000;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 1;
				auiControl_w = 0;
			end
			//B - type (beq)
			7'b1100011: begin
				immControl = 3'b010;
				aluSrc = 0;
				aluControl = 4'b0001;
				memWrite = 0;
				memToReg = 0;
				regWrite = 0;
				branchBeq = 1;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//S - type (sw)
			7'b0100011: begin
				immControl = 3'b001;
				aluSrc = 1;
				aluControl = 4'b0000;
				memWrite = 1;
				memToReg = 1;
				regWrite = 0;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//U - type (lui)
			7'b0110111: begin
				immControl = 3'b011;
				aluSrc = 1;
				aluControl = 4'b0101;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;	
			end
			//J - type (jal)
			7'b1101111: begin
				immControl = 3'b100;
				aluSrc = 0;
				aluControl = 4'b0000;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 1;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//qb - type (addu.qb)
			7'b0001011: begin
				immControl = 3'b000;
				aluSrc = 0;
				aluControl = 4'b0100;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 0;
			end
			//auipc - type (auipc)
			7'b0010111: begin
				immControl = 3'b011;
				aluSrc = 1;
				aluControl = 4'b0100;
				memWrite = 0;
				memToReg = 0;
				regWrite = 1;
				branchBeq = 0;
				branchJal = 0;
				branchJalr = 0;
				auiControl_w = 1;
			end
		default: immControl = 3'b111;	
		endcase
	end


endmodule

module pc_passer(input[31:0] PCn, input clk, input reset, output reg[31:0] PC);
	always@(posedge clk) begin
		if(reset == 1) PC = 31'b0;
		else PC = PCn;
	end
endmodule



