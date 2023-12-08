
`define OPCODE_COMPUTE    7'b0110011
`define OPCODE_COMPUTE_IMM  7'b0010011
`define OPCODE_BRANCH     7'b1100011
`define OPCODE_LOAD       7'b0000011
`define OPCODE_STORE      7'b0100011
`define OPCODE_LUI        7'b0110111
`define OPCODE_AUIPC      7'b0010111
`define OPCODE_JAL        7'b1101111 
`define OPCODE_JALR       7'b1100111

// FUNCT3 For Immediate Shifts
`define SHIFT_LEFT_I      3'b001
`define SHIFT_RIGHT_I     3'b101

// FUNCT3 for Branches
`define BEQ          3'b000
`define BNE          3'b001
`define BLT          3'b100
`define BGE          3'b101
`define BLTU         3'b110
`define BGEU         3'b111

// FUNCT3 for Loads
`define LB           3'b000
`define LH           3'b001
`define LW           3'b010
`define LBU          3'b100
`define LHU          3'b101

// FUNCT3 for Stores
`define SB           3'b000
`define SH           3'b001
`define SW           3'b010


`define FUNC_ADD      3'b000
`define AUX_FUNC_ADD  7'b0000000
`define AUX_FUNC_SUB  7'b0100000

`define SIZE_BYTE     2'b00
`define SIZE_HWORD    2'b01
`define SIZE_WORD     2'b10 
`define NO_DATA_WRITE 2'b11



// DECODER DEFINES
`define REGISTOR        1'b1
`define IMMEDIATE       1'b0
`define MEM_WRITE       1'b1 // Opposite from register for some reason
`define MEM_NO_WRITE    1'b0
`define REG_WRITE       1'b0 //For some reason, Registor Write Enable is 0 :`(
`define REG_NO_WRITE    1'b1



module PipelinedCPU(halt, clk, rst);
   output halt;
   input clk, rst;


   //Declaration of wires used in instantiation.


   //^ Instruction Memory Inputs and Outputs (InstMem)
   //* INPUTS
   // The PC or Program Counter
   wire [31:0] InstWord; // The Instruction from Memory for this current cycle, use this to decode the instruction
   // Clock signal
   // `SIZE_WORD because we always want the full 32 bit instruction
   



   //^ IF_ID_REG Inputs and Outputs (IF_ID_REG)
   //* INPUTS
   reg stall;
   reg act; 
   wire branch_check;

   //? OUTPUTS
   wire [31:0] IF_ID_Instword; 
   wire [31:0] IF_ID_PC;


   //^ CONTROL Inputs and Outputs (Control)
   //* INPUTS
   // IF_ID_InstWord defined above

   //? OUTPUTS
   wire DWrEn;
   wire signed [31:0] immediate;
   wire RWrEn;
   wire [1:0] mem_size;


   //^ REGFILE Inputs and Outputs (RegFile)
   //* INPUTS
   wire [4:0] Rsrc1; //Desired register #1 
   wire [4:0] Rsrc2; // Desired register #2
   wire [4:0] Rdst;  // Desired destination register #
   wire reg_write_en; // Bool to decide whether to write to register or not

   //? OUTPUTS
   wire [31:0] Rdata1; // Data in Rsrc1
   wire [31:0] Rdata2; //Data in Rsrc2 
   wire [31:0] RWrdata; //Data to be written to register (to Rdst)


   //^ ID_EX_REG Inputs and Outputs (ID_Ex_reg)
   //* INPUTS
   // clk,
   // stall, FROM PIPELINE CPU
   // branch_check,

   // Register File Inputs
   wire shift_type;
   // Rsrc1
   // Rsrc2 
   // Rdst
   wire [2:0] funct3;
   wire [6:0] opcode;
   // reg_write_en,
   // Rdata1
   // Rdata2
   // x31 ?

   // Control Block Inputs
   // wire data_write_en;
   // immediate
   // PC

   //? OUTPUTS
   // Register File Outputs
   wire ID_EX_shift_type;
   wire [4:0] ID_EX_Rsrc1;
   wire [4:0] ID_EX_Rsrc2;
   wire [4:0] ID_EX_Rdst;
   wire [2:0] ID_EX_funct3;
   wire [6:0] ID_EX_opcode;
   wire ID_EX_reg_write_en;
   wire [31:0] ID_EX_Rdata1;
   wire [31:0] ID_EX_Rdata2;
   // Control block outputs
   wire ID_EX_data_write_en;
   wire signed [31:0] ID_EX_immediate;
   wire [31:0] ID_EX_PC;
   wire ID_EX_stall;
   wire [1:0] ID_EX_mem_size;



   //^ ALU or ExecutionUnit Inputs and Outputs (ExecutionUnit)
   //* INPUTS
   // clk, 
   // stall,
   // branch_check,
   // Register File Inputs
   // shift_type,
   //  rs1,
   // rs2,
   //  rd,
   //  funct3,
   //  opcode,
   //  reg_write_en,
   reg [31:0] ALU_rv1_in;
   reg [31:0] ALU_rv2_in;
   //  x31,
   //  Control Block Inputs
   // data_write_en,
   // immediate,
   // iaddr,

   //? OUTPUTS
   wire [31:0] ALU_reg_data;
   wire [31:0] ALU_addr;
   wire ALU_data_write_en;
   wire [31:0] ALU_dw_data;
	wire [31:0] pc_new; 
	wire pc_JALR;
   reg [31:0] pc_update;
   wire [1:0] ALU_mem_size;




   //^ EX_MEM_REG Inputs and Outputs (EX_MEM)
   //* INPUTS
   // clk, 

   // branch_check 
   // stall,
   //* FROM ID_EX
   // opcode,
   // funct3,
   // rd,
   // reg_write_en;

   //* FROM ALU
   // data_write_en,
   // addr,
   // reg_data,
   // dw_data,

   //? OUTPUTS
   wire [6:0] EX_MEM_opcode;
   wire [2:0] EX_MEM_funct3;
   wire [31:0] EX_MEM_addr;
   wire EX_MEM_data_write_en;
   wire [4:0] EX_MEM_rd;
   wire [31:0] EX_MEM_reg_data;
   wire [31:0] EX_MEM_dw_data;
   wire EX_MEM_reg_write_en;// EX_MEM_branch_check, EX_MEM_stall;
   wire [1:0] EX_MEM_mem_size;


   //^ DMEM OUTPUTS
   //*INPUTs
   //? OUTPUTS
   wire [31:0] DMEM_data;



   //^ MEM_WB inputs and outputs (MEM_WB_reg)
   //* INPUTS
   // clk, 
   //* FROM EX_MEM
   // reg_write_en,
   // opcode,
   // funct3,
   // addr,
   // rd,
   // reg_data,
   //* FROM DMEM
   // dmem_data,

   //? OUTPUTS
   wire MEM_WB_reg_write_en;
   wire [6:0] MEM_WB_opcode;
   wire [2:0] MEM_WB_funct3;
   wire [31:0] MEM_WB_addr;
   wire [4:0] MEM_WB_rd;
   wire [31:0] MEM_WB_reg_data;
   wire [31:0] MEM_WB_data;



   //^ L_INST INPUTS AND OUTPUTS
   //* INPUTS
   //* ALL FROM MEM_WB
   // opcode,
   // funct3,
   //addr,
   // data,
   // reg_data,
   //? OUTPUTS
   wire [31:0] L_INST_data;

   //^ WB_REG INPUTS AND OUTPUTS
   //* INPUTS
   // clk,
   // FROM L_INST
   // rd,
   // reg_data,
   // reg_write_en,
   //? OUTPUTS
   wire [4:0] WB_rd;
   wire [31:0] WB_reg_data;
   wire WB_reg_write_en;



   wire [31:0] PC;
   wire [31:0] NPC;
	reg [31:0] NPC_temp;
   assign NPC = NPC_temp;

   //! IF stall == 0 // Hazard detected, stall


	//^ PC update
   always@(clk)
      begin
         // if(rst)       
         //    NPC_temp = 4;
			if(stall == 0 || branch_check == 0) begin
				NPC_temp = PC + 4 - pc_update;
         end
         else if(pc_new && pc_JALR == 0) begin
				NPC_temp = pc_new;
         end
         else if(pc_JALR==1) begin
            NPC_temp = PC + 4 - pc_update + pc_new;
         end
   end
	 

	 
	 //^ Simple R-Type Decode of Instruction Word
   assign opcode = IF_ID_Instword[6:0];   
   assign Rdst = IF_ID_Instword[11:7]; 
   assign Rsrc1 = IF_ID_Instword[19:15]; 
   assign Rsrc2 = IF_ID_Instword[24:20];
   assign funct3 = IF_ID_Instword[14:12];  // R-Type, I-Type, S-Type
   assign funct7 = IF_ID_Instword[31:25];  // R-Type

   wire [4:0] rd_final;
	assign rd_final = (PC < 8)? 0 :  MEM_WB_rd;
   wire reg_wr;


	always@(*) begin
      //* FORWARD MUX, only for register values
      // Branching is done with branch_check
      //Default Values
		pc_update = 32'b0;
		stall = 1;
		act = 1;
   
      // For ALU_rv1_in
		if(EX_MEM_rd == ID_EX_Rsrc1 
      && EX_MEM_reg_write_en == 1 
      && PC>=8) begin
			if(EX_MEM_opcode == `OPCODE_LOAD) begin
				pc_update = 4;
				stall=0;
				act=0;
			end
			else ALU_rv1_in = EX_MEM_reg_data;	
		end

		else if(MEM_WB_rd == ID_EX_Rsrc1 
           && MEM_WB_reg_write_en == 1
           && PC >= 8) begin
			ALU_rv1_in = L_INST_data;
		end

		else if(WB_rd == ID_EX_Rsrc1 
           && WB_reg_write_en == 1
           && PC>=12) begin
			
         ALU_rv1_in = WB_reg_data;
		
      end
      else begin
			ALU_rv1_in = ID_EX_Rdata1;
		end

      // FOR ALU_rv2_in
		if(EX_MEM_rd == ID_EX_Rsrc2 
      && EX_MEM_reg_write_en == 1 
      && PC>=8 
      && (ID_EX_opcode[6:4]==3'b110 
         || ID_EX_opcode[6:4]==3'b011 
         || ID_EX_opcode[6:4]==3'b010)
      ) begin
			if(EX_MEM_opcode == `OPCODE_LOAD) begin
				pc_update = 4;
				stall=0;
				act=0;
			end
			else begin
			   ALU_rv2_in = EX_MEM_reg_data;
			end
		end

		else if(MEM_WB_rd == ID_EX_Rsrc2 
           && MEM_WB_reg_write_en == 1 
           && PC >= 8 
           && (ID_EX_opcode[6:4]==3'b110 
            || ID_EX_opcode[6:4]==3'b011 
            || ID_EX_opcode[6:4]==3'b010)
         ) begin
			   ALU_rv2_in = L_INST_data;
		end

		else if(WB_rd == ID_EX_Rsrc2 
           && WB_reg_write_en == 1 
           && PC >= 8 
           && (ID_EX_opcode[6:4]==3'b110 
            || ID_EX_opcode[6:4]==3'b011 
            || ID_EX_opcode[6:4]==3'b010)
         )  begin
			   ALU_rv2_in = WB_reg_data;
		end
		else begin
         ALU_rv2_in = ID_EX_Rdata2;
		end


	end

	//Instantiation of all modules used in the architecture. This allows output of 1 module to be used by another.

   //* VERIFIED
   InstMem IMEM(.Addr(PC), 
                .Size(`SIZE_WORD), 
                .DataOut(InstWord), 
                .CLK(clk));

   //* VERIFIED
   IF_ID_reg fd1(.clk(clk), 
                 .InstWord(InstWord), 
                 .pc(PC), 
                 .stall(stall),  
                 .branch_check(branch_check), 
                 .InstWord_out(IF_ID_Instword), 
                 .pc_out(IF_ID_PC)); // PC 
   
   //* VERIFIED
   Control c1(.InstWord(IF_ID_Instword), 
              .data_write_en(DWrEn), 
              .immediate(immediate), 
              .reg_write_en(RWrEn), 
              .mem_size(mem_size));
   
   
   RegFile RF(.AddrA(Rsrc1), 
              .DataOutA(Rdata1), 
              .AddrB(Rsrc2), 
              .DataOutB(Rdata2), 
              .AddrW(rd_final), 
              .DataInW(L_INST_data), 
              .WenW(~MEM_WB_reg_write_en), //negate it so 1 means write and 0 means no write
              .CLK(clk));                                                                                                                        

   ID_EX_reg de1(.clk(clk), 
                 .stall(stall), 
                 .branch_check(branch_check), 
                 .mem_size(mem_size), 
                 .shift_type(IF_ID_Instword[30]), 
                 .rs1(Rsrc1), 
                 .rs2(Rsrc2), 
                 .rd(Rdst), 
                 .funct3(funct3), 
                 .opcode(opcode), 
                 .reg_write_en(RWrEn), 
                 .rv1(Rdata1), 
                 .rv2(Rdata2), 
                 .data_write_en(DWrEn), 
                 .immediate(immediate), 
                 .pc(IF_ID_PC), 
                 .shift_type_out(ID_EX_shift_type), 
                 .rs1_out(ID_EX_Rsrc1), 
                 .rs2_out(ID_EX_Rsrc2), 
                 .rd_out(ID_EX_Rdst), 
                 .funct3_out(ID_EX_funct3), 
                 .opcode_out(ID_EX_opcode), 
                 .reg_write_en_out(ID_EX_reg_write_en), 
                 .rv1_out(ID_EX_Rdata1), 
                 .rv2_out(ID_EX_Rdata2), 
                 .data_write_en_out(ID_EX_data_write_en), 
                 .immediate_out(ID_EX_immediate), 
                 .pc_out(ID_EX_PC), 
                 .stall_out(ID_EX_stall), 
                 .mem_size_out(ID_EX_mem_size));



   ALU a1(.funct3(ID_EX_funct3), 
          .mem_size(ID_EX_mem_size), 
          .opcode(ID_EX_opcode), 
          .shift_type(ID_EX_shift_type), 
          .rv1(ALU_rv1_in), 
          .rv2(ALU_rv2_in), 
          .data_write_en(ID_EX_data_write_en), 
          .immediate(ID_EX_immediate), 
          .pc(ID_EX_PC), 
          .branch_check(EX_MEM_branch_check), 
          .stall(EX_MEM_stall), 
          .reg_data(ALU_reg_data), 
          .addr(ALU_addr), 
          .data_write_en_out(ALU_data_write_en), 
          .dw_data(ALU_dw_data), 
          .pc_new(pc_new), 
          .branch_check_out(branch_check), 
          .pc_JALR_out(pc_JALR), 
          .mem_size_out(ALU_mem_size));

     
   EX_MEM_reg em1(.clk(clk), 
                  .act(act), 
                  .mem_size(ALU_mem_size),  
                  .branch_check(branch_check), 
                  .stall(stall), 
                  .opcode(ID_EX_opcode), 
                  .funct3(ID_EX_funct3), 
                  .rd(ID_EX_Rdst), 
                  .reg_write_en(ID_EX_reg_write_en), 
                  .data_write_en(ALU_data_write_en), 
                  .addr(ALU_addr), 
                  .reg_data(ALU_reg_data), 
                  .dw_data(ALU_dw_data), 
                  .opcode_out(EX_MEM_opcode), 
                  .funct3_out(EX_MEM_funct3), 
                  .addr_out(EX_MEM_addr), 
                  .data_write_en_out(EX_MEM_data_write_en), 
                  .rd_out(EX_MEM_rd), 
                  .reg_data_out(EX_MEM_reg_data), 
                  .dw_data_out(EX_MEM_dw_data), 
                  .reg_write_en_out(EX_MEM_reg_write_en), 
                  .branch_check_out(EX_MEM_branch_check), 
                  .stall_out(EX_MEM_stall), 
                  .mem_size_out(EX_MEM_mem_size));
  
   DataMem DMEM(.Addr(EX_MEM_addr), 
                .Size(EX_MEM_mem_size), 
                .DataIn(EX_MEM_dw_data), 
                .DataOut(DMEM_data), 
                .WEN(~EX_MEM_data_write_en), 
                .CLK(clk));
    
   MEM_WB_reg mw1(.clk(clk), 
                  .reg_write_en(EX_MEM_reg_write_en), 
                  .opcode(EX_MEM_opcode), 
                  .funct3(EX_MEM_funct3), 
                  .addr(EX_MEM_addr), 
                  .rd(EX_MEM_rd), 
                  .reg_data(EX_MEM_reg_data), 
                  .dmem_data(DMEM_data), 
                  .reg_write_en_out(MEM_WB_reg_write_en), 
                  .opcode_out(MEM_WB_opcode), 
                  .funct3_out(MEM_WB_funct3), 
                  .addr_out(MEM_WB_addr), 
                  .rd_out(MEM_WB_rd), 
                  .reg_data_out(MEM_WB_reg_data), 
                  .data_out(MEM_WB_data));
   
   L_Inst l1(.opcode(MEM_WB_opcode), 
             .funct3(MEM_WB_funct3), 
             .addr(MEM_WB_addr), 
             .data(MEM_WB_data), 
             .reg_data(MEM_WB_reg_data), 
             .data_out(L_INST_data));       

   WB_reg wb1(.clk(clk), 
              .rd(MEM_WB_rd), 
              .reg_data(L_INST_data), 
              .reg_write_en(MEM_WB_reg_write_en), 
              .rd_out(WB_rd), 
              .reg_data_out(WB_reg_data), 
              .reg_write_en_out(WB_reg_write_en));

   Reg PC_REG(.Din(NPC), 
              .Qout(PC), 
              .WEN(1'b0), 
              .CLK(clk), 
              .RST(rst));

   // assign halt = (InstWord[6:0])? 0 : (WB_rd && PC >20)? 0 : 1;
   assign halt = (PC < 1000)? 0: 1;
endmodule // PipelinedCPU

 

//* VERIFIED
module IF_ID_reg(
   input clk, 
   input stall, 
   input branch_check, 
   input [31:0] InstWord, 
   input [31:0] pc, // PC 
   output reg [31:0] InstWord_out, 
   output reg [31:0] pc_out
); 
   
   // Check if stalling 
   always @(posedge clk) begin
      if (stall == 1) begin // if not stalling
         // If not stalling and branch_check = 1, then add NOP aka (ADDI R0, R0, 0)
         InstWord_out = branch_check ? (32'b00000000000000000000000000010011) : (InstWord);
         pc_out = pc;
         
      end
   end
endmodule


//* VERIFIED
module Control(
   input [31:0] InstWord, //IF_ID_InstWord
   output reg data_write_en,
   output reg signed [31:0] immediate,
   output reg reg_write_en,
   output reg [1:0] mem_size
);
wire [6:0]  opcode;
assign opcode = InstWord[6:0];
wire [2:0] funct3;
assign funct3 = InstWord[14:12];

always @(*) begin
   mem_size = `SIZE_BYTE;
      case (opcode) 
         `OPCODE_COMPUTE: begin
            reg_write_en = 1; //write to register
            data_write_en = 0; // dont write to mem
             // won't write since write enable is off
         end
         `OPCODE_COMPUTE_IMM: begin
            immediate = {{20{InstWord[31]}},InstWord[31:20]}; //sext(imm[31:20])
            reg_write_en = 1; //write to register
            data_write_en = 0; //don't write to memory
            // won't write since write enable is off
         end
         `OPCODE_BRANCH: begin
            immediate = {{20{InstWord[31]}},InstWord[31],InstWord[7],InstWord[30:25],InstWord[11:8],1'b0};
            reg_write_en = 0;
            data_write_en = 0;
             // won't write since write enable is off
         end
         `OPCODE_LOAD: begin
            immediate = {{20{InstWord[31]}},InstWord[31:20]}; // sext(imm[31:20])
            reg_write_en = 1;
            data_write_en = 0; // DONT WRITE TO MEMORY, just load from memory
            case (funct3) // Decides size of memory to load based on funct3
               `LB: mem_size = `SIZE_BYTE;
               `LBU: mem_size = `SIZE_BYTE;
               `LHU: mem_size = `SIZE_HWORD;
               `LH: mem_size = `SIZE_HWORD;
               `LW: mem_size = `SIZE_WORD;
            endcase
         end
         `OPCODE_STORE: begin
            immediate = {{20{InstWord[31]}},InstWord[31:25],InstWord[11:7]};
            reg_write_en = 0;
            data_write_en = 1;
            case (funct3) // Decides size of memory to store based on funct3
               `SB: mem_size = `SIZE_BYTE;
               `SH: mem_size = `SIZE_HWORD;
               `SW: mem_size = `SIZE_WORD;
            endcase
         end
         `OPCODE_LUI: begin
            immediate = {InstWord[31:12],12'b0};
            reg_write_en = 1;
            data_write_en = 0;
         end
         `OPCODE_AUIPC: begin
            immediate = {InstWord[31:12],12'b0};
            reg_write_en = 1;
            data_write_en = 0;
         end
         `OPCODE_JAL: begin
            immediate = {{11{InstWord[31]}},InstWord[31],InstWord[19:12],InstWord[20],InstWord[30:21],1'b0};
            reg_write_en = 1;
            data_write_en = 0;
         end
         `OPCODE_JALR: begin
            immediate = {{20{InstWord[31]}},InstWord[31:20]};
            reg_write_en = 1;
            data_write_en = 0;
         end
      endcase
   end
endmodule



//* VERIFIED
module ID_EX_reg(
   input clk,
   input stall,
   input branch_check,
   // Register File Inputs
   input shift_type,
   input [4:0] rs1,
   input [4:0] rs2,
   input [4:0] rd,
   input [2:0] funct3,
   input [6:0] opcode,
   input reg_write_en,
   input [31:0] rv1,
   input [31:0] rv2,  
   input [1:0] mem_size,
   // Control Block Inputs
   input data_write_en,
   input signed [31:0] immediate,
   input [31:0] pc,
   // Register File Outputs
   output reg shift_type_out, 
   output reg [4:0] rs1_out,
   output reg [4:0] rs2_out,
   output reg [4:0] rd_out,
   output reg [2:0] funct3_out,
   output reg [6:0] opcode_out,
   output reg reg_write_en_out ,
   output reg [31:0] rv1_out,
   output reg [31:0] rv2_out,
   // Control block outputs
   output reg data_write_en_out,
   output reg signed [31:0] immediate_out,
   output reg [31:0] pc_out,
   output reg stall_out,
   output reg [1:0] mem_size_out
);
   always@(posedge clk)
   begin
      if (stall == 1) begin // if NOT stalling
         rs1_out = rs1;
         rs2_out = rs2;
         rd_out = rd;
         funct3_out = funct3;
         opcode_out = opcode;
         immediate_out = immediate;
         pc_out = pc;
         reg_write_en_out = reg_write_en & !branch_check & stall;
         data_write_en_out = data_write_en & !branch_check & stall;
         rv1_out = rv1;
         rv2_out = rv2;
         shift_type_out = shift_type;
         stall_out = stall;
         mem_size_out = mem_size;
      end
   end
endmodule

module ALU(
   input [2:0] funct3,
   input [6:0] opcode,
   input shift_type,
   input [31:0] rv1,
   input [31:0] rv2,
   input data_write_en, 
   input signed [31:0] immediate,
   input [31:0] pc,
	input branch_check,
	input stall,
   input [1:0] mem_size,
   output reg [31:0] reg_data,
   output reg [31:0] addr,
   output reg data_write_en_out,
   output reg [31:0] dw_data,
	output reg [31:0] pc_new,
	output reg branch_check_out,
	output reg pc_JALR_out,
   output reg [1:0] mem_size_out
);

   wire [31:0] reg_data_R, reg_data_I, pc_val;
   wire jump_flag;
   initial begin
      pc_new = 0;
      branch_check_out = 0;
      pc_JALR_out = 0;
      mem_size_out = mem_size;
      addr = 0;
      dw_data = 0;
   end

   always@(*) begin
      dw_data = 0;
      addr = 0;
      pc_new = 0;
      branch_check_out = 0;
      pc_JALR_out = 0;
      mem_size_out = mem_size;

      case(opcode)
         `OPCODE_COMPUTE: begin
            reg_data = reg_data_R; //result of R-type (so two register) ALU computation
            data_write_en_out = data_write_en;
         end
         `OPCODE_COMPUTE_IMM: begin
            reg_data = reg_data_I; //Result of immediate ALU computation
            data_write_en_out = data_write_en;
         end
         `OPCODE_BRANCH: begin
            pc_new = pc_val; //new pc computed by the B Type module ALU computation
            data_write_en_out = data_write_en;
            branch_check_out = (branch_check | !stall) ? 0 : jump_flag; //jump flag comes from B Type Module
         end
         `OPCODE_LOAD: begin
            addr = rv1 + immediate; //address to get the data from, to later be written to the desired register (rd)
            data_write_en_out = data_write_en;
         end
         `OPCODE_STORE: begin
            addr = rv1 + immediate; // rs1 + immediate (or minus if negative)
            dw_data = rv2; // Since mem_size is taken care of by mem_size, we just input rs2 data into dw_data to later be written
                           // to the data memory by DMEM
            data_write_en_out = data_write_en;
         end
         `OPCODE_JALR: begin
            reg_data = pc + 4; // 4 is length(inst) and reg_data is what is written to resgister later
            pc_new = (rv1 + immediate) & (~1); //pc <- (rs1 + offset) and last bit set to zero
            // 1 = 00....01  -> ~1 = 111...1110 so last bit changes to zero when we AND it 
            ///(rv1 + immediate) & 32'hfffffffe; 
            data_write_en_out = data_write_en;
            branch_check_out = (branch_check | !stall)? 0:1;
            pc_JALR_out = 1; // this is set to true because this instrction is JALR instructions
         end
         `OPCODE_JAL: begin
            reg_data = pc + 4; // 4 is length(inst)
            pc_new = pc + immediate; // pc <- pc + offset (previously had as -12)
            data_write_en_out = data_write_en;
            branch_check_out = (branch_check | !stall)?0:1;
         end
         `OPCODE_AUIPC: begin
            reg_data = pc + immediate; //pc + sext(immediate[31:12] << 12)
            data_write_en_out = data_write_en;
         end
         `OPCODE_LUI: begin
            reg_data = immediate; // sext(immediate[31:12] << 12)
            data_write_en_out = data_write_en;
         end
      endcase
   // $display("ALU BRANCH:%01b", branch_check_out);
   // $display("reg_data:%d", reg_data);

 end
   // wire stall_out;
   R_inst r1(.funct3(funct3), .shift_type(shift_type), .opA(rv1), .opB(rv2), .out(reg_data_R));
   I_inst i1(.funct3(funct3), .shift_type(shift_type), .opA(rv1), .imm(immediate), .out(reg_data_I));
   B_inst b1(.funct3(funct3), .addr(pc), .imm(immediate), .opA(rv1), .opB(rv2), .out(pc_val), .stall_out(jump_flag));
endmodule



//* VERIFIED
module EX_MEM_reg (
   input clk, act, branch_check, stall,
   // FROM ID_EX
   input [6:0] opcode,
   input [2:0] funct3,
   input [4:0] rd,
   input reg_write_en,

   // FROM ALU
   input data_write_en,
   input [31:0] addr, //data addr
   input [31:0] reg_data,
   input [31:0] dw_data,
   input [1:0] mem_size,

   //OUTPUTS
   output reg [6:0] opcode_out,
   output reg [2:0] funct3_out,
   output reg [31:0] addr_out,
   output reg data_write_en_out,
   output reg [4:0] rd_out,
   output reg [31:0] reg_data_out,
   output reg [31:0] dw_data_out,
   output reg reg_write_en_out, branch_check_out, stall_out,
   output reg [1:0] mem_size_out
);

   always @(clk) begin
      opcode_out = opcode;
      funct3_out = funct3;
      addr_out = addr;
      data_write_en_out = data_write_en & act; //act sets controls to zero if ALU follows LD to prevent register being written to or data mem
      reg_write_en_out = reg_write_en & act;
      rd_out = rd;
      reg_data_out = reg_data;
      dw_data_out = dw_data;
      branch_check_out = branch_check;
      stall_out = stall;
      mem_size_out = mem_size;
   end   
endmodule

//* VERIFIED
module WB_reg (
   input clk,
   //FROM MEM_WB
   input [4:0] rd,
   input reg_write_en,

   // FROM L_INST
   input [31:0] reg_data,

   // OUTPUTS
   output reg [4:0] rd_out,
   output reg [31:0] reg_data_out,
   output reg reg_write_en_out
);

   always @ (posedge clk) begin
      rd_out = rd;
      reg_data_out = reg_data;
      reg_write_en_out = reg_write_en;
   end
endmodule


//* VERIFIED
module MEM_WB_reg (
   input clk, 
   // FROM EX_MEM
   input reg_write_en,
   input [6:0] opcode,
   input [2:0] funct3,
   input [31:0] addr,
   input [4:0] rd,
   input [31:0] reg_data,
   // FROM DMEM
   input [31:0] dmem_data,

   //OUTPUTS
   output reg reg_write_en_out,
   output reg [6:0] opcode_out,
   output reg [2:0] funct3_out,
   output reg [31:0] addr_out,
   output reg [4:0] rd_out,
   output reg [31:0] reg_data_out,
   output reg [31:0] data_out
);
   always@(clk)
   begin 
      funct3_out = funct3;
      opcode_out = opcode;
      addr_out = addr;
      reg_write_en_out = reg_write_en;
      reg_data_out = reg_data;
      rd_out = rd;
      data_out = dmem_data;
   end
endmodule

// Crosscheck to make sure type of load corresponds to correct construction
// Write to reg
module L_Inst (
   input [6:0] opcode,
   input [2:0] funct3,
   input [31:0] addr,
   input [31:0] data,
   input [31:0] reg_data,
   output reg[31:0] data_out
);

   reg [31:0] offset;

   always @(*) begin
      if (opcode != `OPCODE_LOAD) data_out = reg_data;
      else 
      begin //only if load instruction
         case (funct3)
         `LB: data_out = {{24{1'b0}},data[7:0]};
         `LH: data_out = {{16{1'b0}}, data[15:0]};
         `LW: data_out = data;
         `LBU: data_out = {{24{1'b0}},data[7:0]};
         `LHU: data_out = {{16{1'b0}}, data[15:0]};
         endcase
      end
   end
endmodule


module R_inst(
    input [2:0] funct3,
    input shift_type,
    input signed [31:0] opA,
    input signed [31:0] opB,
    output reg[31:0] out
);

    wire [31:0] opA_temp;
    wire [31:0] opB_temp;
    assign opA_temp = opA;
    assign opB_temp = opB;

    always @(*) 
    begin
    	case({shift_type,funct3}) //Easy way to check funt3 and the one bit difference in funct7
        4'b0000:    out = opA + opB;          //add
        4'b1000:    out = opA - opB;          //sub
        4'b0001:    out = opA << opB[4:0];	  //sll
        4'b0010:    out = opA < opB;          //slt
        4'b0011:    out = opA_temp < opA_temp;        //sltu
        4'b0100:    out = opA ^ opB;          //xor
        4'b0101:    out = opA >> opB[4:0];    //srl
        4'b1101:    out = opA >>> opB[4:0];   //sra
        4'b0110:    out = opA | opB;          //or
        4'b0111:    out = opA & opB;          //and
        endcase
        $display("OPA:%d, OPB:%d, Result:%d", opA, opB, out);
    end
endmodule



module I_inst (
   input [2:0] funct3,
   input shift_type, // Right or left shift indicator
   input signed [31:0] opA,
   input signed [31:0] imm,
   output reg [31:0] out
);
   wire [31:0] opA_temp; //unsigned
   wire [11:0] imm_temp; //unsigned
   assign opA_temp = opA;
   assign imm_temp = imm;

   always @(*) begin
      case (funct3)
         3'b000: out = opA + imm;
         3'b001: out = opA << imm[4:0]; //slli
         3'b010: out = opA < imm;
         3'b011: out = opA_temp < imm_temp;
         3'b100: out = opA ^ imm;
         3'b101: begin
            //imm[4:0] = shamt
            if (shift_type) out = opA >> imm[4:0]; //SRA
            else out = opA >>> imm[4:0]; //srl
         end
         3'b110: out = opA | imm;
         3'b111: out = opA & imm;
      endcase
   $display("OPA:%d, OPB:%d, Result:%d", opA, imm, out);
   end
endmodule



module B_inst (
   input [2:0] funct3,
   input [31:0] addr,
   input signed [31:0] imm, opA, opB,
   output reg [31:0] out,
   output reg stall_out
);
   wire [31:0] opA_temp, opB_temp;
   assign opA_temp = opA;
   assign opB_temp = opB;

   always @(*) begin
      //default
      out = addr;
      stall_out = 0;


      case (funct3)
      `BEQ: begin
         out = (opA == opB) ? (addr + imm)  : 0;
         stall_out = (opA == opB) ? 1 : 0;
      end
      `BNE: begin
         out = (opA != opB) ? (addr + imm)  : 0;
         stall_out = (opA != opB) ? 1 : 0;
      end
      `BLT: begin
         out = (opA < opB) ? (addr + imm) : 0;
         stall_out = (opA < opB) ? 1 : 0;
      end
      `BGE: begin
         out = (opA >= opB) ? (addr + imm)  : (0);
         stall_out = (opA >= opB) ? 1 : 0;
      end
      `BLTU: begin
         out = (opA_temp < opB_temp) ? (addr + imm)  : 0;
         stall_out = (opA_temp < opB_temp) ? 1 : 0;
      end
      `BGEU: begin
         out = (opA_temp >= opB_temp) ? (addr + imm)  : (0);
         stall_out = (opA_temp >= opB_temp) ? 1 : 0;
      end
      endcase
   end
endmodule 
