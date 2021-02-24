/* ********************************************
 *  COSE222 Lab #4
 *
 *  Module: pipelined_cpu.sv
 *  - Top design of the 5-stage pipelined RISC-V processor
 *  - Processor supports instructions described in Chapter 4 of COD book
 *
 *  Author: Gunjae Koo (gunjaekoo@korea.ac.kr)
 *
 * ********************************************
 */

`timescale 1ns/1ps
`define FF 1    // Flip-flop delay for just better waveform view

// Packed structures for pipeline registers
// Pipe reg: IF/ID
typedef struct packed {
    logic   [63:0]  pc;
    logic   [31:0]  inst;
} pipe_if_id;

// Pipe reg: ID/EX
typedef struct packed {
    logic   [63:0]  rs1_dout;
    logic   [63:0]  rs2_dout;
    logic   [63:0]  imm64;
    logic   [2:0]   funct3;
    logic   [6:0]   funct7;
    logic           branch;
    logic           alu_src;
    logic   [1:0]   alu_op;
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rs1;
    logic   [4:0]   rs2;
    logic   [4:0]   rd;         // rd for regfile
    logic           reg_write;
    logic           mem_to_reg;
} pipe_id_ex;

// Pipe reg: EX/MEM
typedef struct packed {
    logic   [63:0]  alu_result; // for address
    logic   [63:0]  rs2_dout;   // for store
    logic           mem_read;
    logic           mem_write;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_ex_mem;

// Pipe reg: MEM/WB
typedef struct packed {
    logic   [63:0]  alu_result;
    logic   [63:0]  dmem_dout;
    logic   [4:0]   rd;
    logic           reg_write;
    logic           mem_to_reg;
} pipe_mem_wb;

module pipeline_cpu
#(  parameter IMEM_DEPTH = 1024,    // imem depth (default: 1024 entries = 4 KB)
              IMEM_ADDR_WIDTH = 10,
              REG_WIDTH = 64,
              DMEM_DEPTH = 1024,    // dmem depth (default: 1024 entries = 8 KB)
              DMEM_ADDR_WIDTH = 10 )
(
    input           clk,            // System clock
    input           reset_b         // Asychronous negative reset
);

    // -------------------------------------------------------------------
    /* Instruction fetch stage:
     * - Accessing the instruction memory with PC
     * - Control PC udpates for pipeline stalls
     */

    // Program counter
    logic           pc_write;   // enable PC updates
    logic   [63:0]  pc_curr, pc_next;
    logic   [63:0]  pc_next_plus4, pc_next_branch;
    logic           branch;
    logic           regfile_zero;   // zero detection from regfile

    assign pc_next_plus4 = pc_curr + 3'd4;
    assign pc_next = (branch && regfile_zero)? pc_next_branch: pc_next_plus4;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            pc_curr <= 'b0;
        end else begin 
            if(pc_write)begin
            pc_curr <= pc_next;
            end
        end
    end

    // imem
    logic   [IMEM_ADDR_WIDTH-1:0]   imem_addr;
    logic   [31:0]  inst;   // instructions = an output of ????
    
    assign imem_addr = pc_curr[IMEM_ADDR_WIDTH+1:2]; 

    // instantiation: instruction memory
    imem #(
        .IMEM_DEPTH         (IMEM_DEPTH),
        .IMEM_ADDR_WIDTH    (IMEM_ADDR_WIDTH)
    ) u_imem_0 (
        .addr               ( imem_addr     ),
        .dout               ( inst          )
    );
    // -------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* IF/ID pipeline register
     * - Supporting pipeline stalls and flush
     */
    pipe_if_id      id;         // THINK WHY THIS IS ID...
    logic           if_flush, if_stall;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            id <= 'b0;
        end else begin
            if (if_flush) begin
                id <= #(`FF) 'b0;
            end else if (~if_stall) begin  
                id.pc <= #(`FF) pc_curr;
                id.inst <= #(`FF) inst;
            end
        end
    end
    // -------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Instruction decoder stage:
     * - Generating control signals
     * - Register file
     * - Immediate generator
     * - Hazard detection unit
     */
    
    // -------------------------------------------------------------------
    /* Main control unit:
     * Main control unit generates control signals for datapath elements
     * The control signals are determined by decoding instructions
     * Generating control signals using opcode = inst[6:0]
     */
    logic   [6:0]   opcode;
    //logic           branch;
    logic           alu_src, mem_to_reg;
    logic   [1:0]   alu_op;
    logic           mem_read, mem_write, reg_write; // declared above

    // COMPLETE THE MAIN CONTROL UNIT HERE

    assign opcode = id.inst[6:0];
    assign branch = (opcode==7'b1100011)? 1'b1:1'b0; //branch
    assign mem_read = (opcode==7'b0000011)? 1'b1:1'b0; //ld
    assign mem_write = (opcode==7'b0100011)? 1'b1:1'b0; //sd
    assign mem_to_reg = mem_read;
    assign reg_write = (opcode==7'b0110011) | mem_read; //ld or R-type
    assign alu_src = (mem_read | mem_write)? 1'b1:1'b0; // ld or sd

    assign alu_op[0] = branch;
    assign alu_op[1] = (opcode==7'b0110011); // R-type

    // --------------------------------------------------------------------

    // ---------------------------------------------------------------------
    /* Immediate generator:
     * Generating immediate value from inst[31:0]
     */
    logic   [63:0]  imm64;
    logic   [63:0]  imm64_branch;  // imm64 left shifted by 1

    // COMPLETE IMMEDIATE GENERATOR HERE
    logic   [11:0]  imm12;

    assign imm12 = (branch)?{id.inst[31], id.inst[7], id.inst[30:25], id.inst[11:8]}:
                    ((mem_read)? id.inst[31:20]: {id.inst[31:25],id.inst[11:7]} );
    assign imm64 = { {52{imm12[11]}}, imm12};
    assign imm64_branch = {imm64[62:0], 1'b0}; 

    // Computing branch target
    assign pc_next_branch = id.pc + imm64_branch;

    // ----------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Hazard detection unit
     * - Detecting data hazards from load instrcutions
     * - Detecting control hazards from taken branches
     */
    logic   [4:0]   rs1, rs2;

    logic           stall_by_load_use;
    logic   [1:0]   stall_by_regwr_branch;   // branch result is decided in ID stage, this is not explained in the textbook
    logic           flush_by_branch;
    
    logic           id_stall;

    pipe_id_ex      ex; 
    assign stall_by_load_use = ((ex.mem_read) && 
                                ((ex.rd == id.inst[19:15]) || (ex.rd == id.inst[24:20])))? 1'b1: 1'b0;
    
    
    assign stall_by_regwr_branch[0] = (branch && ex.reg_write && 
                                        ((ex.rd == id.inst[19:15])|| (ex.rd == id.inst[24:20])))?1'b1: 1'b0;

    
    assign stall_by_regwr_branch[1] = (branch && mem.reg_write &&
                                        ((mem.rd == id.inst[19:15]) || (mem.rd == id.inst[24:20])))?1'b1: 1'b0; 

    assign flush_by_branch = regfile_zero;   //FLUSH CONDITION- branch instruction is taken

    assign id_stall = |stall_by_regwr_branch | stall_by_load_use;

    assign if_flush = branch && flush_by_branch; // IF stage is flushed
    assign if_stall = id_stall;   // IF stage is stalled

    assign pc_write = ~id_stall; // pc_write == 0, pc is not updated

    // ----------------------------------------------------------------------


    // regfile
    pipe_mem_wb     wb;

    logic   [4:0]   rd;    // register numbers
    logic   [REG_WIDTH-1:0] rd_din;
    logic   [REG_WIDTH-1:0] rs1_dout, rs2_dout;
    
    assign rs1 = id.inst[19:15];
    assign rs2 = id.inst[24:20];
    assign rd = id.inst[11:7];
    // rd, rd_din, and reg_write will be determined in WB stage
    
    // instantiation of register file
    regfile #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_regfile_0 (
        .clk                (clk),
        .rs1                (id.inst[19:15]),
        .rs2                (id.inst[24:20]),
        .rd                 (wb.rd),
        .rd_din             (rd_din),
        .reg_write          (wb.reg_write),
        .rs1_dout           (rs1_dout),
        .rs2_dout           (rs2_dout)
    );

    assign regfile_zero = ~|(rs1_dout ^ rs2_dout); // NOR(bitwiseXOR) 

    // ------------------------------------------------------------------

    // -------------------------------------------------------------------
    /* ID/EX pipeline register
     * - Supporting pipeline stalls
     */
    // pipe_id_ex      ex;         // THINK WHY THIS IS EX...
    logic   [6:0]   funct7;
    logic   [2:0]   funct3;

    // THE FOLLOWING SIGNALS WILL BE USED FOR ALU CONTROL
    assign funct7 = ex.funct7;
    assign funct3 = ex.funct3;

    // COMPLETE ID/EX PIPELINE REGISTER
    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            ex <= 'b0;
        end else begin
            ex.rs1_dout <= rs1_dout;
            ex.rs2_dout <= rs2_dout;
            ex.imm64 <= imm64;
            ex.funct3 <= id.inst[14:12];
            ex.funct7 <= id.inst[31:25];
            ex.branch <= branch;
            ex.alu_src <= alu_src;
            ex.alu_op <= alu_op;
            ex.mem_read <= mem_read;
            ex.mem_write <= mem_write;
            ex.rs1 <= id.inst[19:15];
            ex.rs2 <= id.inst[24:20];
            ex.rd <= id.inst[11:7];
            ex.reg_write <= reg_write;
            ex.mem_to_reg <= mem_to_reg;
        end
    end

    // ------------------------------------------------------------------

    // ------------------------------------------------------------------
    /* Excution stage:
     * - ALU & ALU control
     * - Data forwarding unit
     */

    // --------------------------------------------------------------------
    /* ALU control unit:
     * ALU control unit generate alu_control signal which selects ALU operations
     * Generating control signals using alu_op, funct7, and funct3 fileds
     */

    logic   [3:0]   alu_control;    // ALU control signal

    // COMPLETE ALU CONTROL UNIT

     always_comb begin
        if(ex.alu_op[1])begin
            case(ex.funct3)
                3'b111: alu_control = 4'b0000;
                3'b110: alu_control = 4'b0001;
                default: alu_control = (ex.funct7[5])? 4'b0110:4'b0010;
            endcase
        end else begin
            alu_control = (ex.alu_op[0])? 4'b0110: 4'b0010;
        end
    end

    // ---------------------------------------------------------------------

    // ----------------------------------------------------------------------
    /* Forwarding unit:
     * - Forwarding from EX/MEM and MEM/WB
     */
    logic   [1:0]   forward_a, forward_b;
    logic   [63:0]  alu_fwd_in1, alu_fwd_in2;   // outputs of forward MUXes
    pipe_ex_mem     mem;

    // COMPLETE FORWARDING MUXES

    assign alu_fwd_in1 = (forward_a == 2'b10)? mem.alu_result : 
                         (forward_a == 2'b01)? rd_din : ex.rs1_dout; // 00
    assign alu_fwd_in2 = (forward_b == 2'b10)? mem.alu_result : 
                         (forward_b == 2'b01)? rd_din : ex.rs2_dout; // 00


    // COMPLETE THE FORWARDING UNIT
    // Need to prioritize forwarding conditions

    assign forward_a = ((mem.reg_write) && (mem.rd != 0) && (mem.rd == ex.rs1)) ? 2'b10 : 
                        ((wb.reg_write) && (wb.rd != 0) 
                            && !((mem.reg_write) && (mem.rd != 0) && (mem.rd == ex.rs1))
                            && (wb.rd == ex.rs1))? 2'b01: 2'b00;
    assign forward_b = ((mem.reg_write) && (mem.rd != 0) && (mem.rd == ex.rs2)) ? 2'b10 : 
                        ((wb.reg_write) && (wb.rd != 0) 
                            && !((mem.reg_write) && (mem.rd != 0) && (mem.rd == ex.rs2))
                            && (wb.rd == ex.rs2))? 2'b01: 2'b00;


    // -----------------------------------------------------------------------

    // ALU
    logic   [REG_WIDTH-1:0] alu_in1, alu_in2;
    logic   [REG_WIDTH-1:0] alu_result;
    logic           alu_zero;   // will not be used

    assign alu_in1 = alu_fwd_in1; 
    assign alu_in2 = (ex.alu_src)? ex.imm64 : alu_fwd_in2;

    // instantiation: ALU
    alu #(
        .REG_WIDTH          (REG_WIDTH)
    ) u_alu_0 (
        .in1                (alu_in1),
        .in2                (alu_in2),
        .alu_control        (alu_control),
        .result             (alu_result),
        .zero               (alu_zero)
    );

    // -------------------------------------------------------------------------
    /* EX/MEM pipeline register
     */
    // pipe_ex_mem     mem;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            mem <= 'b0;
        end else begin
            mem.alu_result <= alu_result;
            mem.rs2_dout <= alu_fwd_in2;
            mem.mem_read <= ex.mem_read;
            mem.mem_write <= ex.mem_write;
            mem.rd <= ex.rd;
            mem.reg_write <= ex.reg_write; 
            mem.mem_to_reg <= ex.mem_to_reg;
        end
    end


    // --------------------------------------------------------------------------
    /* Memory srage
     * - Data memory accesses
     */

    // dmem
    logic   [DMEM_ADDR_WIDTH-1:0]    dmem_addr;
    logic   [63:0]  dmem_din, dmem_dout;

    assign dmem_addr = mem.alu_result[DMEM_ADDR_WIDTH+2:3];
    assign dmem_din = mem.rs2_dout;
    
    // instantiation: data memory
    dmem #(
        .DMEM_DEPTH         (DMEM_DEPTH),
        .DMEM_ADDR_WIDTH    (DMEM_ADDR_WIDTH)
    ) u_dmem_0 (
        .clk                (clk),
        .addr               (dmem_addr),
        .din                (dmem_din),
        .mem_read           (mem.mem_read),
        .mem_write          (mem.mem_write),
        .dout               (dmem_dout)
    );


    // -----------------------------------------------------------------------
    /* MEM/WB pipeline register
     */

    //pipe_mem_wb         wb;

    always_ff @ (posedge clk or negedge reset_b) begin
        if (~reset_b) begin
            wb <= 'b0;
        end else begin
            wb.alu_result <= mem.alu_result;
            wb.dmem_dout <= dmem_dout;
            wb.rd <= mem.rd;
            wb.reg_write <= mem.reg_write;
            wb.mem_to_reg <= mem.mem_to_reg;
        end
    end

    // ----------------------------------------------------------------------
    /* Writeback stage
     * - Write results to regsiter file
     */
    
    assign rd_din = (wb.mem_to_reg)?dmem_dout:wb.alu_result;

endmodule