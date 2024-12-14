// See LICENSE for license details.

package mini

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._

object Consts {
  val PC_START = 0x200
  val PC_EVEC = 0x100
}

class DatapathIO(xlen: Int) extends Bundle {
  val host = new HostIO(xlen)
  val icache = Flipped(new CacheIO(xlen, xlen))
  val dcache = Flipped(new CacheIO(xlen, xlen))
  val ctrl = Flipped(new ControlSignals)
}

class FetchDecodePipelineRegister(xlen: Int) extends Bundle {
  val inst = chiselTypeOf(Instructions.NOP)
  val pc = UInt(xlen.W)
}

class DecodeExecutePipelineRegister(xlen: Int) extends Bundle {
  val inst = chiselTypeOf(Instructions.NOP)
  val pc = UInt(xlen.W)
  val alu_a = UInt(xlen.W)
  val alu_b = UInt(xlen.W)
  val alu_op = UInt(4.W)
  val csr_in = UInt(xlen.W)
  val st_type = UInt(2.W)
  val csr_cmd = UInt(3.W)
  val illegal = Bool()
  val pc_check = Bool()
  val ld_type = UInt(3.W)
  val wb_sel = UInt(2.W)
  val wb_en = Bool()
  val taken = Bool()
  val pc_sel = UInt(2.W)
}

class ExecuteMemoryPipelineRegister(xlen: Int) extends Bundle {
  val inst = chiselTypeOf(Instructions.NOP)
  val pc = UInt(xlen.W)
  val alu = UInt(xlen.W)
  val csr_in = UInt(xlen.W)
  val st_type = UInt(2.W)
  val csr_cmd = UInt(3.W)
  val illegal = Bool()
  val pc_check = Bool()
  val ld_type = UInt(3.W)
  val wb_sel = UInt(2.W)
  val wb_en = Bool()
  val pc_sel = UInt(2.W)
}

class MemoryWritebackPipelineRegister(xlen: Int) extends Bundle {
  val inst = chiselTypeOf(Instructions.NOP)
  val pc = UInt(xlen.W)
  val ld_type = UInt(3.W)
  val wb_sel = UInt(2.W)
  val wb_en = Bool()
  val alu = UInt(xlen.W)
  val load = SInt(xlen.W)
  val csr_out = UInt(xlen.W)
}

class Datapath(val conf: CoreConfig) extends Module {
  val io = IO(new DatapathIO(conf.xlen))
  val csr = Module(new CSR(conf.xlen))
  val regFile = Module(new RegFile(conf.xlen))
  val alu = Module(conf.makeAlu(conf.xlen))
  val immGen = Module(conf.makeImmGen(conf.xlen))
  val brCond = Module(conf.makeBrCond(conf.xlen))

  import Control._

  /** Pipeline State Registers * */

  /** *** Fetch / Decode Registers ****
    */
  val fd_reg = RegInit(
    (new FetchDecodePipelineRegister(conf.xlen)).Lit(
      _.inst -> Instructions.NOP,
      _.pc -> 0.U
    )
  )

  /** *** Decode / Execute Registers ****
    */
  val de_reg = RegInit(
    (new DecodeExecutePipelineRegister(conf.xlen)).Lit(
      _.inst -> Instructions.NOP,
      _.pc -> 0.U,
      _.alu_a -> 0.U,
      _.alu_b -> 0.U,
      _.alu_op -> 0.U,
      _.csr_in -> 0.U,
      _.st_type -> 0.U,
      _.csr_cmd -> 0.U,
      _.illegal -> false.B,
      _.pc_check -> false.B,
      _.ld_type -> 0.U,
      _.wb_sel -> 0.U,
      _.wb_en -> false.B,
      _.taken -> false.B,
      _.pc_sel -> 0.U   // has better way than just pipelining the pc_sel
    )
  )

  /** *** Execute / Memory Registers ****
    */
  val em_reg = RegInit(
    (new ExecuteMemoryPipelineRegister(conf.xlen)).Lit(
      _.inst -> Instructions.NOP,
      _.pc -> 0.U,
      _.alu -> 0.U,
      _.csr_in -> 0.U,
      _.st_type -> 0.U,
      _.csr_cmd -> 0.U,
      _.illegal -> false.B,
      _.pc_check -> false.B,
      _.ld_type -> 0.U,
      _.wb_sel -> 0.U,
      _.wb_en -> false.B,
      _.pc_sel -> 0.U
    )
  )

  /** *** Memory / Writeback Registers ****
    */  
  val mw_reg = RegInit(
    (new MemoryWritebackPipelineRegister(conf.xlen)).Lit(
      _.inst -> Instructions.NOP,
      _.pc -> 0.U,
      _.ld_type -> 0.U,
      _.alu -> 0.U,
      _.load -> 0.S,
      _.wb_sel -> 0.U,
      _.wb_en -> false.B,
      _.csr_out -> 0.U
    )
  )

  /** **** Fetch ****
    */
  val started = RegNext(reset.asBool)
  val stall = !io.icache.resp.valid || !io.dcache.resp.valid
  val pc = RegInit(Consts.PC_START.U(conf.xlen.W) - 4.U(conf.xlen.W))
  // Next Program Counter
  val next_pc = MuxCase(
    pc + 4.U,
    IndexedSeq(
      stall -> pc,
      csr.io.expt -> csr.io.evec,
      (em_reg.pc_sel === PC_EPC) -> csr.io.epc,
      ((de_reg.pc_sel === PC_ALU) || (de_reg.taken)) -> (alu.io.sum >> 1.U << 1.U),
      (io.ctrl.pc_sel === PC_0) -> pc
    )
  )

  // nop instruction is inserted here
  val inst =
    Mux(started || io.ctrl.inst_kill || (io.ctrl.pc_sel === PC_ALU) || de_reg.taken || brCond.io.taken 
    || csr.io.expt || de_reg.pc_sel === PC_ALU || de_reg.pc_sel === PC_EPC || em_reg.pc_sel === PC_EPC, 
    Instructions.NOP, io.icache.resp.bits.data)
  pc := next_pc
  io.icache.req.bits.addr := next_pc
  io.icache.req.bits.data := 0.U
  io.icache.req.bits.mask := 0.U
  io.icache.req.valid := !stall
  io.icache.abort := false.B

  // Pipelining Fe/De
  // fd_reg reset when branch prediction miss
  
  // when((de_reg.taken) || (io.ctrl.pc_sel === PC_ALU)) {
  //   fd_reg.pc := 0.U
  //   fd_reg.inst := Instructions.NOP
  // }.else
  when(!stall) { 
    fd_reg.pc := pc
    fd_reg.inst := inst
  }


  /** **** Decode ****
    */
  io.ctrl.inst := fd_reg.inst

  // regFile read
  val rd_addr = fd_reg.inst(11, 7)
  val rs1_addr = fd_reg.inst(19, 15)
  val rs2_addr = fd_reg.inst(24, 20)
  regFile.io.raddr1 := rs1_addr
  regFile.io.raddr2 := rs2_addr

  // gen immdeates
  immGen.io.inst := fd_reg.inst
  immGen.io.sel := io.ctrl.imm_sel

  // bypass signal
  val load = Wire(SInt(conf.xlen.W))
  val regWrite = Wire(UInt(conf.xlen.W))

  // bypass ex/de
  val de_rd_addr = de_reg.inst(11, 7)
  val de_rs1hazard = de_reg.wb_en && rs1_addr.orR && (rs1_addr === de_rd_addr)
  val de_rs2hazard = de_reg.wb_en && rs2_addr.orR && (rs2_addr === de_rd_addr)

  // bypass me/de
  val em_rd_addr = em_reg.inst(11, 7)
  val em_rs1hazard = em_reg.wb_en && rs1_addr.orR && (rs1_addr === em_rd_addr)
  val em_rs2hazard = em_reg.wb_en && rs2_addr.orR && (rs2_addr === em_rd_addr)
  val em_regWrite =
    MuxLookup(em_reg.wb_sel, em_reg.alu.zext)(
      Seq(WB_MEM -> load, WB_PC4 -> (em_reg.pc + 4.U).zext, WB_CSR -> csr.io.out.zext)
    ).asUInt

  // bypass wb/de
  val mw_rd_addr = mw_reg.inst(11, 7)
  val mw_rs1hazard = mw_reg.wb_en && rs1_addr.orR && (rs1_addr === mw_rd_addr)
  val mw_rs2hazard = mw_reg.wb_en && rs2_addr.orR && (rs2_addr === mw_rd_addr)

  // forward logic
  val rs1 = MuxCase(regFile.io.rdata1, Seq(
    (de_rs1hazard) -> alu.io.out,
    (em_rs1hazard) -> em_regWrite,
    (mw_rs1hazard) -> regWrite
  ))
  val rs2 = MuxCase(regFile.io.rdata2, Seq(
    (de_rs2hazard) -> alu.io.out,
    (em_rs2hazard) -> em_regWrite,
    (mw_rs2hazard) -> regWrite
  ))

  // Branch condition calc
  brCond.io.rs1 := rs1
  brCond.io.rs2 := rs2
  brCond.io.br_type := io.ctrl.br_type

  // ALU Operands
  alu.io.A := Mux(io.ctrl.A_sel === A_RS1, rs1, fd_reg.pc)
  alu.io.B := Mux(io.ctrl.B_sel === B_RS2, rs2, immGen.io.out)

  // Pipelining De/Ex
  when(reset.asBool || !stall && csr.io.expt) {
    de_reg.st_type := 0.U
    de_reg.csr_cmd := 0.U
    de_reg.illegal := false.B
    de_reg.pc_check := false.B
    de_reg.ld_type := 0.U
    de_reg.wb_en := false.B
    de_reg.taken := false.B
    de_reg.pc_sel := 0.U
  }.elsewhen(!stall && !csr.io.expt) {
    de_reg.pc := fd_reg.pc
    de_reg.inst := fd_reg.inst
    de_reg.alu_a := Mux(io.ctrl.A_sel === A_RS1, rs1, fd_reg.pc)
    de_reg.alu_b := Mux(io.ctrl.B_sel === B_RS2, rs2, immGen.io.out)
    de_reg.csr_in := Mux(io.ctrl.imm_sel === IMM_Z, immGen.io.out, rs1)
    de_reg.alu_op := io.ctrl.alu_op
    de_reg.st_type := io.ctrl.st_type
    de_reg.csr_cmd := io.ctrl.csr_cmd
    de_reg.illegal := io.ctrl.illegal
    de_reg.pc_check := io.ctrl.pc_sel === PC_ALU
    de_reg.ld_type := io.ctrl.ld_type
    de_reg.wb_sel := io.ctrl.wb_sel
    de_reg.wb_en := io.ctrl.wb_en
    de_reg.taken := brCond.io.taken
    de_reg.pc_sel := io.ctrl.pc_sel
  }


  /** **** Execute ****
    */
  // ALU caculation
  alu.io.A := de_reg.alu_a
  alu.io.B := de_reg.alu_b
  alu.io.alu_op := de_reg.alu_op

  // Pipelining Ex/Me
  when(reset.asBool || !stall && csr.io.expt) {
    em_reg.st_type := 0.U
    em_reg.csr_cmd := 0.U
    em_reg.illegal := false.B
    em_reg.pc_check := false.B
    em_reg.ld_type := 0.U
    em_reg.wb_en := false.B
    em_reg.pc_sel := 0.U
  }.elsewhen(!stall && !csr.io.expt) {
    em_reg.pc := de_reg.pc
    em_reg.inst := de_reg.inst
    em_reg.alu := alu.io.out
    em_reg.csr_in := de_reg.csr_in
    em_reg.st_type := de_reg.st_type
    em_reg.csr_cmd := de_reg.csr_cmd
    em_reg.illegal := de_reg.illegal
    em_reg.pc_check := de_reg.pc_check
    em_reg.ld_type := de_reg.ld_type
    em_reg.wb_sel := de_reg.wb_sel
    em_reg.wb_en := de_reg.wb_en
    em_reg.pc_sel := de_reg.pc_sel
  }
  

  /** **** Memory ****
    */
  // ALU caculation
  // D$ access
  val abort = RegInit(false.B)
  val offset = (em_reg.alu(1) << 4.U).asUInt | (em_reg.alu(0) << 3.U).asUInt

  val daddr = em_reg.alu >> 2.U << 2.U
  io.dcache.req.valid := !stall && (em_reg.st_type.orR || em_reg.ld_type.orR)
  io.dcache.req.bits.addr := daddr
  io.dcache.req.bits.data := rs2 << offset
  io.dcache.req.bits.mask := MuxLookup(em_reg.st_type, "b0000".U)(
    Seq(ST_SW -> "b1111".U, ST_SH -> ("b11".U << em_reg.alu(1, 0)), ST_SB -> ("b1".U << em_reg.alu(1, 0)))
  )

  // Load
  val lshift = io.dcache.resp.bits.data >> offset
  load := MuxLookup(em_reg.ld_type, io.dcache.resp.bits.data.zext)(
    Seq(
      LD_LH -> lshift(15, 0).asSInt,
      LD_LB -> lshift(7, 0).asSInt,
      LD_LHU -> lshift(15, 0).zext,
      LD_LBU -> lshift(7, 0).zext
    )
  )
  
  // CSR access
  csr.io.stall := stall
  csr.io.in := em_reg.csr_in
  csr.io.cmd := em_reg.csr_cmd
  csr.io.inst := em_reg.inst
  csr.io.pc := em_reg.pc
  csr.io.addr := em_reg.alu
  csr.io.illegal := em_reg.illegal
  csr.io.pc_check := em_reg.pc_check
  csr.io.ld_type := em_reg.ld_type
  csr.io.st_type := em_reg.st_type
  io.host <> csr.io.host

  // Abort store when there's an excpetion
  abort := csr.io.expt
  io.dcache.abort := abort


  /** **** Writeback ****
    */
  // Pipelining
  when(reset.asBool || !stall && csr.io.expt) {
    mw_reg.ld_type := 0.U
    mw_reg.wb_en := false.B
  }.elsewhen(!stall && !csr.io.expt) {
    mw_reg.pc := em_reg.pc
    mw_reg.inst := em_reg.inst
    mw_reg.alu := em_reg.alu
    mw_reg.load := load
    mw_reg.csr_out := csr.io.out
    mw_reg.ld_type := em_reg.ld_type
    mw_reg.wb_sel := em_reg.wb_sel
    mw_reg.wb_en := em_reg.wb_en
  }

  // Regfile Write
  regWrite :=
    MuxLookup(mw_reg.wb_sel, mw_reg.alu.zext)(
      Seq(WB_MEM -> mw_reg.load, WB_PC4 -> (mw_reg.pc + 4.U).zext, WB_CSR -> mw_reg.csr_out.zext)
    ).asUInt

  regFile.io.wen := mw_reg.wb_en && !stall 
  regFile.io.waddr := mw_rd_addr
  regFile.io.wdata := regWrite

  // TODO: re-enable through AOP
//  if (p(Trace)) {
//    printf(
//      "PC: %x, INST: %x, REG[%d] <- %x\n",
//      ew_reg.pc,
//      ew_reg.inst,
//      Mux(regFile.io.wen, wb_rd_addr, 0.U),
//      Mux(regFile.io.wen, regFile.io.wdata, 0.U)
//    )
//  }
}
