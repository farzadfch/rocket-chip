package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._

class BwRegulator(address: BigInt) (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bw-reg",Seq("ku-csl,bw-reg"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(address, 0x7f)),
    device = device,
    beatBytes = 8)

  val node = TLAdapterNode(
    clientFn  = { case c => c },
    managerFn = { case m => m })

  lazy val module = new LazyModuleImp(this)
  {
    val n = node.in.length
    require(n == node.out.length)
    require(n <= 32)

    val io = IO(new Bundle {
      val nWbInhibit = Output(Vec(n, Bool()))
    })

    val memBase = p(ExtMem).get.base.U
    val wWndw = 32
    val w = 24
    val wWrCost = 3
    val nDomains = n
    var masterNames = new Array[String](n)
    val enableBW = RegInit(false.B)
    val countInstFetch = RegInit(true.B)
    val enIhibitWb = RegInit(true.B)
    val windowCntr = Reg(UInt(wWndw.W))
    val windowSize = Reg(UInt(wWndw.W))
    val transCntrs = Reg(Vec(nDomains, UInt(w.W)))
    val maxTransRegs = Reg(Vec(nDomains, UInt(w.W)))
    val transCntrsWr = Reg(Vec(nDomains, UInt(w.W)))
    val maxTransRegsWr = Reg(Vec(nDomains, UInt(w.W)))
    val enableMasters = Reg(Vec(n, Bool()))
    val domainId = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))

    when (windowCntr >= windowSize || !enableBW) {
      windowCntr := 0.U
      transCntrs.foreach(_ := 0.U)
      transCntrsWr.foreach(_ := 0.U)
    } .otherwise {
      windowCntr := windowCntr + 1.U
    }

    for (i <- 0 until n) {
      val (out, edge_out) = node.out(i)
      val (in, edge_in) = node.in(i)

      val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
      val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
      // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
      val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

      out <> in
      io.nWbInhibit(i) := true.B

      when (enableBW && enableMasters(i)) {
        when (transCntrs(domainId(i)) >= maxTransRegs(domainId(i))) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when (out.a.fire() && (aIsAcquire || aIsInstFetch && countInstFetch)) {
          transCntrs(domainId(i)) := transCntrs(domainId(i)) + 1.U
        }
        when (transCntrsWr(domainId(i)) >= maxTransRegsWr(domainId(i)) && enIhibitWb) {
          io.nWbInhibit(i) := false.B
        }
        when (edge_out.done(out.c) && cIsWb) {
          transCntrsWr(domainId(i)) := transCntrsWr(domainId(i)) + 1.U
        }
      }

      masterNames(i) = edge_in.client.clients(0).name
    }

    val enableBwRegField = Seq(0 -> Seq(RegField(enableBW.getWidth, enableBW,
      RegFieldDesc("enableBW", "Enable BW-regulator"))))

    val bwSettings = Seq(4*1 -> Seq(
      RegField(countInstFetch.getWidth, countInstFetch,
        RegFieldDesc("countInstFetch", "Count instruction fetch")),
      RegField(enIhibitWb.getWidth, enIhibitWb,
        RegFieldDesc("enIhibitWb", "Enable writeback inhibit"))))

    val windowRegField = Seq(4*2 -> Seq(RegField(windowSize.getWidth, windowSize,
      RegFieldDesc("windowsize", "Size of the window"))))

    val maxTransRegFields = maxTransRegs.zipWithIndex.map { case (reg, i) =>
      4*(3 + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"max$i", s"Maximum transactions for domain $i"))) }

    val maxTransRegWrFields = maxTransRegsWr.zipWithIndex.map { case (reg, i) =>
      4*(3+nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"max$i", s"Maximum transactions for domain $i"))) }

    val enableMastersField = Seq(4*(3+2*nDomains) -> enableMasters.zipWithIndex.map { case (bit, i) =>
      RegField(bit.getWidth, bit, RegFieldDesc("enableMasters", s"Enable BW-regulator for ${masterNames(i)}")) })

    val domainIdFields = domainId.zipWithIndex.map { case (reg, i) =>
      4*(4+2*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"domainId$i", s"Domain ID for ${masterNames(i)}"))) }

    regnode.regmap(enableBwRegField ++ bwSettings ++ windowRegField ++ maxTransRegFields ++ maxTransRegWrFields ++
      enableMastersField ++ domainIdFields: _*)

    println("BW-regulated masters:")
    for (i <- masterNames.indices)
      println(s"  $i: ${masterNames(i)}")
  }
}
