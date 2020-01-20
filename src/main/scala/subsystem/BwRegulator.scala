package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._
import midas.targetutils._

class BwRegulator(address: BigInt) (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bru",Seq("ku-csl,bru"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(address, 0x7f)),
    device = device,
    beatBytes = 8)

  val node = TLAdapterNode()

  lazy val module = new LazyModuleImp(this)
  {
    val n = node.in.length
    require(n == node.out.length)
    require(n <= 32)

    val io = IO(new Bundle {
      val nWbInhibit = Output(Vec(n, Bool()))
    })

    val memBase = p(ExtMem).get.base.U
    val wWndw = 25 // for max 10ms period, F=2.13GHz
    val w = wWndw - 3 // it can count up to a transaction per 8 cycles when window size is set to max
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
    val masterTransActive = Wire(Vec(n, Bool()))
    val masterWrTransActive = Wire(Vec(n, Bool()))
    val throttleDomain = Wire(Vec(nDomains, Bool()))
    val throttleDomainWr = Wire(Vec(nDomains, Bool()))

    val perfCycleW = 40 // about 8 minutes in target machine time
    val perfPeriodW = 18 // max 100us
    val perfCntrW = perfPeriodW - 3
    val perfEnable = RegInit(false.B)
    val perfPeriod = Reg(UInt(perfPeriodW.W))
    val perfPeriodCntr = Reg(UInt(perfPeriodW.W))
    // It is not required to reset these counters but we keep it for now as it helps to close timing
    //  more easily in PnR
    val aCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
    val cCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
    val cycle = RegInit(0.U(perfCycleW.W))

    cycle := cycle + 1.U
    val perfPeriodCntrReset = perfPeriodCntr >= perfPeriod
    perfPeriodCntr := Mux(perfPeriodCntrReset || !perfEnable, 0.U, perfPeriodCntr + 1.U)

    val windowCntrReset = windowCntr >= windowSize
    windowCntr := Mux(windowCntrReset || !enableBW, 0.U, windowCntr + 1.U)

    // generator loop for domains
    for (i <- 0 until nDomains) {
      // bit vector for masters that are enabled & access mem in the current cycle & are assigned to domain i
      val masterActiveMasked = (domainId zip masterTransActive).map { case (d, act) => d === i.U && act }
      // sbus accepts transaction from only one master in a cycle, so it's ok to reduce-or the active masters bit vector
      transCntrs(i) := Mux(enableBW, masterActiveMasked.reduce(_||_) + Mux(windowCntrReset, 0.U, transCntrs(i)), 0.U)
      throttleDomain(i) := transCntrs(i) >= maxTransRegs(i)

      val masterWrActiveMasked = (domainId zip masterWrTransActive).map { case (d, act) => d === i.U && act }
      transCntrsWr(i) := Mux(enableBW, masterWrActiveMasked.reduce(_||_) + Mux(windowCntrReset, 0.U, transCntrsWr(i)), 0.U)
      throttleDomainWr(i) := transCntrsWr(i) >= maxTransRegsWr(i)
    }

    // generator loop for masters
    for (i <- 0 until n) {
      val (out, edge_out) = node.out(i)
      val (in, edge_in) = node.in(i)

      val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
      val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
      // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
      val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

      masterTransActive(i) := enableMasters(i) && out.a.fire() && (aIsAcquire || aIsInstFetch && countInstFetch)
      masterWrTransActive(i) := enableMasters(i) && edge_out.done(out.c) && cIsWb

      out <> in
      io.nWbInhibit(i) := true.B

      when (enableBW && enableMasters(i)) {
        when (throttleDomain(domainId(i))) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when (throttleDomainWr(domainId(i)) && enIhibitWb) {
          io.nWbInhibit(i) := false.B
        }
      }

      // DCache and ICache
      masterNames(i) = edge_in.client.clients(0).name + ", " + edge_in.client.clients(2).name

      when (perfPeriodCntrReset && perfEnable) {
        printf(SynthesizePrintf("%d %d %d %d\n", cycle, i.U, aCounters(i), cCounters(i)))
      }
      aCounters(i) := Mux(perfEnable,
        (out.a.fire() && (aIsAcquire || aIsInstFetch)) + Mux(perfPeriodCntrReset, 0.U, aCounters(i)), 0.U)
      cCounters(i) := Mux(perfEnable,
        (edge_out.done(out.c) && cIsWb) + Mux(perfPeriodCntrReset, 0.U, cCounters(i)), 0.U)
    }

    val enableBwRegField = Seq(0 -> Seq(
      RegField(enableBW.getWidth, enableBW,
        RegFieldDesc("enableBW", "Enable BW-regulator"))))

    val bwSettings = Seq(4*1 -> Seq(
      RegField(countInstFetch.getWidth, countInstFetch,
        RegFieldDesc("countInstFetch", "Count instruction fetch")),
      RegField(enIhibitWb.getWidth, enIhibitWb,
        RegFieldDesc("enIhibitWb", "Enable writeback inhibit"))))

    val windowRegField = Seq(4*2 -> Seq(
      RegField(windowSize.getWidth, windowSize,
        RegFieldDesc("windowsize", "Size of the window"))))

    val maxTransRegFields = maxTransRegs.zipWithIndex.map { case (reg, i) =>
      4*(3 + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"max$i", s"Maximum transactions for domain $i"))) }

    val maxTransRegWrFields = maxTransRegsWr.zipWithIndex.map { case (reg, i) =>
      4*(3+nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"max$i", s"Maximum writeback transactions for domain $i"))) }

    val enableMastersField = Seq(4*(3 + 2*nDomains) -> enableMasters.zipWithIndex.map { case (bit, i) =>
      RegField(bit.getWidth, bit, RegFieldDesc("enableMasters", s"Enable BW-regulator for ${masterNames(i)}")) })

    val domainIdFields = domainId.zipWithIndex.map { case (reg, i) =>
      4*(4 + 2*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"domainId$i", s"Domain ID for ${masterNames(i)}"))) }

    val perfEnField = Seq(4*(4 + 2*nDomains + n) -> Seq(
      RegField(perfEnable.getWidth, perfEnable,
        RegFieldDesc("perfEnable", "perfEnable"))))

    val perfPeriodField = Seq(4*(5 + 2*nDomains + n) -> Seq(
      RegField(perfPeriod.getWidth, perfPeriod,
        RegFieldDesc("perfPeriod", "perfPeriod"))))

    regnode.regmap(enableBwRegField ++ bwSettings ++ windowRegField ++ maxTransRegFields ++ maxTransRegWrFields ++
      enableMastersField ++ domainIdFields ++ perfEnField ++ perfPeriodField: _*)

    println("BW-regulated masters:")
    for (i <- masterNames.indices)
      println(s"  $i: ${masterNames(i)}")
  }
}
