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
  val device = new SimpleDevice("tsu",Seq("ku-csl,tsu"))

  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(address, 0x7f)),
    device = device,
    beatBytes = 8)

  val node = TLAdapterNode()

  lazy val module = new LazyModuleImp(this)
  {
    // A TLAdapterNode has equal number of input and output edges
    val n = node.in.length
    require(n <= 32)

    val io = IO(new Bundle {
      val nThrottleWb = Output(Vec(n, Bool()))
    })

    val nDomains = n
    val wPeriod = 12 // for max 1us period, F = 2.13GHz
    val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when period length is set to max
    val wS2Period = 6
    val memBase = p(ExtMem).get.base.U
    var clientNames = new Array[String](n)

    val enBRUGlobal = RegInit(false.B)
    val countInstFetch = RegInit(true.B)
    val enWbThrottle = RegInit(false.B)
    val enDramThrottle = RegInit(false.B)
    val enS2 = RegInit(false.B)
    val periodCntr = Reg(UInt(wPeriod.W))
    val periodLen = Reg(UInt(wPeriod.W))
    val s2PeriodCntr = Reg(UInt(wS2Period.W))
    val s2PeriodLen = Reg(UInt(wS2Period.W))
    val accCntrs = Reg(Vec(nDomains, UInt(w.W)))
    val maxAccs = Reg(Vec(nDomains, UInt(w.W)))
    val dramAccCntrs = Reg(Vec(nDomains, UInt(w.W + 1)))
    val maxDramAccs = Reg(Vec(nDomains, UInt(w.W)))
    val wbCntrs = Reg(Vec(nDomains, UInt(w.W)))
    val maxWbs = Reg(Vec(nDomains, UInt(w.W)))
    val bwREnables = Reg(Vec(n, Bool()))
    val domainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
    val coreAcc = Wire(Vec(n, Bool()))
    val coreAccActive = Wire(Vec(n, Bool()))
    val coreDramAccActive = Wire(Vec(n, Bool()))
    val coreWbActive = Wire(Vec(n, Bool()))
    val throttleDomain = Wire(Vec(nDomains, Bool()))
    val throttleDomainDram = Wire(Vec(nDomains, Bool()))
    val throttleDomainWb = Wire(Vec(nDomains, Bool()))

    val cycleW = 40 // about 8 minutes in target machine time
    val cycle = RegInit(0.U(cycleW.W))

    val genPrintLat = true
    val accLatCntrW = 10
    val printLatEnable = RegInit(false.B)
    val endSourceId = node.in(0)._2.client.endSourceId
    val accLatCntrs = Seq.fill(n)(Reg(Vec(endSourceId, UInt(accLatCntrW.W))))
    val latThresh = Reg(UInt(accLatCntrW.W))

    val genPerf = true
    val perfPeriodW = 18 // max 100us
    val perfCntrW = perfPeriodW - 3
    val perfEnable = RegInit(false.B)
    val perfPeriod = Reg(UInt(perfPeriodW.W))
    val perfPeriodCntr = Reg(UInt(perfPeriodW.W))
    // It is not required to reset these counters but we keep it for now as it helps to close timing more easily in PnR
    val aCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
    val cCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))

    val periodCntrReset = periodCntr === periodLen
    periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

    val s2PeriodCntrReset = s2PeriodCntr === s2PeriodLen
    val s2GateOpen = s2PeriodCntr === 0.U
    val s2Fire = coreAcc.reduce(_||_)
    s2PeriodCntr := Mux(s2PeriodCntrReset || !enBRUGlobal || !enS2 || (s2GateOpen && !s2Fire), 0.U, s2PeriodCntr + 1.U)

    cycle := cycle + 1.U
    val perfPeriodCntrReset = perfPeriodCntr === perfPeriod
    perfPeriodCntr := Mux(perfPeriodCntrReset || !perfEnable, 0.U, perfPeriodCntr + 1.U)

    // generator loop for domains
    for (i <- 0 until nDomains) {
      // bit vector for cores that are enabled & access mem in the current cycle & are assigned to domain i
      val coreAccActMasked = (domainIds zip coreAccActive).map { case (d, act) => d === i.U && act }
      // sbus accepts transaction from only one core in a cycle, so it's ok to reduce-or the active cores bit vector
      accCntrs(i) := Mux(enBRUGlobal, coreAccActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, accCntrs(i)), 0.U)
      throttleDomain(i) := accCntrs(i) === maxAccs(i)

      val coreDramAccActMasked = (domainIds zip coreDramAccActive).map { case (d, act) => d === i.U && act }
      dramAccCntrs(i) := Mux(enBRUGlobal, coreDramAccActMasked.reduce(_||_) +
        Mux(periodCntrReset, Mux(throttleDomainDram(i), dramAccCntrs(i) - maxDramAccs(i), 0.U), dramAccCntrs(i)), 0.U)
      throttleDomainDram(i) := dramAccCntrs(i) >= maxDramAccs(i)

      val coreWbActMasked = (domainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
      wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
      throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)
    }

    // generator loop for cores
    for (i <- 0 until n) {
      val (out, edge_out) = node.out(i)
      val (in, edge_in) = node.in(i)

      val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
      val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
      // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
      val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

      coreAcc(i) := out.a.fire() && (aIsAcquire || aIsInstFetch && countInstFetch)
      coreAccActive(i) := bwREnables(i) && coreAcc(i)
      coreWbActive(i) := bwREnables(i) && edge_out.done(out.c) && cIsWb

      out <> in
      io.nThrottleWb(i) := true.B

      when (enBRUGlobal) {
        when ((bwREnables(i) && (throttleDomain(domainIds(i)) || throttleDomainDram(domainIds(i)) && enDramThrottle)) ||
          !s2GateOpen) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when (bwREnables(i) && throttleDomainWb(domainIds(i)) && enWbThrottle) {
          io.nThrottleWb(i) := false.B
        }
      }

      // DCache and ICache
      clientNames(i) = edge_in.client.clients(0).name + ", " + edge_in.client.clients(2).name

      // Measure access latency
      accLatCntrs(i).foreach { c =>
        c := c + 1.U
      }
      when(in.a.fire()) {
        accLatCntrs(i)(in.a.bits.source) := 0.U
      }
      when(in.c.fire() && in.c.bits.opcode === TLMessages.ReleaseData) {
        accLatCntrs(i)(in.c.bits.source) := 0.U
      }

      val accLat = MuxLookup(in.d.bits.source, 0.U,
        Array(0.U -> accLatCntrs(i)(0),
          1.U -> accLatCntrs(i)(1),
          2.U -> accLatCntrs(i)(2),
          3.U -> accLatCntrs(i)(3),
          8.U -> accLatCntrs(i)(8)))

      coreDramAccActive(i) := bwREnables(i) && in.d.fire() && edge_in.first(in.d) &&
        in.d.bits.opcode =/= TLMessages.ReleaseAck && (accLat > latThresh)

      if (genPrintLat) {
        when(printLatEnable && in.d.fire() && edge_in.first(in.d)) {
          printf(SynthesizePrintf("%d %d %d %d %d\n", cycle, i.U, in.d.bits.opcode, in.d.bits.source, accLat))
        }
      }

      // Performance counters
      if (genPerf) {
        when(perfPeriodCntrReset && perfEnable) {
          printf(SynthesizePrintf("%d %d %d %d\n", cycle, i.U, aCounters(i), cCounters(i)))
        }
      }
      aCounters(i) := Mux(perfEnable,
        (out.a.fire() && (aIsAcquire || aIsInstFetch)) + Mux(perfPeriodCntrReset, 0.U, aCounters(i)), 0.U)
      cCounters(i) := Mux(perfEnable,
        (edge_out.done(out.c) && cIsWb) + Mux(perfPeriodCntrReset, 0.U, cCounters(i)), 0.U)
    }

    val enBRUGlobalRegField = Seq(0 -> Seq(
      RegField(enBRUGlobal.getWidth, enBRUGlobal,
        RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

    val settingsRegField = Seq(4*1 -> Seq(
      RegField(countInstFetch.getWidth, countInstFetch,
        RegFieldDesc("countInstFetch", "Count instruction fetch")),
      RegField(enWbThrottle.getWidth, enWbThrottle,
        RegFieldDesc("enWbThrottle", "Enable writeback throttling")),
      RegField(enDramThrottle.getWidth, enDramThrottle,
        RegFieldDesc("enDramThrottle", "Enable DRAM throttling")),
      RegField(enS2.getWidth, enS2,
        RegFieldDesc("enS2", "Enable seconed stage"))))

    val periodLenRegField = Seq(4*2 -> Seq(
      RegField(periodLen.getWidth, periodLen,
        RegFieldDesc("periodLen", "Period length"))))

    val maxAccRegFields = maxAccs.zipWithIndex.map { case (reg, i) =>
      4*(3 + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

    val maxWbRegFields = maxWbs.zipWithIndex.map { case (reg, i) =>
      4*(3+nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxWb$i", s"Maximum writeback for domain $i"))) }

    val bwREnablesField = Seq(4*(3 + 2*nDomains) -> bwREnables.zipWithIndex.map { case (bit, i) =>
      RegField(bit.getWidth, bit, RegFieldDesc("bwREnables", s"Enable bandwidth regulation for ${clientNames(i)}")) })

    val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) =>
      4*(4 + 2*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

    val perfEnField = Seq(4*(4 + 2*nDomains + n) -> Seq(
      RegField(perfEnable.getWidth, perfEnable,
        RegFieldDesc("perfEnable", "perfEnable"))))

    val perfPeriodField = Seq(4*(5 + 2*nDomains + n) -> Seq(
      RegField(perfPeriod.getWidth, perfPeriod,
        RegFieldDesc("perfPeriod", "perfPeriod"))))

    val printLatEnField = Seq(4*(6 + 2*nDomains + n) -> Seq(
      RegField(printLatEnable.getWidth, printLatEnable,
        RegFieldDesc("printLatEnable", "Enable print latency"))))

    val latThreshField = Seq(4*(7 + 2*nDomains + n) -> Seq(
      RegField(latThresh.getWidth, latThresh,
        RegFieldDesc("latThresh", "Latency threshold"))))

    val maxDramAccRegFields = maxDramAccs.zipWithIndex.map { case (reg, i) =>
      4*(8 + 2*nDomains + n + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxDramAcc$i", s"Maximum DRAM access for domain $i"))) }

    val s2PeriodLenField = Seq(4*(8 + 3*nDomains + n) -> Seq(
      RegField(s2PeriodLen.getWidth, s2PeriodLen,
        RegFieldDesc("s2PeriodLen", "S2 period length"))))

    regnode.regmap(enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxAccRegFields ++ maxWbRegFields ++
      bwREnablesField ++ domainIdFields ++ perfEnField ++ perfPeriodField ++ printLatEnField ++ latThreshField ++
      maxDramAccRegFields ++ s2PeriodLenField: _*)

    println("Traffic Shapper (TSU):")
    for (i <- clientNames.indices)
      println(s"  $i => ${clientNames(i)}")
  }
/*
  lazy val module = new LazyModuleImp(this)
  {
    // A TLAdapterNode has equal number of input and output edges
    val n = node.in.length
    require(n <= 32)

    val io = IO(new Bundle {
      val nThrottleWb = Output(Vec(n, Bool()))
    })

    val memBase = p(ExtMem).get.base.U
    val wPeriod = 25 // for max 10ms period, F = 2.13GHz
    val w = wPeriod - 3 // it can count up to a transaction per 8 cycles when window size is set to max
    val nDomains = n
    var clientNames = new Array[String](n)

    val enBRUGlobal = RegInit(false.B)
    val countInstFetch = RegInit(true.B)
    val enWbThrottle = RegInit(true.B)
    val periodCntr = Reg(UInt(wPeriod.W))
    val periodLen = Reg(UInt(wPeriod.W))
    val accCntrs = Reg(Vec(nDomains, UInt(w.W)))
    val maxAccs = Reg(Vec(nDomains, UInt(w.W)))
    val wbCntrs = Reg(Vec(nDomains, UInt(w.W)))
    val maxWbs = Reg(Vec(nDomains, UInt(w.W)))
    val bwREnables = Reg(Vec(n, Bool()))
    val domainIds = Reg(Vec(n, UInt(log2Ceil(nDomains).W)))
    val coreAccActive = Wire(Vec(n, Bool()))
    val coreWbActive = Wire(Vec(n, Bool()))
    val throttleDomain = Wire(Vec(nDomains, Bool()))
    val throttleDomainWb = Wire(Vec(nDomains, Bool()))

    val cycleW = 40 // about 8 minutes in target machine time
    val cycle = RegInit(0.U(cycleW.W))

    val enablePrintDelay = true
    val transDelayCntrW = 10
    val printDelayEnable = RegInit(false.B)
    val endSourceId = node.in(0)._2.client.endSourceId
    val transDelayCntrs = Reg(Vec(endSourceId, UInt(transDelayCntrW.W)))

    val enablePerf = true
    val perfPeriodW = 18 // max 100us
    val perfCntrW = perfPeriodW - 3

    val perfEnable = RegInit(false.B)
    val perfPeriod = Reg(UInt(perfPeriodW.W))
    val perfPeriodCntr = Reg(UInt(perfPeriodW.W))

    cycle := cycle + 1.U

    // It is not required to reset these counters but we keep it for now as it helps to close timing
    //  more easily in PnR
    val aCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))
    val cCounters = RegInit(VecInit(Seq.fill(n)(0.U(perfCntrW.W))))

    val perfPeriodCntrReset = perfPeriodCntr >= perfPeriod
    perfPeriodCntr := Mux(perfPeriodCntrReset || !perfEnable, 0.U, perfPeriodCntr + 1.U)

    val periodCntrReset = periodCntr >= periodLen
    periodCntr := Mux(periodCntrReset || !enBRUGlobal, 0.U, periodCntr + 1.U)

    // generator loop for domains
    for (i <- 0 until nDomains) {
      // bit vector for cores that are enabled & access mem in the current cycle & are assigned to domain i
      val coreAccActMasked = (domainIds zip coreAccActive).map { case (d, act) => d === i.U && act }
      // sbus accepts transaction from only one core in a cycle, so it's ok to reduce-or the active cores bit vector
      accCntrs(i) := Mux(enBRUGlobal, coreAccActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, accCntrs(i)), 0.U)
      throttleDomain(i) := accCntrs(i) >= maxAccs(i)

      val coreWbActMasked = (domainIds zip coreWbActive).map { case (d, act) => d === i.U && act }
      wbCntrs(i) := Mux(enBRUGlobal, coreWbActMasked.reduce(_||_) + Mux(periodCntrReset, 0.U, wbCntrs(i)), 0.U)
      throttleDomainWb(i) := wbCntrs(i) >= maxWbs(i)
    }

    // generator loop for cores
    for (i <- 0 until n) {
      val (out, edge_out) = node.out(i)
      val (in, edge_in) = node.in(i)

      val aIsAcquire = in.a.bits.opcode === TLMessages.AcquireBlock
      val aIsInstFetch = in.a.bits.opcode === TLMessages.Get && in.a.bits.address >= memBase
      // ReleaseData or ProbeAckData cause a PutFull in Broadcast Hub
      val cIsWb = in.c.bits.opcode === TLMessages.ReleaseData || in.c.bits.opcode === TLMessages.ProbeAckData

      coreAccActive(i) := bwREnables(i) && out.a.fire() && (aIsAcquire || aIsInstFetch && countInstFetch)
      coreWbActive(i) := bwREnables(i) && edge_out.done(out.c) && cIsWb

      out <> in
      io.nThrottleWb(i) := true.B

      when (enBRUGlobal && bwREnables(i)) {
        when (throttleDomain(domainIds(i))) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when (throttleDomainWb(domainIds(i)) && enWbThrottle) {
          io.nThrottleWb(i) := false.B
        }
      }

      // DCache and ICache
      clientNames(i) = edge_in.client.clients(0).name + ", " + edge_in.client.clients(2).name

      // Measure access latency
      if (enablePrintDelay && i == 3) {
        transDelayCntrs.foreach { c =>
          c := c + 1.U
        }
        when(in.a.fire()) {
          transDelayCntrs(in.a.bits.source) := 0.U
        }
        when(in.c.fire() && in.c.bits.opcode === TLMessages.ReleaseData) {
          transDelayCntrs(in.c.bits.source) := 0.U
        }

        val printDelay = MuxLookup(in.d.bits.source, 0.U,
          Array(0.U -> transDelayCntrs(0),
            1.U -> transDelayCntrs(1),
            2.U -> transDelayCntrs(2),
            3.U -> transDelayCntrs(3),
            8.U -> transDelayCntrs(8)))

        when(printDelayEnable && in.d.fire() && edge_in.first(in.d)) {
          printf(SynthesizePrintf("%d %d %d %d %d\n", cycle, i.U, in.d.bits.opcode, in.d.bits.source, printDelay))
        }
      }

      // Performance counters
      if (enablePerf) {
        when(perfPeriodCntrReset && perfEnable) {
          printf(SynthesizePrintf("%d %d %d %d\n", cycle, i.U, aCounters(i), cCounters(i)))
        }
      }
      aCounters(i) := Mux(perfEnable,
        (out.a.fire() && (aIsAcquire || aIsInstFetch)) + Mux(perfPeriodCntrReset, 0.U, aCounters(i)), 0.U)
      cCounters(i) := Mux(perfEnable,
        (edge_out.done(out.c) && cIsWb) + Mux(perfPeriodCntrReset, 0.U, cCounters(i)), 0.U)
    }

    val enBRUGlobalRegField = Seq(0 -> Seq(
      RegField(enBRUGlobal.getWidth, enBRUGlobal,
        RegFieldDesc("enBRUGlobal", "Enable BRU Global"))))

    val settingsRegField = Seq(4*1 -> Seq(
      RegField(countInstFetch.getWidth, countInstFetch,
        RegFieldDesc("countInstFetch", "Count instruction fetch")),
      RegField(enWbThrottle.getWidth, enWbThrottle,
        RegFieldDesc("enWbThrottle", "Enable writeback throttling"))))

    val periodLenRegField = Seq(4*2 -> Seq(
      RegField(periodLen.getWidth, periodLen,
        RegFieldDesc("periodLen", "Period length"))))

    val maxAccRegFields = maxAccs.zipWithIndex.map { case (reg, i) =>
      4*(3 + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxAcc$i", s"Maximum access for domain $i"))) }

    val maxWbRegFields = maxWbs.zipWithIndex.map { case (reg, i) =>
      4*(3+nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"maxWb$i", s"Maximum writeback for domain $i"))) }

    val bwREnablesField = Seq(4*(3 + 2*nDomains) -> bwREnables.zipWithIndex.map { case (bit, i) =>
      RegField(bit.getWidth, bit, RegFieldDesc("bwREnables", s"Enable bandwidth regulation for ${clientNames(i)}")) })

    val domainIdFields = domainIds.zipWithIndex.map { case (reg, i) =>
      4*(4 + 2*nDomains + i) -> Seq(RegField(reg.getWidth, reg,
        RegFieldDesc(s"domainId$i", s"Domain ID for ${clientNames(i)}"))) }

    val perfEnField = Seq(4*(4 + 2*nDomains + n) -> Seq(
      RegField(perfEnable.getWidth, perfEnable,
        RegFieldDesc("perfEnable", "perfEnable"))))

    val perfPeriodField = Seq(4*(5 + 2*nDomains + n) -> Seq(
      RegField(perfPeriod.getWidth, perfPeriod,
        RegFieldDesc("perfPeriod", "perfPeriod"))))

    val printDelayEnField = Seq(4*(6 + 2*nDomains + n) -> Seq(
      RegField(printDelayEnable.getWidth, printDelayEnable,
        RegFieldDesc("printDelayEnable", "Enable print delay"))))

    regnode.regmap(enBRUGlobalRegField ++ settingsRegField ++ periodLenRegField ++ maxAccRegFields ++ maxWbRegFields ++
      bwREnablesField ++ domainIdFields ++ perfEnField ++ perfPeriodField ++ printDelayEnField: _*)

    println("Bandwidth regulation (BRU):")
    for (i <- clientNames.indices)
      println(s"  $i => ${clientNames(i)}")
  }
*/
}
