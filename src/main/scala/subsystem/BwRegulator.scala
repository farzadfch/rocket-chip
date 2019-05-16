package freechips.rocketchip.subsystem

import chisel3._
import chisel3.util._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._

class BwRegulator(
    address: BigInt,
    beatBytes: Int = 8)
    (implicit p: Parameters) extends LazyModule
{
  val device = new SimpleDevice("bw-reg",Seq("ku-csl,bw-reg"))
  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(address, 0x3f)),
    device = device,
    beatBytes = beatBytes)
  val node = TLAdapterNode(
    clientFn  = { case c => c },
    managerFn = { case m => m })

  lazy val module = new LazyModuleImp(this)
  {
    val n = node.in.length
    require(n == node.out.length)
    require(n <= 32)

    val w = 32
    var masterNames = new Array[String](n)
    val enableBW = RegInit(false.B)
    val windowCntr = Reg(UInt(w.W))
    val windowSize = Reg(UInt(w.W))
    val xactionCntrs = Reg(Vec(n, UInt(w.W)))
    val maxXactionRegs = Reg(Vec(n, UInt(w.W)))
    val enableMasters = RegInit(VecInit(Seq.fill(n)(false.B)))
    val domainId = Reg(Vec(n, UInt(log2Ceil(n).W)))

    when (windowCntr >= windowSize || !enableBW) {
      windowCntr := 0.U
      xactionCntrs.foreach(_ := 0.U)
    } .otherwise {
      windowCntr := windowCntr + 1.U
    }

    for (i <- 0 until n) {
      val (out, _) = node.out(i)
      val (in, edge_in) = node.in(i)

      out <> in
      when (enableBW && enableMasters(i)) {
        when(xactionCntrs(domainId(i)) >= maxXactionRegs(domainId(i))) {
          out.a.valid := false.B
          in.a.ready := false.B
        }
        when(out.a.fire()) {
          xactionCntrs(domainId(i)) := xactionCntrs(domainId(i)) + 1.U
        }
      }

      masterNames(i) = edge_in.client.clients(0).name
    }

    val enableBWField = Seq(0 -> Seq(RegField(enableBW.getWidth, enableBW,
      RegFieldDesc("enableBW", "Enable BW-regulator"))))

    val windowRegField = Seq(4*1 -> Seq(RegField(windowSize.getWidth, windowSize,
      RegFieldDesc("windowsize", "Size of the window"))))

    val maxRegFields = maxXactionRegs.zipWithIndex.map { case (reg, i) =>
      4*(2 + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"maxxaction$i", s"Maximum number of transactions for domain $i"))) }

    val enableMastersField = Seq(4*(2 + n) -> enableMasters.zipWithIndex.map { case (bit, i) =>
      RegField(bit.getWidth, bit, RegFieldDesc("enableMasters", s"Enable BW-regulator for ${masterNames(i)}")) })

    val domainIdFields = domainId.zipWithIndex.map { case (reg, i) =>
      4*(3+n + i) -> Seq(RegField(reg.getWidth, reg,
      RegFieldDesc(s"domainId$i", s"Domain ID for ${masterNames(i)}"))) }

    regnode.regmap(enableBWField ++ windowRegField ++ maxRegFields ++ enableMastersField ++ domainIdFields: _*)

    println("\nBW-regulated masters:")
    for (i <- masterNames.indices)
      println(s"$i: ${masterNames(i)}")
    println
  }
}

