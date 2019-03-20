package freechips.rocketchip.subsystem

import Chisel._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.regmapper._

class BwRegulator(
    address: BigInt,
    beatBytes: Int = 8)
    (implicit p: Parameters) extends LazyModule {

  val device = new SimpleDevice("bw-reg",Seq("ku-csl,bw-reg"))
  val regnode = new TLRegisterNode(
    address = Seq(AddressSet(address, 0x3f)),
    device = device,
    beatBytes = beatBytes)
  val node = TLAdapterNode(
    clientFn  = { case c => c },
    managerFn = { case m => m })

  lazy val module = new LazyModuleImp(this) {
    val w = 10
    var maxXactionRegs = Seq.empty[UInt]

    (node.in zip node.out) foreach { case ((in, _),(out, _)) =>
      out <> in

      val windowCntr = RegInit(UInt(w.W), 0.U)
      val xactionCntr = RegInit(UInt(w.W), 0.U)
      val maxXaction = RegInit(UInt(w.W), 1023.U)
      maxXactionRegs = maxXactionRegs :+ maxXaction

      windowCntr := windowCntr + 1.U
      when (xactionCntr >= maxXaction) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when (windowCntr === 0.U) {
        xactionCntr := 0.U
      }
      when (out.a.fire()) {
        xactionCntr := xactionCntr + 1.U
      }
    }

    val maxRegFields = maxXactionRegs.zipWithIndex.map { case (reg, i) =>
      8*i -> Seq(RegField(w, reg)) }
    regnode.regmap(maxRegFields: _*)
  }
}

