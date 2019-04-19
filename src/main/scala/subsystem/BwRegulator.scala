package freechips.rocketchip.subsystem

import chisel3._
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
    val w = 32
    var maxXactionRegs = Seq.empty[UInt]
    var masterNames = Seq.empty[String]
    val windowCntr = RegInit(UInt(w.W), 0.U)
    val windowSize = RegInit(UInt(w.W), 0xffffffffL.U)

    val resetWindow = windowCntr >= windowSize
    when (resetWindow) {
      windowCntr := 0.U
    } .otherwise {
      windowCntr := windowCntr + 1.U
    }

    (node.in zip node.out) foreach { case ((in, edge_in),(out, _)) =>
      out <> in

      masterNames = masterNames :+ edge_in.client.clients(0).name

      val xactionCntr = RegInit(UInt(w.W), 0.U)
      val maxXaction = RegInit(UInt(w.W), 0xffffffffL.U)
      maxXactionRegs = maxXactionRegs :+ maxXaction

      when (xactionCntr >= maxXaction) {
        out.a.valid := false.B
        in.a.ready := false.B
      }
      when (resetWindow) {
        xactionCntr := 0.U
      }
      when (out.a.fire()) {
        xactionCntr := xactionCntr + 1.U
      }
    }

    val windowRegField = Seq(0 -> Seq(RegField(w, windowSize,
      RegFieldDesc("windowsize", "Size of the window"))))

    val maxRegFields = maxXactionRegs.zipWithIndex.map { case (reg, i) =>
      4*(i+1) -> Seq(RegField(w, reg,
      RegFieldDesc(s"maxxaction$i", s"Maximum number of transactions for ${masterNames(i)}"))) }

    regnode.regmap(windowRegField ++ maxRegFields: _*)

    println("\nBW regulated masters in order they appear in register map:")
    masterNames.foreach(println)
    println
  }
}

