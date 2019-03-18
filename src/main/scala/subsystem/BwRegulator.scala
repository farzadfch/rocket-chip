package freechips.rocketchip.subsystem

import Chisel._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._

class BwRegulator(implicit p: Parameters) extends LazyModule {

  val node = TLAdapterNode(
    clientFn  = { case c => c },
    managerFn = { case m => m })
    
  lazy val module = new LazyModuleImp(this) {
    val (out, _) = node.out(0)
    val (in, _) = node.in(0)
    
    out := in
    
    val window_cntr = RegInit(UInt(10.W), 0.U)
    val xaction_cntr = RegInit(UInt(10.W), 0.U)
    
    window_cntr := window_cntr + 1.U
    when (xaction_cntr >= 2.U) {
      out.a.valid := Bool(false)
      in.a.ready := Bool(false)
    }
    when (window_cntr === 0.U) {
      xaction_cntr := 0.U
    }
    when (out.a.fire()) {
      xaction_cntr := xaction_cntr + 1.U
    }
  }
}