/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 * +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Lighhouse deck FPGA
 *
 * Copyright (C) 2020 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

package lighthouse

import com.github.tototoshi.csv._

import spinal.core._
import spinal.lib._
import spinal.lib.fsm._
import spinal.lib.io.TriState
import spinal.lib.com.uart._

import spinal.lib.blackbox.lattice.ice40._
import _root_.lighthouse.lighthouse.Ddr
import _root_.lighthouse.lighthouse.ShiftBuffer
import java.io.File
import scala.collection.mutable
import _root_.lighthouse.lighthouse.SB_WARMBOOT

class LighthouseTopLevel(nSensors: Int = 4,
                         frequency: HertzNumber = 48 MHz,
                         useDdrDecoder: Boolean = true,
                         uartBaudrate: HertzNumber = 230400 Hz,
                         decodershortDelay: TimeNumber = 124 ns,
                         decoderUnsyncDelay: TimeNumber = 235 ns
                        ) extends Component {
  val io = new Bundle {
    val clk12MHz = in Bool

    val e = Vec(inout(Analog(Bool)), nSensors)
    val d = Vec(inout(Analog(Bool)), nSensors)

    val uart = master(Uart())

    val led0 = out Bool
    val led1 = out Bool
    val led2 = out Bool
  }

  // PLL, fast clock generation
  val clkCtrl = new Area {
    val pll = new pll

    pll.io.clock_in := io.clk12MHz

    val coreClockDomain = ClockDomain.internal(
      name = "Core",
      frequency = FixedFrequency(frequency),
      config = ClockDomainConfig(
        resetKind = BOOT
      )
    )

    coreClockDomain.clock := pll.io.clock_out
  }

  // Slow clock generation and definition
  val slowClkCtrl = new ClockingArea(clkCtrl.coreClockDomain) {
    // Generate slow clock
    val slowClockDomain = ClockDomain.internal(
      name = "Slow",
      frequency = FixedFrequency(ClockDomain.current.frequency.getValue / 2),
      config = ClockDomainConfig(
        resetKind = BOOT
      )
    )

    val slowClk = RegInit(False)
    slowClk := !slowClk

    slowClockDomain.clock := slowClk
  }

  // Area clocked at 48MHzとArea clocked at 24MHzの間で、以下のようなデータのやりとり（共有）が行われています：
  // 入力/出力（E / D）：
  // 各エリアでは、EおよびDという名前のベクトルが、
  // センサから読み込むための入力として、およびセンサに書き込むための出力として使用されます。 
  // これらのベクトルは、48MHzエリアで読み込まれ、24MHzエリアで書き込まれます。
  // 
  // ビームワード：
  // 48MHzエリアでセンサから読み取られたデータは、ビームワードと呼ばれる構造体にパッケージ化されます。
  // このビームワードは、24MHzのエリアでさらに処理され、パルスとともにシングルストリームにパッケージ化されます。

  // Sensors IO wires.
  // The IO cells is clocked in the fast clock domain
  // the output wires, *_out and *_oe, are driven by the slow clock domain
  val e = Vec(Bool, nSensors)
  val e_out = Vec(Bool, nSensors)
  val e_oe = Vec(Bool, nSensors)
  val d = Vec(Bool, nSensors)
  val d_out = Vec(Bool, nSensors)
  val d_oe = Vec(Bool, nSensors)

  // Area clocked at 48MHz
  //  この領域では、beamWordsスタートリームを作成し、各センサに対して特定の動作を行います：
  //  入力/出力（IO）としてSB_IOデバイス（おそらく独自の入出力デバイス）を使用してデータを読み書きします。
  //  
  //  Ddrオブジェクト内にD_IN_0フィールドから値を取得します。
  //  （データはDDR（Double Data Rate）モードで転送される（一つのクロックサイクルに2ビットのデータを転送する方式）と思われます。）
  //  
  //  データをBMCデコード器に送り、脈動と位相を分析します
  //  （これは光パルスをデータに変換するプロセスと思われます）。
  //  
  //  具体的な処理方法はuseDdrDecoderオプションによります。
  //  このオプションがtrueの場合はDdrBmcDecoderを使用し、falseの場合はBmcDecoderを使用します。
  //  デコードされたデータはShiftBufferへ送られ、データビームの一部としてストリーム化されます。
  val core = new ClockingArea(clkCtrl.coreClockDomain) {

    val beamWords = Vec(Stream(Bits(17 bits)), nSensors)

    for (sensor <- 0 until nSensors) {
      // DDR IOs ...
      val ioE = SB_IO.ddrRegistredInout
      ioE.INPUT_CLK := clockDomain.clock
      ioE.OUTPUT_CLK := clockDomain.clock
      ioE.OUTPUT_ENABLE := BufferCC(e_oe(sensor))
      ioE.D_OUT_0 := BufferCC(e_out(sensor))
      ioE.D_OUT_1 := BufferCC(e_out(sensor))
      ioE.CLOCK_ENABLE := True
      ioE.PACKAGE_PIN := io.e(sensor)
      e(sensor) := ioE.D_IN_0

      val ioD = SB_IO.ddrRegistredInout
      ioD.INPUT_CLK := clockDomain.clock
      ioD.OUTPUT_CLK := clockDomain.clock
      ioD.CLOCK_ENABLE := True
      ioD.OUTPUT_ENABLE := BufferCC(d_oe(sensor))
      ioD.D_OUT_0 := BufferCC(d_out(sensor))
      ioD.D_OUT_1 := BufferCC(d_out(sensor))
      ioD.PACKAGE_PIN := io.d(sensor)
      d(sensor) := ioD.D_IN_0

      // Put DDR data input in a ddr object
      val signalD = new Ddr
      signalD.v(0) := ioD.D_IN_0
      signalD.v(1) := ioD.D_IN_1

      // BMC decoder
      val decoderOutput = if (useDdrDecoder) {
        val decoder = new DdrBmcDecoder(decodershortDelay, decoderUnsyncDelay)
        decoder.io.signal := RegNext(RegNext(signalD))
        decoder.io.enable := True
        decoder.io.output
      } else {
        val decoder = new BmcDecoder(decodershortDelay, decoderUnsyncDelay)
        decoder.io.signal := RegNext(RegNext(ioD.D_IN_0))
        decoder.io.output
      }

      // Beam buffer, reset it at the end of the envelope
      val dataBuffer = new ShiftBuffer(17)
      dataBuffer.io.resetBuffer := RegNext(ioE.D_IN_0).fall()
      dataBuffer.io.dataIn << decoderOutput.toStream.stage().throwWhen(RegNext(RegNext(ioE.D_IN_0)))
      beamWords(sensor) << dataBuffer.io.dataOut.stage()
    }
  }

  // Area clocked at 24MHz
  // この領域の主な目的は、センサデータの取得と処理、そしてそのデータを専用のストリームにパッケージ化することです。以下は、この領域で行われる主なステップです：
  // - IDマップされたビームワードの生成：各センサに対して、「Sensor Configurator」と「Pulse Timer and Data Acquisition」の２つの主要な操作が行われます。
  val slowArea = new ClockingArea(slowClkCtrl.slowClockDomain) {
    // Global time
    // グローバルタイム：この部分では、グローバルな時間カウンタが定義されています。
    // それは24ビットの無符号整数（UInt）として表され、初期値はゼロで、各クロックサイクルで1ずつ増加します。
    val time = Reg(UInt(24 bits)) init 0 // Regと定義された値は、クロックサイクルに同期する
    time := time + 1

    val idBeamWords = Vec(Stream(PulseWithData()), nSensors)

    for (sensor <- 0 until nSensors) {

      // Sensor configurator
      // 'Sensor Configurator'はセンサが適切に動作するように設定を調整します。
      val configurator = new ts4231Configurator
      configurator.io.reconfigure := RegNext(True, init = False) // Generates a rising edge at startup
      configurator.io.d_in := BufferCC(d(sensor))
      d_out(sensor) := configurator.io.d_out
      d_oe(sensor) := configurator.io.d_oe
      configurator.io.e_in := BufferCC(e(sensor))
      e_out(sensor) := configurator.io.e_out
      e_oe(sensor) := configurator.io.e_oe

      // Pulse timer and data acquisition
      // 'Pulse Timer and Data Acquisition'はセンサからのパルスデータ（センサが物体を感知するために使用する光パルス）を取得し、そのデータをビームワード（パルスデータの集合）に変換します。
      val pulseTimer = new PulseTimer
      pulseTimer.io.time := time
      // BufferCC関数はクロッククロッシングバッファと呼ばれるもので、異なるクロックドメイン間の信号の安全な伝送を担当します
      pulseTimer.io.e := BufferCC(e(sensor))

      // Store the beamWord in a register when it has been acquired
      // ビームストリームの生成：
      // 上記ステップで取得したデータは最終的に単一のビームストリームに結合されます。
      // このストリームは、処理がブロックされるのを防ぐためにキューを介して管理されます。
      val beamWordStream = StreamCCByToggle(core.beamWords(sensor), clkCtrl.coreClockDomain, slowClkCtrl.slowClockDomain)
      val beamWord = Reg(beamWordStream.payload.clone())
      beamWordStream.ready := True
      when(beamWordStream.fire) {
        beamWord := beamWordStream.payload
      }

      // Pack pulse timing and beamWord in a single stream at the end of the pulse
      // パルスタイミングとビームワードをパルスの終わりで単一のストリームにパックします。
      val pulseWithData = PulseWithData()
      pulseWithData.id := sensor
      pulseWithData.pulse := pulseTimer.io.pulse
      pulseWithData.beamWord := beamWord
      val idBeamWord = pulseTimer.io.pulse.translateWith(pulseWithData)

      // Reset beamword to 0 when this beam has been processed
      // 0 is an invalid beam data (not possible to produce by LFSR)
      // This ensure that the pulse will report an invalid beamWord if
      // no data or not enough bits has been received
      // このビームが処理されたら、ビームワードを0にリセットする。
      // 0は無効なビームデータ（LFSRでは生成できない）。
      // 以下の場合、パルスは無効な beamWord を報告します。
      // データがない、または十分なビットが受信されていない場合
      when(idBeamWord.fire) {
        beamWord := 0
      }

      idBeamWords(sensor) << idBeamWord.toStream.queue(2)
    }

    // Create the combined beam stream.
    // The queue ensure that the pulse measurement is not blocked
    // 結合されたビーム ストリームを作成します。
    // キューによりパルス測定がブロックされないことが保証されます
    val beamsStream = StreamArbiterFactory.on(idBeamWords).queue(128)

    // Pulse identifier
    val pulseIdentifier = new PulseIdentifier
    pulseIdentifier.io.pulseIn << beamsStream

    // Pulse Offset finder
    val pulseOffsetFinder = new PulseOffsetFinder(speedMultiplier = 4)
    pulseOffsetFinder.io.pulseIn << pulseIdentifier.io.pulseOut

    val finalStream = pulseOffsetFinder.io.pulseOut

    // Pack data, fields are aligned on bytes boundary as much as possible
    // データをパックし、フィールドは可能な限りバイト境界に揃えられます
    //   0                   1                   2       
    //   0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3 
    //  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  |SID|   nPoly   |          Width                |
    //  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  |          Sync Offset            |0 0 0 0 0 0 0|
    //  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  |          Beam Word              |0 0 0 0 0 0 0|
    //  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    //  |                 Timestamp                     |
    //  +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
    val identifiedBeamStream = finalStream.translateWith(
      pulseOffsetFinder.io.pulseOut.payload.pulse.timestamp ## // 24 Bits
        B(0, 7 bits) ## finalStream.payload.beamWord ## // 24 Bits
        B(0, 7 bits) ## finalStream.payload.offset ## // 24 Bits
        finalStream.payload.pulse.width ## finalStream.payload.npoly ## finalStream.payload.id // 24 Bits
    )

    // Generate syncrhonization frame (all ones) at regular interval
    val syncStream = identifiedBeamStream.clone
    syncStream.payload.assignFromBits(Bits(identifiedBeamStream.payload.getBitsWidth bits).setAll())
    val syncTimeout = Timeout(2 Hz)
    syncStream.valid := syncTimeout
    when(syncTimeout) {
      syncTimeout.clear()
    }


    // データがUARTに出力される部分
    // まず UartCtrl オブジェクトを作成し、設定しています。その後、syncBeamStream というストリームを使用します。
    // これは、センサデータがパルスデータとマージされ、最終的に1つのストリームとしてパッケージ化される場所です。
    // Combine synchronization and beams, synchronzation has higher priotity (lower first)
    val syncBeamStream = StreamArbiterFactory.lowerFirst.onArgs(syncStream.stage(), identifiedBeamStream.stage())

    // Push datas in the UART converting form the measurement to bytes
    // UART にデータをプッシュし、測定値をバイトに変換します
    val uartCtrl = new UartCtrl()
    uartCtrl.io.config.setClockDivider(uartBaudrate)
    uartCtrl.io.config.frame.dataLength := 7 //8 bits
    uartCtrl.io.config.frame.parity := UartParityType.NONE
    uartCtrl.io.config.frame.stop := UartStopType.ONE
    uartCtrl.io.uart <> io.uart
    // ここでの StreamWidthAdapter は、EMAのバス幅をUARTが扱えるバイト単位に変換するための関数呼び出しであり、これによりデータ処理は最終化され、外部のUARTデバイスに対して送信可能な信号になります。
    // そして、その架橋を行い、UARTコントローラの書き込みインターフェースに接続（uartCtrl.io.write）します。
    // これにより、処理されたデータはUARTを通じて外部に送信されることになります。
    StreamWidthAdapter(syncBeamStream.queue(128), uartCtrl.io.write, endianness = LITTLE)

    // UART command handler
    val commandHandler = new CommandHandler
    commandHandler.io.input << uartCtrl.io.read

    // LED Control
    val ledControl = commandHandler.io.ledCommand.toReg()

    val slowBlink = RegInit(False)
    val slowBlinkTimer = Timeout(2 Hz)
    when(slowBlinkTimer) {
      slowBlinkTimer.clear()
      slowBlink := !slowBlink
    }
    val fastBlink = RegInit(False)
    val fastBlinkTimer = Timeout(8 Hz)
    when(fastBlinkTimer) {
      fastBlinkTimer.clear()
      fastBlink := !fastBlink
    }

    val ledDimming = RegInit(False)
    val ledCtr = Reg(UInt(4 bits))
    val ledDimmingTimer = Timeout(100000 Hz)
    when(ledDimmingTimer) {
      ledDimmingTimer.clear()
      ledCtr := ledCtr + 1
      ledDimming := ledCtr === 0
    }

    val ledMux = Seq((0, False), (1, slowBlink), (2, fastBlink), (3, True))

    io.led0 := !ledControl(0 to 1).muxListDc(ledMux)
    io.led1 := !ledControl(2 to 3).muxListDc(ledMux)
    io.led2 := !(ledControl(4 to 5).muxListDc(ledMux) & ledDimming)

    // Reset control
    val resetControl = commandHandler.io.resetCommand.toReg()

    val warmBoot = SB_WARMBOOT()
    warmBoot.S0 := False
    warmBoot.S1 := False
    warmBoot.BOOT := resetControl === 0xCF
  }
}

object GenerateTopLevel {
  def main(args: Array[String]) {
    val report = SpinalConfig()
      .generateVerilog(new LighthouseTopLevel)
    report.mergeRTLSource("blackboxes")
  }
}

//Top level simulation

import spinal.sim._
import spinal.core.sim._

object TopLevelSim {
  def main(args: Array[String]): Unit = {

    SimConfig.allOptimisation
      .addSimulatorFlag("-Wno-PINMISSING -I../../sim_rtl")
      .withWave
      .compile {
        val top = new LighthouseTopLevel

        top.clkCtrl.coreClockDomain.clock.simPublic()

        top
      }.doSim { dut =>
        dut.clkCtrl.coreClockDomain.forkStimulus(10)

        dut.io.d(0) #= false
        sleep(300)

        for (_ <- 0 to 5) {
          for (_ <- 0 to 8) {
            dut.clkCtrl.coreClockDomain.waitEdge(1)
            dut.io.d(0) #= true
          }
          dut.clkCtrl.coreClockDomain.waitRisingEdge(4)

          for (_ <- 0 to 8) {
            dut.clkCtrl.coreClockDomain.waitEdge(1)
            dut.io.d(0) #= true
          }
          dut.clkCtrl.coreClockDomain.waitRisingEdge(4)

          for (_ <- 0 to 16) {
            dut.clkCtrl.coreClockDomain.waitEdge(1)
            dut.io.d(0) #= true
          }

          dut.clkCtrl.coreClockDomain.waitRisingEdge(8)
        }

        sleep((10 * 24000000) / 10)

        simSuccess()
      }
  }
}

case class SoftLfsr(poly: Int, start: Int = 1) {
  var state = start

  def parity(bb: Int): Int = {
    var b = bb
    b ^= b >> 16
    b ^= b >> 8
    b ^= b >> 4
    b ^= b >> 2
    b ^= b >> 1
    b &= 1
    b
  }

  def iterate() = {
    var b = state & poly
    b = parity(b)
    state = (state << 1) | b
    state &= (1 << 17) - 1
  }
}

object SoftLfsr {
  def getStateAtOffset(poly: Int, offset: Int): Int = {
    val lfsr = SoftLfsr(poly)
    for (i <- 0 until offset) {
      lfsr.iterate()
    }
    lfsr.state
  }
}

object TopLevelSimWithSalaeData {
  def analyze(state: Int): Seq[Option[Int]] = {
    val polys = Seq(0x00012BD0, 0x0001CF73)

    var offsets = mutable.ArrayBuffer[Option[Int]]()

    polys.foreach { poly =>
      val lfst = SoftLfsr(poly)

      var offset = 0
      while (offset < 131072 && lfst.state != state) {
        lfst.iterate()
        offset += 1
      }

      if (offset < 131072) {
        offsets += Some(offset)
      } else {
        offsets += None
      }
    }

    offsets
  }

  def main(args: Array[String]): Unit = {
    val sampleReader = CSVReader.open(new File("test_data/take2_5.csv"))
    // Skip header
    sampleReader.readNext()

    var d = false
    var e = false


    SimConfig.allOptimisation
      .addSimulatorFlag("-Wno-PINMISSING -I../../sim_rtl")
      .withWave
      .compile {
        val top = new LighthouseTopLevel(uartBaudrate = 1 MHz,
          useDdrDecoder = true,
          frequency = 48 MHz)

        top.clkCtrl.coreClockDomain.clock.simPublic() // To clock the design
        top.slowArea.beamsStream.valid.simPublic() // To read the beams before the UART
        top.slowArea.beamsStream.payload.simPublic() // To read the beams before the UART
        top.slowArea.beamsStream.ready.simPublic() // To read the beams before the UART


        top
      }.doSim { dut =>

        dut.clkCtrl.coreClockDomain.forkStimulus(10)

        var signalThread = fork {
          var currentTime = TimeNumber(0)
          var sampleTime = TimeNumber(-1)
          val clockHalfPeriod = dut.clkCtrl.coreClockDomain.frequency.getValue.toTime / 2

          while (currentTime.toBigDecimal < (500 ms).toBigDecimal) {
            val sample = sampleReader.readNext()
            if (sample.isEmpty) {
              println("Empty sample?!")
              return
            }
            sampleTime = TimeNumber(BigDecimal(sample.get(0)))

            if (true || sampleTime.toBigDecimal >= currentTime.toBigDecimal) {

              val waitEdge = ((sampleTime.toBigDecimal - currentTime.toBigDecimal) / clockHalfPeriod.toBigDecimal).toInt
              // println(waitEdge)
              if (waitEdge > 0) {
                // dut.clkCtrl.coreClockDomain.waitEdge(waitEdgbeamWorde)
                // currentTime += clockHalfPeriod * waitEdge


                if (waitEdge > 10000) {
                  for (i <- 0 to 10000) {
                    dut.io.d(0) #= d
                    dut.io.e(0) #= e
                    dut.clkCtrl.coreClockDomain.waitEdge(1)
                  }
                  currentTime = sampleTime
                } else {
                  for (i <- 0 to waitEdge - 1) {
                    dut.io.d(0) #= d
                    dut.io.e(0) #= e
                    dut.clkCtrl.coreClockDomain.waitEdge(1)
                  }
                  currentTime += clockHalfPeriod * waitEdge
                }
              }

              d = if (sample.get(1).stripPrefix(" ").toInt == 0) false else true
              e = if (sample.get(2).stripPrefix(" ").toInt == 0) false else true
            }
          }
        }

        signalThread.join()

        simSuccess()
      }
  }
}
