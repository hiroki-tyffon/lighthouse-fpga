/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
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

package lighthouse

import spinal.core._
import spinal.lib._

/**
  * Shift-register implemented counter
  *
  * Implements a counter using a shift register. This means that it is going
  * to be using one flip-flop per count and it is not (easily) possible to
  * get the current count. Though, it can be clocked very fast and comparing
  * the current count is very efficient
  *
  * @param size Size (ie. max count) of the counter
  */
class ShiftCounter(size: Int) extends Area {
    val sr = Reg(Bits(size+1 bits)) init 1

    def reset(at: Int) = sr := (1<<at)
    def apply(i: Int) = sr(i)
    def <(that: Int) = sr(0 to that-1).orR
    def <=(that: Int) = sr(0 to that).orR
    def >(that: Int) = sr(that+1 to size).orR
    def >=(that: Int) = sr(that to size).orR
    def ===(that: Int) = this(that)

    def increment(i: Int = 1) = sr := sr.rotateLeft(i)
}

// おそらく2つの信号を受け取り、それらのエッジを検出するためのモジュール
case class Ddr() extends Bundle {
    val v = Vec(Bool, 2)

    def apply(i: Int) : Bool = v(i)
    def edge(i: Int) : Bool = {
        if (i==0) RegNext(v(1)) ^ v(0)
        else v(0) ^ v(1)
    }
}

// ビット幅（width）を指定したシフトレジスタの実装を提供します。
// このクラスは、データの一時的な保存とシフト操作（新しいデータの追加と古いデータの削除）を行います。
// このクラスは、新しいデータが入力ストリームに到着すると、
// それをバッファの最下位ビットに追加し、最上位ビットを削除します（シフト操作）。
// バッファが満杯でない場合、またはデータが読み出された場合にのみ、新しいデータが追加されます。
// また、出力ストリームがデータを要求したとき、またはリセット信号が受信されたときに、バッファの内容がリセットされます。
class ShiftBuffer(width: Int) extends Component {
    val io = new Bundle {
        // 外部からのデータ入力ストリームを受け取ります。これはslaveとして定義されているため、外部から制御されます。
        val dataIn = slave Stream(Bool)
        // 内部のデータを外部に出力するストリームを提供します。これはmasterとして定義されているため、このクラスが制御します。
        val dataOut = master Stream(Bits(width bits))

        // 外部からの信号でバッファをリセットします。
        val resetBuffer = in Bool
    }

    val buffer = Reg(Bits(width+1 bits)) init 1
    // バッファが満杯である（最上位ビットがセットされている）ことを示すフラグです。
    val stalled = Bool
    val read = RegInit(False)

    stalled := buffer.msb
    io.dataIn.ready := !stalled
    io.dataOut.valid := stalled && !read
    io.dataOut.payload := buffer(0 to width-1)

    when(io.resetBuffer) {
        buffer := 1
        read := False
    }.elsewhen(!stalled && io.dataIn.fire) {
        buffer(1 to width) := buffer(0 to width-1)
        buffer.lsb := io.dataIn.payload
    }

    when(io.dataOut.fire) {
        read := True
    }
}

class Lfsr(poly: Int, length: Int) extends Area {
    val state = RegInit(B(0, length bits))

    def :=(newState: Bits) = this.state := newState
    def apply: Bits = this.state

    // レジスタの状態を更新するメソッドです。
    // 現在の状態と多項式（poly）のビットごとのANDを取り、その結果のビットごとのXORを計算します。
    // その結果を新しいビットとしてレジスタの左端に追加し、右端のビットを削除します。
    def iterate() = {
        val b = (state & poly).xorR
        state := state(0 to length-2) ## b.asBits
    }
}

// Useful system constants

// 特定の多項式を検索し、それらの状態遷移を追跡するための基本的なパラメータとして機能します。

// PolyFinderクラスでは、constants.Polysの各多項式に対して処理を行い、その状態が目標状態と一致するかどうかを確認します。
// 状態が目標状態と一致した場合、その多項式が見つかったとして、検索を終了します。

// OffsetFinderクラスでは、constants.Polysの各多項式に対して、特定のオフセットでのLFSRの状態を計算し、それをメモリに保存します。
// これは、特定のオフセットでの多項式の状態を高速に取得するための準備ステップとなります。
object constants {
  val Polys = Seq(0x0001D258, 0x00017E04,
                  0x0001FF6B, 0x00013F67,
                  0x0001B9EE, 0x000198D1,
                  0x000178C7, 0x00018A55,
                  0x00015777, 0x0001D911,
                  0x00015769, 0x0001991F,
                  0x00012BD0, 0x0001CF73,
                  0x0001365D, 0x000197F5,
                  0x000194A0, 0x0001B279,
                  0x00013A34, 0x0001AE41,
                  0x000180D4, 0x00017891,
                  0x00012E64, 0x00017C72,
                  0x00019C6D, 0x00013F32,
                  0x0001AE14, 0x00014E76,
                  0x00013C97, 0x000130CB,
                  0x00013750, 0x0001CB8D)
}

case class SB_WARMBOOT() extends BlackBox {
    val BOOT = in Bool
    val S1 = in Bool
    val S0 = in Bool
}
