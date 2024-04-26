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

import spinal.core._
import spinal.lib._

import lighthouse.Lfsr
import lighthouse.constants

// PolyFinderは、特定の多項式（そのタップ構成で表される）を検索するハードウェアコンポーネントを実装したScalaクラスです。
// このコンポーネントは、線形帰還シフトレジスタ（LFSR）の状態遷移を探します。
// 具体的には、PolyFinderは以下のような動作をします：
// 1. startイベントが発生すると、検索が開始されます。
//      このとき、各LFSRの状態はstartStateの入力で初期化されます。
// 2. 検索中（searchingがtrue）は、各LFSRが反復されます。
//      その状態がtargetStateと一致し、カウンタ条件を満たす場合、対応するビットがfoundレジスタでtrueに設定され、searching状態がfalseに設定されます。
// 3. foundがtrueになった場合、見つかった多項式のインデックスがpolyFound出力に割り当てられます。
// 4. doneイベントは、検索が終了した（searchingがfalseになった）ときに発生します。
//      このクラスは、そのタップ構成（LFSR）で表される複数の多項式に対して並列検索を行うハードウェアコンポーネントを実装しています。
//      コンポーネントは初期状態と目標状態を入力とし、指定された反復回数内で目標状態を生成する多項式のLFSRを検索します。
//      見つかった多項式のインデックスやその他のステータス信号が出力として提供されます。

class PolyFinder extends Component {

    // 選択範囲の開始
    val io = new Bundle {
        // 開始状態、目標状態、最大ティック数を入力として定義します
        val startState = in Bits(17 bits)
        val targetState = in Bits(17 bits)
        val maxTick = in UInt(10 bits)

        // スタートイベントを定義します
        val start = slave Event

        // 見つかった多項式、そのインデックス、完了イベントを出力として定義します
        val found = out Bool
        val polyFound = out UInt(5 bits)
        val done = master Event
    }

    // 検索中かどうかを示すフラグと、見つかった多項式を保存するレジスタを定義します
    val searching = RegInit(False)
    val found = RegInit(B(0, constants.Polys.length bits))

    // 出力を設定します
    io.polyFound := OHToUInt(found)
    io.done.valid := searching.fall()
    io.found := found.orR
    io.start.ready := True

    // カウンタを定義し、検索中にデクリメントします
    val counter = RegInit(U(0, io.maxTick.getBitsWidth bits))
    when (searching) {
        counter := counter - 1

        // カウンタが0になったら、検索を終了します
        when (counter === 0) {
            searching := False
            found := 0
        }
    }.otherwise {
        counter := io.maxTick
    }

    // スタートイベントが発生したら、検索を開始します
    when (io.start.fire) {
        searching := True
        found := 0
    }

    // 各多項式の状態を保存するベクタを定義します
    val states = Vec(Bits(17 bits), constants.Polys.length)

    // 各多項式に対して処理を行います
    for (i <- 0 to constants.Polys.length - 1) {
        val lfsr = new Lfsr(constants.Polys(i), 17)
        states(i) := lfsr.state

        // スタートイベントが発生したら、LFSRの状態を開始状態で初期化します
        when (io.start.fire) {
            lfsr.state := io.startState
        }

        // 検索中は、LFSRを反復し、その状態が目標状態と一致するかどうかを確認します
        when (searching) {
            lfsr.iterate()

            // 状態が目標状態と一致したら、その多項式を見つけたとして、検索を終了します
            when ((counter(2 to counter.getBitsWidth-1) === 0) && (lfsr.state === io.targetState)) {
                found(i) := True
                searching := False
            }
        }
    }
    // 選択範囲の終了
}


import spinal.sim._
import spinal.core.sim._

object PolyFinderSim {
  def main(args: Array[String]): Unit = {
    SimConfig.allOptimisation
            .addSimulatorFlag("-I../../sim_rtl")
            .withWave
            .compile (new PolyFinder).doSim{ dut =>
      dut.clockDomain.forkStimulus(10)

      dut.io.startState #= 0x1fe72
      dut.io.targetState #= 0x0bd25
      dut.io.maxTick #= 183
      dut.io.start.valid #= false

      dut.clockDomain.waitRisingEdge()
      dut.io.start.valid #= true
      dut.clockDomain.waitRisingEdge()
      dut.io.start.valid #= false

      dut.clockDomain.waitRisingEdge(200)

      simSuccess()
    }
  }
}
