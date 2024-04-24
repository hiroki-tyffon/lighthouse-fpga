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

//  はい、この Scala のコードについて日本語で説明します。
//
//この コードは、ハードウェア記述言語 (おそらく Chisel) で記述されたハードウェアコンポーネント `PolyFinder` を定義しています。このコンポーネントは、特定の多項式 (その tap 構成で表される) を検索する操作を行います。
//
//1. `PolyFinder` クラスは `Component` クラスを継承しており、ベースクラスとなっています。
//
//2. `io` バンドルはコンポーネントのインターフェースを定義しており、入出力信号が含まれています。
//   - `startState`: 検索の初期状態を表す 17 ビットの入力信号
//   - `targetState`: 検索対象の目標状態を表す 17 ビットの入力信号 
//   - `maxTick`: 最大反復回数を表す 10 ビットの入力信号
//   - `start`: 検索をトリガーする入力イベント信号
//   - `found`: 目標状態が見つかったかどうかを示す出力ブール信号
//   - `polyFound`: 見つかった多項式のインデックスを表す 5 ビットの出力信号
//   - `done`: 検索が完了したことを示す出力イベント信号
//
//3. `searching` と `found` の 2 つのレジスタがあります。`searching` は検索が進行中かどうかを示し、`found` は見つかった多項式のインデックスをワンホットエンコーディングしたベクトルです。
//
//4. 出力信号 `polyFound`、`done`、`found`、`start.ready` に内部状態とレジスタに基づいた値が割り当てられています。
//
//5. `counter` レジスタは、現在の検索における残りの反復回数を保持しています。
//
//6. 入力信号と内部状態に基づいて、`searching`、`found`、`counter` レジスタを更新する制御ロジックが含まれています。
//
//7. `states` という `Bits` の `Vec` が、多項式の数 (`constants.Polys.length`) の長さで定義されています。この配列は各多項式の線形帰還シフトレジスタ (LFSR) の状態を保持します。
//
//8. ループ内で、各多項式に対して `Lfsr` インスタンスが作成され、その状態が `states` 配列の対応する要素に割り当てられています。
//
//9. ループ内では、入力信号と検索状態に基づいて LFSR の状態が更新されます。`start` イベントが発生すると、LFSR の状態が `startState` の入力で初期化されます。検索中 (`searching` が true) は、LFSR が反復され、その状態が `targetState` と一致し、カウンタ条件を満たす場合、対応するビットが `found` レジスタで true に設定され、`searching` 状態が false に設定されます。
//
//全体として、このコードは、その tap 構成 (LFSR) で表される複数の多項式に対して並列検索を行うハードウェアコンポーネントを実装しています。コンポーネントは初期状態と目標状態を入力とし、指定された反復回数内で目標状態を生成する多項式の LFSR を検索します。見つかった多項式のインデックスやその他のステータス信号が出力として提供されます。
class PolyFinder extends Component {

    val io = new Bundle {
        val startState = in Bits(17 bits)
        val targetState = in Bits(17 bits)
        val maxTick = in UInt(10 bits)

        val start = slave Event

        val found = out Bool
        val polyFound = out UInt(5 bits)
        val done = master Event
    }

    val searching = RegInit(False)
    val found = RegInit(B(0, constants.Polys.length bits))

    io.polyFound := OHToUInt(found)
    io.done.valid := searching.fall()
    io.found := found.orR
    io.start.ready := True

    val counter = RegInit(U(0, io.maxTick.getBitsWidth bits))
    when (searching) {
        counter := counter - 1

        when (counter === 0) {
            searching := False
            found := 0
        }
    }.otherwise {
        counter := io.maxTick
    }

    when (io.start.fire) {
        searching := True
        found := 0
    }

    val states = Vec(Bits(17 bits), constants.Polys.length)

    for (i <- 0 to constants.Polys.length - 1) {
        val lfsr = new Lfsr(constants.Polys(i), 17)
        states(i) := lfsr.state

        when (io.start.fire) {
            lfsr.state := io.startState
        }

        when (searching) {
            lfsr.iterate()

            when ((counter(2 to counter.getBitsWidth-1) === 0) && (lfsr.state === io.targetState)) {
                found(i) := True
                searching := False
            }
        }
    }
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
