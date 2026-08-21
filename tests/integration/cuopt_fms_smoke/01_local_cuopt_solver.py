#!/usr/bin/env python3
"""Minimal local cuOpt solver for the integration smoke test.

Run this with the Python environment where cuOpt is installed.
It writes `cuopt_route.json` next to this script and then exits.
"""

from __future__ import annotations

import json
from pathlib import Path

import cudf
import numpy as np
from cuopt import routing

BASE_DIR = Path(__file__).resolve().parent
OUTPUT_PATH = BASE_DIR / "cuopt_route.json"

# Shared logical map. These IDs/coordinates must match the Isaac Sim world.
LOCATIONS = {
    0: {"name": "Depot", "x": 0.0, "y": 0.0},
    1: {"name": "Parts_Supermarket", "x": -3.0, "y": 2.0},
    2: {"name": "Assembly_Cell_A", "x": 3.0, "y": 2.0},
    3: {"name": "Assembly_Cell_B", "x": 3.0, "y": -2.0},
}


def build_cost_matrix() -> tuple[cudf.DataFrame, np.ndarray]:
    """Build a Euclidean distance matrix for this tiny connectivity test."""
    location_ids = sorted(LOCATIONS)
    xy = np.array(
        [[LOCATIONS[i]["x"], LOCATIONS[i]["y"]] for i in location_ids],
        dtype=np.float32,
    )
    delta = xy[:, None, :] - xy[None, :, :]
    matrix = np.linalg.norm(delta, axis=2).astype(np.float32)
    return cudf.DataFrame(matrix), matrix


def solve_route() -> dict:
    cost_matrix, matrix_np = build_cost_matrix()

    # One AMR; depot is location 0. Visit locations 1, 2, 3 once each.
    order_locations = cudf.Series([1, 2, 3], dtype="int32") # 명령 당 수행 장소

    data_model = routing.DataModel(     # 경로문제의 크기 정의
        len(LOCATIONS),                 # 로케이션의 개수
        1,  # n_vehicles                # vehicle의 개수
        len(order_locations),           # 오더의 개수
    )
    data_model.add_cost_matrix(cost_matrix)     # 비용행렬 데이터모델에 등록
    data_model.set_order_locations(order_locations)     # 명령 당 수행 장소 데이터모델에 등록

    settings = routing.SolverSettings()     # 솔버 설정 세팅
    settings.set_time_limit(1.0)            # 최대 계산시간 설정

    solution = routing.Solve(data_model, settings)      # 최적해 계산 실행
    status = int(solution.get_status())                 # 계산 결과 반환
    if status != 0:                                     # 계산 실패 시 에러발생
        raise RuntimeError(
            f"cuOpt failed: status={status}, message={solution.get_message()}"
        )

    route_df = solution.get_route()     # 계산 결과에서 최적루트(실제 차량 경로 테이블) 꺼내와 반환
    route = [int(v) for v in route_df["location"].to_arrow().to_pylist()]     # gpu/분석용 자료구조를 일반python/fms에서 쓰기 쉬운 구조로 변환

    return {
        "solver": "NVIDIA cuOpt local Python API",
        "purpose": "minimal local cuOpt -> FMS -> ROS2 -> Isaac Sim integration test",
        "route": route,
        "locations": {str(k): v for k, v in LOCATIONS.items()},
        "cost_matrix": matrix_np.tolist(),
    }, route_df     # fms전달용 일반 파이선 dict, 디버깅 및 확인용 dataframe. 2종류 반환.


def main() -> None:
    result, route_df = solve_route()

    OUTPUT_PATH.write_text(
        json.dumps(result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print("\n=== cuOpt local smoke route ===")
    print(route_df)
    print(f"\nRoute IDs : {result['route']}")
    print(f"Saved     : {OUTPUT_PATH}")
    print("RESULT    : CUOPT_ROUTE_READY")


if __name__ == "__main__":
    main()
