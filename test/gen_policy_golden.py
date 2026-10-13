#!/usr/bin/env python3
# Copyright 2026 kid_rl authors
# SPDX-License-Identifier: Apache-2.0
"""policy_runner_test 가 쓰는 모델별 기댓값을 만든다.

관측은 '에피소드 첫 관측'과 같은 모양이다: 모든 항목 0, 위상을 쓰는 모델이면 마지막 두 칸
(sin, cos) = (0, 1). 모델을 바꾸면 이 스크립트로 기댓값을 다시 만든다.

  python3 test/gen_policy_golden.py MODEL.onnx OUT.golden
"""
import sys

import numpy as np
import onnx
import onnxruntime as ort

model_path, out_path = sys.argv[1], sys.argv[2]
meta = {p.key: p.value for p in onnx.load(model_path).metadata_props}
names = meta.get("observation_names", "").split(",")
uses_gait = names[-1:] == ["gait_phase"]
sess = ort.InferenceSession(model_path)
size = sess.get_inputs()[0].shape[1]
obs = np.zeros((1, size), np.float32)
if uses_gait:
    obs[0, -1] = 1.0  # cos(0)
act = sess.run(None, {sess.get_inputs()[0].name: obs})[0][0]
with open(out_path, "w") as f:
    f.write(f"# {model_path.split('/')[-1]} run_path={meta.get('run_path')}\n")
    f.write(f"{size} {int(uses_gait)}\n")
    f.write(" ".join(repr(float(v)) for v in obs[0]) + "\n")
    f.write(" ".join(repr(float(v)) for v in act) + "\n")
print(out_path, "size", size, "gait", uses_gait)
