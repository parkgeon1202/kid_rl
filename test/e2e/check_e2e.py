import csv, math, sys
import numpy as np, onnx, onnxruntime as ort
T, DT = 0.8544895701575631, 0.02
import os
MODEL = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data", "policy_gait_phase.onnx")
rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else "run.csv")))
J = [c[4:] for c in rows[0] if c.startswith("pos_")]
meta = {p.key: p.value for p in onnx.load(MODEL).metadata_props}
names = meta["joint_names"].split(","); dall = [float(x) for x in meta["default_joint_pos"].split(",")]
D = np.array([dall[names.index(j)] for j in J])
sess = ort.InferenceSession(MODEL); inp = sess.get_inputs()[0].name
f = lambda r, k: float(r[k])
P_all = [i for i, r in enumerate(rows) if r["mode"] == "POLICY_RUNNING"]
assert P_all, "no POLICY_RUNNING rows"
ran = lambda i: any(rows[i]["action_"+j] != rows[i-1]["action_"+j] for j in J)
first = next(i for i in P_all if ran(i))          # 첫 추론 틱 (진입 틱 다음)
P = [i for i in P_all if i >= first]
skipped = [i for i in P[1:] if not ran(i)]
print(f"POLICY_RUNNING {len(P_all)} 틱, 진입 틱 → 첫 추론까지 {first - P_all[0]} 틱, "
      f"첫 추론 이후 추론 건너뛴 틱: {len(skipped)} / {len(P)-1}")
p0 = first

# 1) 첫 추론 틱 위상 = 0 (학습의 리셋 직후 관측)
print(f"[1] 첫 추론 틱 gait_phase = {rows[p0]['gait_phase']}  (진입 틱 {rows[P_all[0]]['gait_phase']}, 진입 전 {rows[P_all[0]-1]['gait_phase']})")

# 2) 위상 = (첫 추론 이후 실제 경과 시간 / T) mod 1
elapsed, worst_rt, worst_k = 0.0, 0.0, 0.0
for n, i in enumerate(P):
    if n > 0: elapsed += f(rows[i], "dt_s")
    ph = f(rows[i], "gait_phase")
    d = abs(ph - (elapsed / T) % 1.0); worst_rt = max(worst_rt, min(d, 1 - d))
    d2 = abs(ph - (n * DT / T) % 1.0); worst_k = max(worst_k, min(d2, 1 - d2))
dts = [f(rows[i], "dt_s") * 1000 for i in P[1:]]
print(f"[2] 위상 vs 실제 경과시간: 최대 차이 {worst_rt:.2e} 주기 (CSV 6자리 반올림 수준이면 통과)")
print(f"    참고: 틱 수 k×0.02 기준과의 최대 차이 {worst_k:.2e} 주기 = {worst_k*T*1000:.2f} ms "
      f"(dt 중앙 {np.median(dts):.2f} ms, 최소 {min(dts):.2f}, 최대 {max(dts):.2f})")

# 3) ONNX 재생: CSV 관측(위상 포함) -> 기록된 action
def obs(r, last, phase):
    a = 2 * math.pi * phase
    return np.concatenate([[f(r,"gyro_x"), f(r,"gyro_y"), f(r,"gyro_z")],
                           [f(r,"gravity_x"), f(r,"gravity_y"), f(r,"gravity_z")],
                           [f(r,"pos_"+j) - D[k] for k, j in enumerate(J)],
                           [f(r,"vel_"+j) for j in J], last,
                           [f(r,"cmd_vx"), f(r,"cmd_vy"), f(r,"cmd_wz")],
                           [math.sin(a), math.cos(a)]]).astype(np.float32)[None]
def replay(shift_ticks):
    # clip(±5) 에 걸린 성분은 위상이 틀려도 둘 다 5 라 비교가 무뎌진다. 걸리지 않은 성분만 본다.
    errs, sat = [], 0
    for n, i in enumerate(P):
        r = rows[i]
        last = np.zeros(25) if n == 0 else np.array([f(rows[i-1], "action_"+j) for j in J])
        ph = (f(r, "gait_phase") + shift_ticks * DT / T) % 1.0
        a = sess.run(None, {inp: obs(r, last, ph)})[0][0]
        logged = np.array([f(r, "action_"+j) for j in J])
        mask = (np.abs(logged) < 4.999) & (np.abs(a) < 4.999)
        sat += int((~mask).sum())
        if mask.any():
            errs.append(np.abs(a - logged)[mask].max())
    return np.array(errs), sat
e, sat = replay(0)
print(f"[3] ONNX 재생 (위상 그대로, clip 안 걸린 성분): action 오차 중앙 {np.median(e):.2e}, "
      f"최대 {e.max():.2e}  (clip 에 걸려 뺀 성분 {sat}/{len(P)*25})")
for sh in (-1, +1):
    es, _ = replay(sh)
    print(f"[4] 위상을 {sh:+d} 틱 밀면: 오차 중앙 {np.median(es):.2e}, 최대 {es.max():.2e}  "
          f"-> 중앙값 기준 {np.median(es)/np.median(e):.0f}배")
print("[5] 위상을 빼고 84 차원으로 넣으면 모델 입력 크기가 달라 load 단계에서 거부된다 (policy_runner_test)")
