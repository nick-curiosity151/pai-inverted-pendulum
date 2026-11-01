#!/usr/bin/env python3
"""bag_to_npz.py で作った npz を、LeRobotDataset v2.1 形式に変換する。

使い方: pixi run python /workspace/pend/npz2lerobot.py [npzフォルダ] [出力フォルダ] [fps]
"""
import sys
import glob
import shutil
import importlib.util
from pathlib import Path

import numpy as np
import torch
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset

NPZ_DIR = sys.argv[1] if len(sys.argv) > 1 else "/workspace/dataset_npz"
OUT_DIR = Path(sys.argv[2] if len(sys.argv) > 2 else "/workspace/lerobot_dataset/pend_balance")
FPS = int(sys.argv[3]) if len(sys.argv) > 3 else 50
FEATURES_PY = "/workspace/training/pend_features.py"


def load_config(path):
    # 講義の rosbag2lerobot.py と同じ方法で、定義ファイルを読み込む
    spec = importlib.util.spec_from_file_location("pend_features", path)
    cfg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cfg)
    return cfg


def resample(d, fps):
    # 200 Hz のデータから、1/fps 秒ごとのサンプルを取り出す
    t = d["t"]
    state = d["state"]
    action = d["action"][:, 0]
    target = np.arange(t[0], t[-1], 1.0 / fps)
    idx = np.clip(np.searchsorted(t, target), 0, len(t) - 1)

    s = state[idx].copy()
    a = action[idx]
    # v_prev は、間引いたあとの「1つ前のフレームの指令」に置き換える
    # （推論も同じ周期で動かすので、条件をそろえるため）
    s[1:, 2] = a[:-1]
    s[0, 2] = 0.0

    # 行動は [前後の速度, 旋回の速度(常に0)] の2次元にする
    a2 = np.stack([a, np.zeros_like(a)], axis=1)
    return s.astype(np.float32), a2.astype(np.float32)


def main():
    cfg = load_config(FEATURES_PY)

    # 出力先がすでにあれば作り直す
    if OUT_DIR.exists():
        print(f"{OUT_DIR} を削除して作り直します")
        shutil.rmtree(OUT_DIR)
    OUT_DIR.parent.mkdir(parents=True, exist_ok=True)

    ds = LeRobotDataset.create(
        repo_id=cfg.repo_id,
        fps=FPS,
        root=OUT_DIR,
        robot_type=cfg.robot_type,
        features=cfg.features,
        use_videos=False,
    )
    if cfg.task_name not in ds.meta.task_to_task_index:
        ds.meta.add_task(cfg.task_name)

    files = sorted(glob.glob(f"{NPZ_DIR}/*.npz"))
    total = 0
    for ep, f in enumerate(files):
        s, a = resample(np.load(f), FPS)
        for i in range(len(a)):
            ds.add_frame({
                "observation.state": torch.from_numpy(s[i]),
                "observation.environment_state": torch.from_numpy(s[i]),
                "action": torch.from_numpy(a[i]),
                "task": cfg.task_name,
            })
        ds.save_episode()
        total += len(a)
        print(f"エピソード {ep}: {Path(f).name} → {len(a)} フレーム "
              f"(ピッチ角の最大 {np.degrees(s[:, 0].max()):+.1f} 度)")

    print(f"完了: {len(files)} エピソード, 合計 {total} フレーム → {OUT_DIR}")


if __name__ == "__main__":
    main()

