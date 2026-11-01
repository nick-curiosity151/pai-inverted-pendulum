# 倒立振子（ACT 模倣学習）用のデータセット定義
robot_type = "wheeled_inverted_pendulum"
repo_id = "pend_balance"
task_name = "balance"

# 観測の各要素の名前: ピッチ角[rad], ピッチ角速度[rad/s], 直前の速度指令[m/s], 移動量[m]
STATE_NAMES = ["pitch", "pitch_rate", "v_prev", "x"]

features = {
    "observation.state": {
        "dtype": "float32", "shape": (4,), "names": STATE_NAMES,
    },
    "observation.environment_state": {
        "dtype": "float32", "shape": (4,), "names": STATE_NAMES,
    },
    "action": {
        "dtype": "float32", "shape": (2,), "names": ["v_cmd", "w_cmd"],
    },
}

