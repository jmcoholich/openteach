#!/usr/bin/env bash
set -euo pipefail

# thread3_rollout_0	Failure
# thread3_rollout_1	Failure
# thread3_rollout_2	Failure
# thread3_rollout_3	Failure
# thread3_rollout_4	Success
# thread3_rollout_5	Failure
# thread3_rollout_6	Failure
# thread3_rollout_7	Success
# thread3_rollout_8	Failure
# thread3_rollout_9	Failure
# thread3_rollout_10	Failure
# thread3_rollout_11	Failure
# thread3_rollout_12	Failure
# thread3_rollout_13	Failure
# thread3_rollout_14	Failure
# thread3_rollout_15	Failure
# thread3_rollout_16	Success
# thread3_rollout_17	Failure
# thread3_rollout_18	Failure
# thread3_rollout_19	Failure
# thread3_rollout_20	Failure
# thread3_rollout_21	Failure
# thread3_rollout_22	Failure
# thread3_rollout_23	Failure
# thread3_rollout_24	Failure
# thread3_rollout_25	Failure
# thread3_rollout_26	Success
# thread3_rollout_27	Success
# thread3_rollout_28	Failure
# thread3_rollout_29	Success
# thread3_rollout_30	Success
# thread3_rollout_31	Failure
# thread3_rollout_32	Failure
# thread3_rollout_33	Failure
# thread3_rollout_34	Failure
# thread3_rollout_35	Failure
# thread3_rollout_36	Failure
# thread3_rollout_37	Failure
# thread3_rollout_38	Success
# thread3_rollout_39	Failure
# thread3_rollout_40	Success
# thread3_rollout_41	Failure
# thread3_rollout_42	Failure
# thread3_rollout_43	Failure
# thread3_rollout_44	Failure
# thread3_rollout_45	Failure
# thread3_rollout_46	Failure
# thread3_rollout_47	Failure
# thread3_rollout_48	Failure
# thread3_rollout_49	Success
# thread3_rollout_50	Failure
# thread3_rollout_51	Failure
# thread3_rollout_52	Failure
# thread3_rollout_53	Failure
# thread3_rollout_54	Failure
# thread3_rollout_55	Failure
# thread3_rollout_56	Failure
# thread3_rollout_57	Failure
# thread3_rollout_58	Failure
# thread3_rollout_59	Partial Success
# thread3_rollout_60	Failure
# thread3_rollout_61	Failure
# thread3_rollout_62	Failure
# thread3_rollout_63	Failure
# thread3_rollout_64	Failure
# thread3_rollout_65	Failure
# thread3_rollout_66	Failure
# thread3_rollout_67	Partial Success
# thread3_rollout_68	Failure
# thread3_rollout_69	Failure
# thread3_rollout_70	Failure
# thread3_rollout_71	Failure
# thread3_rollout_72	Failure
# thread3_rollout_73	Failure
# thread3_rollout_74	Failure

ROOT=/data3/extracted_data/thread3_rollouts_w_dagger
DEST="$ROOT/full_rollout_h5_files"
SPLICE=/home/jeremiah/openteach/splice_demos.py
VISUALIZE=/home/jeremiah/openteach/visualize_demo.py
mkdir -p "$DEST"

success=(4 7 16 26 27 29 30 38 40 49)
needs_dagger=(0 1 2 3 5 6 8 9 10 11 12 13 14 15 17 18 19 20 21 22 23 24 25 28 31 32 33 34 35 36 37 39 41 42 43 44 45 46 47 48 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65 66 67 68 69 70 71 72 73 74)

for i in "${success[@]}"; do
    src="$ROOT/demonstration_thread3_rollout_$i/demo_thread3_rollout_$i.h5"
    out="$DEST/demo_thread3_full_rollout_$i.h5"
    if [ -e "$out" ]; then
        echo "Skipping successful rollout $i; output already exists: $out"
        continue
    fi
    echo "Moving successful rollout $i -> $out"
    mv "$src" "$out"
done

for i in "${needs_dagger[@]}"; do
    rollout="$ROOT/demonstration_thread3_rollout_$i/demo_thread3_rollout_$i.h5"
    dagger="$ROOT/demonstration_thread3_dagger_$i/demo_thread3_dagger_$i.h5"
    out="$DEST/demo_thread3_full_rollout_$i.h5"
    if [ -e "$out" ]; then
        echo "Skipping spliced rollout $i; output already exists: $out"
        continue
    fi
    if [ "$i" = 59 ] || [ "$i" = 67 ]; then
        echo "Patching dagger $i gripper_action nulls to -1"
        python - "$dagger" <<'PY'
import sys

import h5py
import numpy as np

path = sys.argv[1]
with h5py.File(path, "a") as h5f:
    values = h5f["gripper_action"][:]
    if np.issubdtype(values.dtype, np.number):
        print(f"{path} gripper_action is already numeric; leaving it unchanged")
        raise SystemExit(0)
    if any(v.decode("utf-8").strip().lower() != "null" for v in values):
        raise ValueError(f"Unexpected non-null gripper_action values in {path}: {values!r}")
    del h5f["gripper_action"]
    h5f.create_dataset("gripper_action", data=np.full(values.shape, -1.0, dtype=np.float64))
PY
    fi
    echo "Splicing rollout $i + dagger $i -> $out"
    python "$SPLICE" "$rollout" "$dagger" "thread3_full_rollout_$i" --data_root "$DEST" --overwrite
    mv "$DEST/demonstration_thread3_full_rollout_$i/demo_thread3_full_rollout_$i.h5" "$out"
    rmdir "$DEST/demonstration_thread3_full_rollout_$i"
done

for i in {70..74}; do
    h5="$DEST/demo_thread3_full_rollout_$i.h5"
    [ -e "$h5" ] || continue
    echo "Creating video for $h5"
    python "$VISUALIZE" --demo_number "$h5" --video_only
done
