#!/usr/bin/env bash
set -euo pipefail

# unthread4_rollout_0	Failure	(--trim_first_end_frames=1000)
# unthread4_rollout_1	Success
# unthread4_rollout_2	Failure	(--trim_first_end_frames=30)
# unthread4_rollout_3	Failure	(--trim_first_end_frames=40)
# unthread4_rollout_4	Failure
# unthread4_rollout_5	Failure
# unthread4_rollout_6	Failure	(--trim_first_end_frames=30)
# unthread4_rollout_7	Failure
# unthread4_rollout_8	Failure	(--trim_first_end_frames=20)
# unthread4_rollout_9	Failure	(--trim_first_end_frames=200)
# unthread4_rollout_10	Failure	(--trim_first_end_frames=300)
# unthread4_rollout_11	Pure Dagger only
# unthread4_rollout_12	Pure Dagger only
# unthread4_rollout_13	Pure Dagger only
# unthread4_rollout_14	Pure Dagger only
# unthread4_rollout_15	Pure Dagger only
# unthread4_rollout_16	Pure Dagger only
# unthread4_rollout_17	Pure Dagger only
# unthread4_rollout_18	Pure Dagger only
# unthread4_rollout_19	Pure Dagger only
# unthread4_rollout_20	Pure Dagger only
# unthread4_rollout_21	Pure Dagger only
# unthread4_rollout_22	Pure Dagger only
# unthread4_rollout_23	Pure Dagger only
# unthread4_rollout_24	Pure Dagger only
# unthread4_rollout_25	Pure Dagger only
# unthread4_rollout_26	Pure Dagger only
# unthread4_rollout_27	Pure Dagger only
# unthread4_rollout_28	Pure Dagger only
# unthread4_rollout_29	Pure Dagger only
# unthread4_rollout_30	Pure Dagger only
# unthread4_rollout_31	Pure Dagger only
# unthread4_rollout_32	Pure Dagger only
# unthread4_rollout_33	Pure Dagger only
# unthread4_rollout_34	Pure Dagger only
# unthread4_rollout_35	Pure Dagger only
# unthread4_rollout_36	Pure Dagger only
# unthread4_rollout_37	Pure Dagger only
# unthread4_rollout_38	Pure Dagger only
# unthread4_rollout_39	Pure Dagger only
# unthread4_rollout_40	Pure Dagger only
# unthread4_rollout_41	Pure Dagger only
# unthread4_rollout_42	Pure Dagger only
# unthread4_rollout_43	Pure Dagger only
# unthread4_rollout_44	Pure Dagger only
# unthread4_rollout_45	Pure Dagger only
# unthread4_rollout_46	Pure Dagger only
# unthread4_rollout_47	Pure Dagger only
# unthread4_rollout_48	Pure Dagger only
# unthread4_rollout_49	Pure Dagger only
# unthread4_rollout_50	Pure Dagger only
# unthread4_rollout_51	Pure Dagger only
# unthread4_rollout_52	Pure Dagger only
# unthread4_rollout_53	Pure Dagger only
# unthread4_rollout_54	Pure Dagger only
# unthread4_rollout_55	Pure Dagger only
# unthread4_rollout_56	Pure Dagger only
# unthread4_rollout_57	Pure Dagger only
# unthread4_rollout_58	Pure Dagger only
# unthread4_rollout_59	Pure Dagger only
# unthread4_rollout_60	Pure Dagger only
# unthread4_rollout_61	Pure Dagger only
# unthread4_rollout_62	Pure Dagger only
# unthread4_rollout_63	Pure Dagger only
# unthread4_rollout_64	Pure Dagger only
# unthread4_rollout_65	Pure Dagger only
# unthread4_rollout_66	Pure Dagger only
# unthread4_rollout_67	Pure Dagger only
# unthread4_rollout_68	Pure Dagger only
# unthread4_rollout_69	Pure Dagger only
# unthread4_rollout_70	Pure Dagger only
# unthread4_rollout_71	Pure Dagger only
# unthread4_rollout_72	Pure Dagger only
# unthread4_rollout_73	Pure Dagger only
# unthread4_rollout_74	Pure Dagger only

ROOT=/data3/extracted_data/unthread4_rollouts_w_dagger
DEST="$ROOT/full_rollout_h5_files"
SPLICE=/home/jeremiah/openteach/splice_demos.py
VISUALIZE=/home/jeremiah/openteach/visualize_demo.py
mkdir -p "$DEST"

move_full_h5() {
    local src="$1"
    local out="$2"
    local label="$3"

    if [ -e "$out" ]; then
        echo "Skipping $label; output already exists: $out"
        return
    fi
    echo "Moving $label -> $out"
    mv "$src" "$out"
}

splice_full_h5() {
    local i="$1"
    local trim="$2"
    local rollout="$ROOT/demonstration_unthread4_rollout_$i/demo_unthread4_rollout_$i.h5"
    local dagger="$ROOT/demonstration_unthread4_dagger_$i/demo_unthread4_dagger_$i.h5"
    local out="$DEST/demo_unthread4_full_rollout_$i.h5"

    if [ -e "$out" ]; then
        echo "Skipping spliced rollout $i; output already exists: $out"
        return
    fi

    echo "Splicing rollout $i + dagger $i -> $out"
    python "$SPLICE" "$rollout" "$dagger" "unthread4_full_rollout_$i" --data_root "$DEST" --trim_first_end_frames "$trim" --overwrite
    mv "$DEST/demonstration_unthread4_full_rollout_$i/demo_unthread4_full_rollout_$i.h5" "$out"
    rmdir "$DEST/demonstration_unthread4_full_rollout_$i"
}

move_full_h5 \
    "$ROOT/demonstration_unthread4_rollout_1/demo_unthread4_rollout_1.h5" \
    "$DEST/demo_unthread4_full_rollout_1.h5" \
    "successful rollout 1"

splice_full_h5 0 1000
splice_full_h5 2 30
splice_full_h5 3 40
splice_full_h5 4 0
splice_full_h5 5 0
splice_full_h5 6 30
splice_full_h5 7 0
splice_full_h5 8 20
splice_full_h5 9 200
splice_full_h5 10 300

for i in {11..74}; do
    move_full_h5 \
        "$ROOT/demonstration_unthread4_dagger_$i/demo_unthread4_dagger_$i.h5" \
        "$DEST/demo_unthread4_full_rollout_$i.h5" \
        "pure dagger rollout $i"
done

for i in {70..74}; do
    h5="$DEST/demo_unthread4_full_rollout_$i.h5"
    [ -e "$h5" ] || continue
    echo "Creating video for $h5"
    python "$VISUALIZE" --demo_number "$h5" --video_only
done
