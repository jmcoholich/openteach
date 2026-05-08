#!/usr/bin/env bash
set -euo pipefail

# unplug3_rollout_0	Success
# unplug3_rollout_1	Success
# unplug3_rollout_2	Success
# unplug3_rollout_3	Success
# unplug3_rollout_4	Success
# unplug3_rollout_5	Success
# unplug3_rollout_6	Failure  (--trim_first_end_frames=20)
# unplug3_rollout_7	Success
# unplug3_rollout_8	Failure
# unplug3_rollout_9	Success
# unplug3_rollout_10	Success
# unplug3_rollout_11	Success
# unplug3_rollout_12	Failure
# unplug3_rollout_13	Failure
# unplug3_rollout_14	Success
# unplug3_rollout_15	Success
# unplug3_rollout_16	Failure
# unplug3_rollout_17	Success
# unplug3_rollout_18	Success
# unplug3_rollout_19	Failure
# unplug3_rollout_20	Success
# unplug3_rollout_21	Success
# unplug3_rollout_22	Failure
# unplug3_rollout_23	Failure
# unplug3_rollout_24	Failure
# unplug3_rollout_25	Failure
# unplug3_rollout_26	Failure (collected an additional dagger -> __2)
# unplug3_rollout_27	Failure
# unplug3_rollout_28	Partial Success
# unplug3_rollout_29	Failure
# unplug3_rollout_30	Partial Success
# unplug3_rollout_31	Success
# unplug3_rollout_32	Failure
# unplug3_rollout_33	Failure
# unplug3_rollout_34	Failure
# unplug3_rollout_35	Failure
# unplug3_rollout_36	Partial Success
# unplug3_rollout_37	Failure
# unplug3_rollout_38	Failure
# unplug3_rollout_39	Partial Success
# unplug3_rollout_40	Partial Success
# unplug3_rollout_41	Partial Success
# unplug3_rollout_42	Partial Success
# unplug3_rollout_43	Partial Success
# unplug3_rollout_44	Partial Success
# unplug3_rollout_45	Failure
# unplug3_rollout_46	Failure
# unplug3_rollout_47	Partial Success
# unplug3_rollout_48	Failure
# unplug3_rollout_49	Partial Success

ROOT=/data3/extracted_data/unplug3_rollouts_w_dagger
DEST="$ROOT/full_rollout_h5_files"
SPLICE=/home/jeremiah/openteach/splice_demos.py
VISUALIZE=/home/jeremiah/openteach/visualize_demo.py
mkdir -p "$DEST"

success=(0 1 2 3 4 5 7 9 10 11 14 15 17 18 20 21 31)
needs_dagger=(6 8 12 13 16 19 22 23 24 25 26 27 28 29 30 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49)

for i in "${success[@]}"; do
    src="$ROOT/demonstration_unplug3_rollout_$i/demo_unplug3_rollout_$i.h5"
    out="$DEST/demo_unplug3_full_rollout_$i.h5"
    echo "Copying successful rollout $i -> $out"
    cp -n "$src" "$out"
done

for i in "${needs_dagger[@]}"; do
    rollout="$ROOT/demonstration_unplug3_rollout_$i/demo_unplug3_rollout_$i.h5"
    dagger="$ROOT/demonstration_unplug3_dagger_$i/demo_unplug3_dagger_$i.h5"
    out="$DEST/demo_unplug3_full_rollout_$i.h5"

    if [ "$i" = 6 ]; then
        echo "Splicing rollout 6 + dagger 6 with --trim_first_end_frames=20 -> $out"
        python "$SPLICE" "$rollout" "$dagger" "unplug3_full_rollout_6" --data_root "$DEST" --trim_first_end_frames 20
        mv "$DEST/demonstration_unplug3_full_rollout_6/demo_unplug3_full_rollout_6.h5" "$out"
        rmdir "$DEST/demonstration_unplug3_full_rollout_6"
        continue
    fi

    if [ "$i" = 26 ]; then
        extra_dagger="$ROOT/demonstration_unplug3_dagger_26__2/demo_unplug3_dagger_26__2.h5"
        tmp_name="unplug3_full_rollout_26_part1"
        tmp="$DEST/demonstration_$tmp_name/demo_$tmp_name.h5"
        echo "Splicing rollout 26 + dagger 26 + dagger 26__2 -> $out"
        python "$SPLICE" "$rollout" "$dagger" "$tmp_name" --data_root "$DEST"
        python "$SPLICE" "$tmp" "$extra_dagger" "unplug3_full_rollout_26" --data_root "$DEST"
        mv "$DEST/demonstration_unplug3_full_rollout_26/demo_unplug3_full_rollout_26.h5" "$out"
        rm -r "$DEST/demonstration_$tmp_name" "$DEST/demonstration_unplug3_full_rollout_26"
        continue
    fi

    echo "Splicing rollout $i + dagger $i -> $out"
    python "$SPLICE" "$rollout" "$dagger" "unplug3_full_rollout_$i" --data_root "$DEST"
    mv "$DEST/demonstration_unplug3_full_rollout_$i/demo_unplug3_full_rollout_$i.h5" "$out"
    rmdir "$DEST/demonstration_unplug3_full_rollout_$i"
done

for h5 in "$DEST"/demo_unplug3_full_rollout_*.h5; do
    [ -e "$h5" ] || continue
    echo "Creating video for $h5"
    python "$VISUALIZE" --demo_number "$h5" --video_only
done
