#!/usr/bin/env bash
set -euo pipefail

# plug3_rollout_0	Success
# plug3_rollout_1	Success
# plug3_rollout_2	Success
# plug3_rollout_3	Success
# plug3_rollout_4	Failure
# plug3_rollout_5	Partial Success
# plug3_rollout_6	Success
# plug3_rollout_7	Partial Success
# plug3_rollout_8	Failure
# plug3_rollout_9	Success
# plug3_rollout_10	Failure
# plug3_rollout_11	Failure
# plug3_rollout_12	Failure
# plug3_rollout_13	Failure
# plug3_rollout_14	Success
# plug3_rollout_15	Failure
# plug3_rollout_16	Failure
# plug3_rollout_17	Success
# plug3_rollout_18	Failure
# plug3_rollout_19	Failure
# plug3_rollout_20	Success
# plug3_rollout_21	Partial Success
# plug3_rollout_22	Failure
# plug3_rollout_23	Failure
# plug3_rollout_24	Failure
# plug3_rollout_25	Success
# plug3_rollout_26	Failure
# plug3_rollout_27	Success
# plug3_rollout_28	Success
# plug3_rollout_29	Failure
# plug3_rollout_30	Failure
# plug3_rollout_31	Failure
# plug3_rollout_32	Failure
# plug3_rollout_33	Success
# plug3_rollout_34	Failure
# plug3_rollout_35	Success
# plug3_rollout_36	Success
# plug3_rollout_37	Failure
# plug3_rollout_38	Failure
# plug3_rollout_39	Failure
# plug3_rollout_40	Failure
# plug3_rollout_41	Failure
# plug3_rollout_42	Success
# plug3_rollout_43	Partial Success
# plug3_rollout_44	Failure
# plug3_rollout_45	Success
# plug3_rollout_46	Failure
# plug3_rollout_47	Failure
# plug3_rollout_48	Success
# plug3_rollout_49	Failure

ROOT=/home/jeremiah/openteach/extracted_data/plug3_rollouts_w_dagger
DEST="$ROOT/full_rollout_h5_files"
SPLICE=/home/jeremiah/openteach/splice_demos.py
mkdir -p "$DEST"

success=(0 1 2 3 6 9 14 17 20 25 27 28 33 35 36 42 45 48)
needs_dagger=(4 5 7 8 10 11 12 13 15 16 18 19 21 22 23 24 26 29 30 31 32 34 37 38 39 40 41 43 44 46 47 49)

for i in "${success[@]}"; do
    src="$ROOT/demonstration_plug3_rollout_$i/demo_plug3_rollout_$i.h5"
    out="$DEST/demo_plug3_full_rollout_$i.h5"
    echo "Copying successful rollout $i -> $out"
    cp -n "$src" "$out"
done

for i in "${needs_dagger[@]}"; do
    rollout="$ROOT/demonstration_plug3_rollout_$i/demo_plug3_rollout_$i.h5"
    dagger="$ROOT/demonstration_plug3_dagger_$i/demo_plug3_dagger_$i.h5"
    out="$DEST/demo_plug3_full_rollout_$i.h5"
    if [ "$i" = 26 ]; then
        extra_dagger="$ROOT/demonstration_plug3_dagger_26__2/demo_plug3_dagger_26__2.h5"
        tmp_name="plug3_full_rollout_26_part1"
        tmp="$DEST/demonstration_$tmp_name/demo_$tmp_name.h5"
        echo "Splicing rollout 26 + dagger 26 + dagger 26__2 -> $out"
        python "$SPLICE" "$rollout" "$dagger" "$tmp_name" --data_root "$DEST"
        python "$SPLICE" "$tmp" "$extra_dagger" "plug3_full_rollout_26" --data_root "$DEST"
        mv "$DEST/demonstration_plug3_full_rollout_26/demo_plug3_full_rollout_26.h5" "$out"
        rm -r "$DEST/demonstration_$tmp_name" "$DEST/demonstration_plug3_full_rollout_26"
        continue
    fi
    echo "Splicing rollout $i + dagger $i -> $out"
    python "$SPLICE" "$rollout" "$dagger" "plug3_full_rollout_$i" --data_root "$DEST"
    mv "$DEST/demonstration_plug3_full_rollout_$i/demo_plug3_full_rollout_$i.h5" "$out"
    rmdir "$DEST/demonstration_plug3_full_rollout_$i"
done

for h5 in "$DEST"/demo_plug3_full_rollout_*.h5; do
    [ -e "$h5" ] || continue
    echo "Creating video for $h5"
    python /home/jeremiah/openteach/visualize_demo.py --demo_number "$h5" --video_only
done
