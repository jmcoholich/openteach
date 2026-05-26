#!/usr/bin/env bash
set -euo pipefail

# nutegunray_plug4_rollout_1	nutegunray_plug4_dagger_1
# nutegunray_plug4_rollout_2	nutegunray_plug4_dagger_2
# nutegunray_plug4_rollout_3	success
# nutegunray_plug4_rollout_4	nutegunray_plug4_dagger_4
# nutegunray_plug4_rollout_5	success
# nutegunray_plug4_rollout_6	nutegunray_plug4_dagger_6
# nutegunray_plug4_rollout_7	nutegunray_plug4_dagger_7
# nutegunray_plug4_rollout_8	nutegunray_plug4_dagger_8
# nutegunray_plug4_rollout_9	nutegunray_plug4_dagger_9
# nutegunray_plug4_rollout_10	nutegunray_plug4_dagger_10
# nutegunray_plug4_rollout_11	success
# nutegunray_plug4_rollout_12	success
# nutegunray_plug4_rollout_13	success
# nutegunray_plug4_rollout_14	success
# nutegunray_plug4_rollout_15	success
# nutegunray_plug4_rollout_16	nutegunray_plug4_dagger_16
# nutegunray_plug4_rollout_17	nutegunray_plug4_dagger_17
# nutegunray_plug4_rollout_18	nutegunray_plug4_dagger_18
# nutegunray_plug4_rollout_19	nutegunray_plug4_dagger_19
# nutegunray_plug4_rollout_20	nutegunray_plug4_dagger_20
# nutegunray_plug4_rollout_21	success
# nutegunray_plug4_rollout_22	nutegunray_plug4_dagger_22
# nutegunray_plug4_rollout_23	success
# nutegunray_plug4_rollout_24	nutegunray_plug4_dagger_24
# nutegunray_plug4_rollout_25	nutegunray_plug4_dagger_25
# nutegunray_plug4_rollout_26	nutegunray_plug4_dagger_26
# nutegunray_plug4_rollout_27	success
# nutegunray_plug4_rollout_28	success
# nutegunray_plug4_rollout_29	nutegunray_plug4_dagger_29
# nutegunray_plug4_rollout_30	nutegunray_plug4_dagger_30
# nutegunray_plug4_rollout_31	nutegunray_plug4_dagger_31
# nutegunray_plug4_rollout_32	nutegunray_plug4_dagger_32
# nutegunray_plug4_rollout_33	nutegunray_plug4_dagger_33
# nutegunray_plug4_rollout_34	nutegunray_plug4_dagger_34
# nutegunray_plug4_rollout_35	nutegunray_plug4_dagger_35
# nutegunray_plug4_rollout_36	success
# nutegunray_plug4_rollout_37	success
# nutegunray_plug4_rollout_38	nutegunray_plug4_dagger_38
# nutegunray_plug4_rollout_39	nutegunray_plug4_dagger_39
# nutegunray_plug4_rollout_40	nutegunray_plug4_dagger_40
# nutegunray_plug4_rollout_41	nutegunray_plug4_dagger_41
# nutegunray_plug4_rollout_42	success
# nutegunray_plug4_rollout_43	nutegunray_plug4_dagger_43
# nutegunray_plug4_rollout_44	success
# nutegunray_plug4_rollout_45	nutegunray_plug4_dagger_45
# nutegunray_plug4_rollout_46	success
# nutegunray_plug4_rollout_47	nutegunray_plug4_dagger_47
# nutegunray_plug4_rollout_48	nutegunray_plug4_dagger_48
# nutegunray_plug4_rollout_49	nutegunray_plug4_dagger_49
# nutegunray_plug4_rollout_50	success

ROOT=/home/jeremiah/openteach/extracted_data/nutegunray_plug4_rollouts_w_dagger
DEST="$ROOT/full_rollout_h5_files"
SPLICE=/home/jeremiah/openteach/splice_demos.py
VISUALIZE=/home/jeremiah/openteach/visualize_demo.py
mkdir -p "$DEST"

success=(3 5 11 12 13 14 15 21 23 27 28 36 37 42 44 46 50)
needs_dagger=(1 2 4 6 7 8 9 10 16 17 18 19 20 22 24 25 26 29 30 31 32 33 34 35 38 39 40 41 43 45 47 48 49)

for i in "${success[@]}"; do
    src="$ROOT/demonstration_nutegunray_plug4_rollout_$i/demo_nutegunray_plug4_rollout_$i.h5"
    out="$DEST/demo_nutegunray_plug4_full_rollout_$i.h5"
    echo "Moving successful rollout $i -> $out"
    mv -n "$src" "$out"
done

for i in "${needs_dagger[@]}"; do
    rollout="$ROOT/demonstration_nutegunray_plug4_rollout_$i/demo_nutegunray_plug4_rollout_$i.h5"
    dagger="$ROOT/demonstration_nutegunray_plug4_dagger_$i/demo_nutegunray_plug4_dagger_$i.h5"
    out="$DEST/demo_nutegunray_plug4_full_rollout_$i.h5"
    echo "Splicing rollout $i + dagger $i -> $out"
    python "$SPLICE" "$rollout" "$dagger" "nutegunray_plug4_full_rollout_$i" --data_root "$DEST"
    mv "$DEST/demonstration_nutegunray_plug4_full_rollout_$i/demo_nutegunray_plug4_full_rollout_$i.h5" "$out"
    rmdir "$DEST/demonstration_nutegunray_plug4_full_rollout_$i"
done

# for h5 in "$DEST"/demo_nutegunray_plug4_full_rollout_*.h5; do
#     [ -e "$h5" ] || continue
#     echo "Creating video for $h5"
#     python "$VISUALIZE" --demo_number "$h5" --video_only
# done
