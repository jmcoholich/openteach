#!/bin/bash
#SBATCH --job-name=viz_unthread4
#SBATCH -p kira-lab
#SBATCH -A kira-lab
#SBATCH -c 20
#SBATCH --mem=48G
#SBATCH --qos=short
#SBATCH --array=0-24
#SBATCH -o slurm_logs/viz_unthread4_%A_%a.out
#SBATCH -e slurm_logs/viz_unthread4_%A_%a.err
#SBATCH -x irona,calculon
set -euox pipefail

source /coc/testnvme/$USER/.bashrc
conda activate lerobot
export PATH="$CONDA_PREFIX/bin:$PATH"
unset LD_PRELOAD
which ffmpeg
cd ~/openteach

echo "Running demo unthread4_${SLURM_ARRAY_TASK_ID}"
python visualize_demo.py --demo_num /coc/testnvme/jcoholich3/openteach/extracted_data/unthread4/demonstration_unthread4_${SLURM_ARRAY_TASK_ID}
