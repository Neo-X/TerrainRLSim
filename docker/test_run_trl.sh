docker build -f Dockerfile -t terrainrl:latest .

# docker build --default-runtime=nvidia -f Dockerfile_trl -t rlframe_trl:latest .
# docker run --rm --runtime=nvidia -v /opt/OpenGL/:/usr/lib/nvidia -it rlframe_trl:latest /bin/bash -c "pushd /root/playground/RL-Framework; git pull origin master; python3 trainModel.py --config=settings/MiniGrid/TagEnv/PPO/Tag_SLAC_mini.json -p 4 --bootstrap_samples=2000 --max_epoch_length=16 --rollouts=4 --skip_rollouts=true --train_actor=false --train_critic=false --epochs=32 --fd_updates_per_actor_update=64 --on_policy=fast"

### Not using nvidia/cuda for now
docker build -f Dockerfile_trl -t terrainrl:latest .
echo "RL_FRAMEWORK_PATH"
echo "$RL_FRAMEWORK_PATH"
echo "arg 1 $1"
# cmd='python3 trainModel.py --config=settings/MiniGrid/TagEnv/PPO/Tag_Dual_FullObserve_SLAC_mini.json  -p 2 --bootstrap_samples=10000 --max_epoch_length=32 --rollouts=32 --pretrain_fd=0 --plot=false --save_video_to_file=eval.mp4 --metaConfig=settings/hyperParamTuning/element/exploration_rate.json --experiment_logging="{\"use_comet\": true, \"project_name\": \"ic2\"}"'
cmd=$1
fullcmd="pushd /root/playground/TerrainRLSim; git pull origin master; popd; pushd /root/playground/RLSimulationEnvironments; git pull origin master; popd; pushd /root/playground/RL-Framework; git pull origin master; ${cmd}"
echo $fullcmd
command=(docker run --rm -v $RL_FRAMEWORK_PATH:/root/playground/RL-Framework -it rlframe_trl:latest /bin/bash -c "$fullcmd" )
echo "${command[@]}"
# eval $command
"${command[@]}"
