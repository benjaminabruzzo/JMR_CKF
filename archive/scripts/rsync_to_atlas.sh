# nohup program &> ~/nohup.out &
# nohup bash ~/ros/src/metahast/scripts/rsync_to_atlas.sh &> ~/rsync_to_atlas.out &

nohup rsync -avI --exclude='*.png' -e ssh ~/ros/data/20200805/ benjamin@atlas.local:/media/benjamin/evo2/20200805/ 





# printf "\n\nalias sshargo='ssh benjamin@argo.local'" >> ~/.bashrc
# printf "\nalias sshatlas='ssh benjamin@atlas.local'" >> ~/.bashrc
# printf "\nalias sshhermes='ssh benjamin@hermes.local'" >> ~/.bashrc
# printf "\nalias sshpompei='ssh benjamin@pompei.local'" >> ~/.bashrc
# printf "\nalias sshtitan='ssh benjamin@titan.local'" >> ~/.bashrc