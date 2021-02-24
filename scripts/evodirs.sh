# DATE="20151001"
# cd ~/ros/src/metahast && bash ./scripts/evodirs.sh 20200917 evo2
# cd ~/ros/src/metahast && bash ./scripts/evodirs.sh 20200914 archive

DATE=$1
DRIVE=$2

FOLDER="/mnt/${DRIVE}"
echo "${FOLDER}/${DATE}/A"; mkdir -p "${FOLDER}/${DATE}/A"
echo "${FOLDER}/${DATE}/B"; mkdir -p "${FOLDER}/${DATE}/B"
echo "${FOLDER}/${DATE}/G"; mkdir -p "${FOLDER}/${DATE}/G"
echo "${FOLDER}/${DATE}/H"; mkdir -p "${FOLDER}/${DATE}/H"
echo "${FOLDER}/${DATE}/I"; mkdir -p "${FOLDER}/${DATE}/I"
echo "${FOLDER}/${DATE}/J"; mkdir -p "${FOLDER}/${DATE}/J"

echo "${FOLDER}/${DATE}/figs/all_evd"; mkdir -p "${FOLDER}/${DATE}/figs/all_evd"
echo "${FOLDER}/${DATE}/figs/all_evt"; mkdir -p "${FOLDER}/${DATE}/figs/all_evt"
echo "${FOLDER}/${DATE}/figs/e_vs_d"; mkdir -p "${FOLDER}/${DATE}/figs/e_vs_d"
