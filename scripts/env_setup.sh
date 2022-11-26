# Expects to be called from this directory
cd ../
_cwd=$(pwd)

# Remove the existing conda environment
if [ -d "$_cwd/.conda" ]
then
    echo "Removing existing virtual environment"
    rm -rf $_cwd/.conda/
fi

# Use the project's environment file to initialize this instance
conda env create --file lib/Conda/project/QuadcopterDrone/environment.yml --prefix ./.conda

# Ensure the current shell can run conda activate
# https://stackoverflow.com/a/65183109/8341975
source ~/anaconda3/etc/profile.d/conda.sh
conda activate $_cwd/.conda

# Add local python tooling
conda develop $_cwd
conda develop $_cwd/src/core/com/serial
conda develop $_cwd/lib/Aurora/lib/nanopb/nanopb/generator