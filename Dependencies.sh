if [ "$#" -ne 1 ]; then
  echo "Give only one argument, the name of the virtual environment to create"
  exit
fi

python3 -m venv $1

source $1/bin/activate

pip install \
	opencv-python==4.5.4.60 \
	pygame==2.1.0 \
	scipy==1.7.3 \
	jupyter==1.0.0 \
  numba==0.54.1