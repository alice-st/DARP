if [ "$#" -ne 1 ]; then
  echo "Give only one argument, the name of the virtual environment to create"
  exit
fi

python3 -m venv $1

source $1/bin/activate

pip install \
	opencv-python \
	pygame \
	scipy
